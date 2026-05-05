/*
 *  ros_camera_server - Intelligent camera stream server.
 *  Copyright (C) 2026  Stefan Fabian
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ros_camera_server/inputs/v4l2_input.hpp"
#include "../logging.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace ros_camera_server
{

std::shared_ptr<V4l2InputConfiguration>
V4l2InputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<V4l2InputConfiguration>();
  result->type = "v4l2";
  result->loadSharedFromYaml( config );
  result->device = config["device"].as<std::string>();
  result->media_type = config["media_type"].as<std::string>( "" );
  result->width = config["width"].as<int>( -1 );
  result->height = config["height"].as<int>( -1 );

  if ( config["controls"] && config["controls"].IsMap() ) {
    for ( const auto &kvp : config["controls"] ) {
      result->controls[kvp.first.as<std::string>()] = kvp.second.as<int>();
    }
  }
  return result;
}

namespace
{
StreamFormat getStreamFormat( GstCaps *caps )
{
  if ( gst_caps_is_empty( caps ) ) {
    SERVER_LOG_ERROR( "No suitable caps found" );
    return StreamFormat::RAW;
  }

  const GstStructure *structure = gst_caps_get_structure( caps, 0 );
  const gchar *media_type = gst_structure_get_name( structure );

  if ( g_strcmp0( media_type, "image/jpeg" ) == 0 ) {
    return StreamFormat::JPEG;
  } else if ( g_strcmp0( media_type, "image/png" ) == 0 ) {
    return StreamFormat::PNG;
  } else if ( g_strcmp0( media_type, "video/x-h264" ) == 0 ) {
    return StreamFormat::H264;
  } else if ( g_strcmp0( media_type, "video/x-h265" ) == 0 ) {
    return StreamFormat::H265;
  }
  return StreamFormat::RAW;
}

// ── V4L2 control helpers ────────────────────────────────────────────────────

// Normalize V4L2 control name to GStreamer extra-controls key format
std::string toGstControlKey( const char *name )
{
  std::string result;
  bool last_underscore = false;
  for ( const char *p = name; *p; ++p ) {
    if ( std::isalnum( static_cast<unsigned char>( *p ) ) ) {
      result += static_cast<char>( std::tolower( static_cast<unsigned char>( *p ) ) );
      last_underscore = false;
    } else if ( !last_underscore ) {
      result += '_';
      last_underscore = true;
    }
  }
  while ( !result.empty() && result.back() == '_' ) { result.pop_back(); }
  return result;
}

struct V4l2ControlDescriptor {
  std::string gst_key;
  std::string display_name;
  __u32 type;
  int32_t min;
  int32_t max;
  int32_t step;
  int32_t default_value;
  int32_t current_value;
};

std::vector<V4l2ControlDescriptor> enumerateV4l2Controls( const std::string &device_path )
{
  std::vector<V4l2ControlDescriptor> descriptors;

  int fd = open( device_path.c_str(), O_RDWR );
  if ( fd < 0 ) {
    SERVER_LOG_WARN( "Cannot open %s for control enumeration", device_path.c_str() );
    return descriptors;
  }

  struct v4l2_queryctrl qctrl{};
  qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while ( ioctl( fd, VIDIOC_QUERYCTRL, &qctrl ) == 0 ) {
    if ( qctrl.flags & V4L2_CTRL_FLAG_DISABLED ) {
      qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      continue;
    }

    // Only expose settable integer, boolean, and menu controls
    bool is_settable = !( qctrl.flags & V4L2_CTRL_FLAG_READ_ONLY );
    bool is_supported_type =
        qctrl.type == V4L2_CTRL_TYPE_INTEGER || qctrl.type == V4L2_CTRL_TYPE_BOOLEAN ||
        qctrl.type == V4L2_CTRL_TYPE_MENU || qctrl.type == V4L2_CTRL_TYPE_INTEGER_MENU;

    if ( is_settable && is_supported_type ) {
      V4l2ControlDescriptor desc;
      desc.gst_key = toGstControlKey( reinterpret_cast<const char *>( qctrl.name ) );
      desc.display_name = reinterpret_cast<const char *>( qctrl.name );
      desc.type = qctrl.type;
      desc.min = qctrl.minimum;
      desc.max = qctrl.maximum;
      desc.step = qctrl.step;
      desc.default_value = qctrl.default_value;

      // Read the current value from the device
      struct v4l2_control ctrl{};
      ctrl.id = qctrl.id & ~V4L2_CTRL_FLAG_NEXT_CTRL;
      if ( ioctl( fd, VIDIOC_G_CTRL, &ctrl ) == 0 ) {
        desc.current_value = ctrl.value;
      } else {
        desc.current_value = qctrl.default_value;
      }

      descriptors.push_back( desc );
    }

    qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }

  close( fd );
  return descriptors;
}

// ── Runtime state for V4L2 parameter callbacks ──────────────────────────────

struct V4l2InputState {
  GstElement *v4l2src = nullptr;
  std::vector<V4l2ControlDescriptor> control_descriptors;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  void applyControls( const std::map<std::string, int32_t> &values )
  {
    if ( !v4l2src || values.empty() )
      return;

    GstStructure *extra = gst_structure_new_empty( "v4l2-controls" );
    for ( const auto &[key, value] : values ) {
      gst_structure_set( extra, key.c_str(), G_TYPE_INT, value, nullptr );
    }
    g_object_set( v4l2src, "extra-controls", extra, nullptr );
    gst_structure_free( extra );
  }
};

} // namespace

StreamInput V4l2InputConfiguration::createInput( const rclcpp::Node::SharedPtr &node,
                                                 const std::string &camera_id ) const
{
  GstElement *src = gst_element_factory_make( "v4l2src", "input" );
  if ( !src ) {
    SERVER_LOG_ERROR( "Failed to create v4l2src element for device: %s", device.c_str() );
    return {};
  }
  g_object_set( G_OBJECT( src ), "device", device.c_str(), "do-timestamp", TRUE, nullptr );

  // Apply initial controls from config before caps negotiation
  if ( !controls.empty() ) {
    GstStructure *extra = gst_structure_new_empty( "v4l2-controls" );
    for ( const auto &[key, value] : controls ) {
      gst_structure_set( extra, key.c_str(), G_TYPE_INT, value, nullptr );
    }
    g_object_set( src, "extra-controls", extra, nullptr );
    gst_structure_free( extra );
    SERVER_LOG_INFO( "Applied %zu initial V4L2 controls for device %s", controls.size(),
                     device.c_str() );
  }

  GstCaps *caps = selectCaps( src );
  if ( !caps ) {
    SERVER_LOG_ERROR( "Failed to select suitable caps for device: %s", device.c_str() );
    gst_object_unref( src );
    return {};
  }
  gchar *caps_str = gst_caps_to_string( caps );
  SERVER_LOG_INFO( "Selected caps for device %s: %s", device.c_str(), caps_str );
  g_free( caps_str );
  GstElement *capsfilter = gst_element_factory_make( "capsfilter", "capsfilter" );
  if ( !capsfilter ) {
    SERVER_LOG_ERROR( "Failed to create capsfilter element for device: %s", device.c_str() );
    gst_caps_unref( caps );
    gst_object_unref( src );
    return {};
  }
  g_object_set( G_OBJECT( capsfilter ), "caps", caps, nullptr );
  StreamFormat stream_format = getStreamFormat( caps );
  gst_caps_unref( caps );

  auto *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  gst_bin_add_many( input_bin, src, capsfilter, nullptr );
  if ( !gst_element_link_many( src, capsfilter, nullptr ) ) {
    SERVER_LOG_ERROR( "Failed to link elements in input bin for V4L2 input: %s", device.c_str() );
    gst_object_unref( input_bin );
    return {};
  }
  GstPad *input_pad = gst_element_get_static_pad( capsfilter, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", input_pad ) );
  gst_object_unref( input_pad );

  // ── Enumerate controls and register ROS 2 parameters ──────────────────────
  auto state = std::make_shared<V4l2InputState>();
  state->v4l2src = src; // Element is owned by the bin, so this pointer is valid while the bin lives.

  state->control_descriptors = enumerateV4l2Controls( device );
  if ( !state->control_descriptors.empty() && node ) {
    SERVER_LOG_INFO( "Registering %zu V4L2 control parameters for device %s",
                     state->control_descriptors.size(), device.c_str() );

    std::string param_prefix = camera_id + ".controls.";

    for ( const auto &desc : state->control_descriptors ) {
      std::string param_name = param_prefix + desc.gst_key;

      // Use current device value; config overrides take priority
      int32_t initial_value = desc.current_value;
      if ( auto it = controls.find( desc.gst_key ); it != controls.end() ) {
        initial_value = it->second;
      }

      rcl_interfaces::msg::ParameterDescriptor param_desc;
      param_desc.name = param_name;
      param_desc.description = desc.display_name;

      if ( desc.type == V4L2_CTRL_TYPE_BOOLEAN ) {
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        node->declare_parameter( param_name, initial_value != 0, param_desc );
      } else {
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = desc.min;
        range.to_value = desc.max;
        range.step = desc.step;
        param_desc.integer_range.push_back( range );
        node->declare_parameter( param_name, static_cast<int64_t>( initial_value ), param_desc );
      }

      SERVER_LOG_DEBUG( "Declared parameter %s [%d..%d] = %d", param_name.c_str(), desc.min,
                        desc.max, initial_value );
    }

    // Register parameter change callback
    state->param_callback_handle = node->add_on_set_parameters_callback(
        [state_weak = std::weak_ptr<V4l2InputState>( state ), prefix = param_prefix](
            const std::vector<rclcpp::Parameter> &parameters ) -> rcl_interfaces::msg::SetParametersResult {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          auto state = state_weak.lock();
          if ( !state ) {
            result.successful = false;
            result.reason = "V4L2 input no longer active";
            return result;
          }

          std::map<std::string, int32_t> changed_controls;
          for ( const auto &param : parameters ) {
            if ( param.get_name().rfind( prefix, 0 ) != 0 )
              continue; // Not our parameter

            std::string control_key = param.get_name().substr( prefix.size() );

            // Validate control exists
            bool found = false;
            for ( const auto &desc : state->control_descriptors ) {
              if ( desc.gst_key == control_key ) {
                found = true;
                int32_t value;
                if ( desc.type == V4L2_CTRL_TYPE_BOOLEAN ) {
                  value = param.as_bool() ? 1 : 0;
                } else {
                  value = static_cast<int32_t>( param.as_int() );
                }
                changed_controls[control_key] = value;
                break;
              }
            }
            if ( !found ) {
              // Not a control we manage - ignore (let other callbacks handle it)
              continue;
            }
          }

          if ( !changed_controls.empty() ) {
            state->applyControls( changed_controls );
            for ( const auto &[key, value] : changed_controls ) {
              SERVER_LOG_INFO( "Applied V4L2 control %s = %d", key.c_str(), value );
            }
          }

          return result;
        } );
  }

  return { input_bin, stream_format, state };
}

GstCaps *V4l2InputConfiguration::selectCaps( GstElement *src ) const
{
  if ( gst_element_set_state( src, GST_STATE_READY ) != GST_STATE_CHANGE_SUCCESS ) {
    return nullptr;
  }

  GstPad *source_pad = gst_element_get_static_pad( GST_ELEMENT( src ), "src" );
  GstCaps *available_caps = gst_pad_query_caps( source_pad, nullptr );
  gst_object_unref( source_pad );

  if ( width <= 0 || height <= 0 || !framerate.isValid() ) {
    gst_caps_unref( available_caps );
    gst_element_set_state( GST_ELEMENT( src ), GST_STATE_NULL );
    throw PipelineBuildError(
        "V4L2 input for device '" + device +
        "' requires explicit width, height, and framerate. "
        "Run 'ros2 run ros_camera_server v4l2_caps_investigator' to see available caps." );
    return nullptr;
  }

  SERVER_LOG_DEBUG_STREAM( "Looking for exact caps: " << width << "x" << height << " @ "
                                                      << framerate.toString() << " with media type "
                                                      << media_type );

  // Collect all caps that exactly match the requested size and framerate.
  // The format/color space is left unspecified so GStreamer can negotiate it downstream.
  GstCaps *result_caps = gst_caps_new_empty();

  for ( guint i = 0; i < gst_caps_get_size( available_caps ); ++i ) {
    const GstStructure *structure = gst_caps_get_structure( available_caps, i );

    // Check media type filter
    if ( const gchar *type = gst_structure_get_name( structure );
         !media_type.empty() && g_strcmp0( type, media_type.c_str() ) != 0 ) {
      continue;
    }

    // Check width - must be fixed and matching, or a range containing the desired value
    const GValue *width_value = gst_structure_get_value( structure, "width" );
    if ( !width_value ) {
      continue;
    } else if ( G_VALUE_HOLDS_INT( width_value ) ) {
      if ( g_value_get_int( width_value ) != width )
        continue;
    } else if ( GST_VALUE_HOLDS_INT_RANGE( width_value ) ) {
      int min_w = gst_value_get_int_range_min( width_value );
      int max_w = gst_value_get_int_range_max( width_value );
      int step_w = gst_value_get_int_range_step( width_value );
      if ( width < min_w || width > max_w || ( step_w > 1 && ( width - min_w ) % step_w != 0 ) )
        continue;
    } else {
      continue;
    }

    // Check height - same logic
    const GValue *height_value = gst_structure_get_value( structure, "height" );
    if ( !height_value ) {
      continue;
    } else if ( G_VALUE_HOLDS_INT( height_value ) ) {
      if ( g_value_get_int( height_value ) != height )
        continue;
    } else if ( GST_VALUE_HOLDS_INT_RANGE( height_value ) ) {
      int min_h = gst_value_get_int_range_min( height_value );
      int max_h = gst_value_get_int_range_max( height_value );
      int step_h = gst_value_get_int_range_step( height_value );
      if ( height < min_h || height > max_h || ( step_h > 1 && ( height - min_h ) % step_h != 0 ) )
        continue;
    } else {
      continue;
    }

    // Check framerate - must be fixed and matching, or a range containing the desired value
    if ( gst_structure_has_field( structure, "framerate" ) ) {
      const GValue *fr_value = gst_structure_get_value( structure, "framerate" );
      if ( GST_VALUE_HOLDS_FRACTION( fr_value ) ) {
        Framerate cap_fr;
        gst_structure_get_fraction( structure, "framerate", &cap_fr.numerator, &cap_fr.denominator );
        if ( cap_fr != framerate )
          continue;
      } else if ( GST_VALUE_HOLDS_FRACTION_RANGE( fr_value ) ) {
        const GValue *min_fr = gst_value_get_fraction_range_min( fr_value );
        const GValue *max_fr = gst_value_get_fraction_range_max( fr_value );
        Framerate min_framerate( gst_value_get_fraction_numerator( min_fr ),
                                 gst_value_get_fraction_denominator( min_fr ) );
        Framerate max_framerate( gst_value_get_fraction_numerator( max_fr ),
                                 gst_value_get_fraction_denominator( max_fr ) );
        if ( framerate < min_framerate || framerate > max_framerate )
          continue;
      } else {
        continue;
      }
    }

    // This caps structure matches - fix up ranges to exact values and aggregate
    GstStructure *fixed = gst_structure_copy( structure );
    gst_structure_set( fixed, "width", G_TYPE_INT, width, "height", G_TYPE_INT, height, nullptr );
    gst_structure_set( fixed, "framerate", GST_TYPE_FRACTION, framerate.numerator,
                       framerate.denominator, nullptr );
    gchar *fixed_str = gst_structure_to_string( fixed );
    SERVER_LOG_DEBUG( "Matched caps: %s", fixed_str );
    g_free( fixed_str );
    gst_caps_append_structure( result_caps, fixed );
  }

  gst_caps_unref( available_caps );
  gst_element_set_state( GST_ELEMENT( src ), GST_STATE_NULL );

  if ( gst_caps_is_empty( result_caps ) ) {
    gst_caps_unref( result_caps );
    throw PipelineBuildError(
        "No caps matching " + std::to_string( width ) + "x" + std::to_string( height ) + " @ " +
        framerate.toString() + " for device '" + device +
        "' requires explicit width, height, and framerate. "
        "Run 'ros2 run ros_camera_server v4l2_caps_investigator' to see available caps." );
    return nullptr;
  }

  return result_caps;
}
} // namespace ros_camera_server
