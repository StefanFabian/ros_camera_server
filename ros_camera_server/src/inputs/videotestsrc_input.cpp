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

#include "ros_camera_server/inputs/videotestsrc_input.hpp"
#include "../logging.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

#include <gst/video/video.h>

namespace ros_camera_server
{

std::shared_ptr<VideoTestSrcInputConfiguration>
VideoTestSrcInputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<VideoTestSrcInputConfiguration>();
  result->type = "videotestsrc";
  result->loadSharedFromYaml( config );
  result->pattern = config["pattern"].as<std::string>( "smpte" );
  result->format = config["format"].as<std::string>( "" );
  result->width = config["width"].as<int>( 640 );
  result->height = config["height"].as<int>( 480 );
  return result;
}

namespace
{
//! @return false if pattern is not a valid nick of the element's pattern enum.
bool setPattern( GstElement *src, const std::string &pattern )
{
  GParamSpec *pspec = g_object_class_find_property( G_OBJECT_GET_CLASS( src ), "pattern" );
  GEnumClass *enum_class = G_ENUM_CLASS( g_type_class_ref( pspec->value_type ) );
  bool valid = g_enum_get_value_by_nick( enum_class, pattern.c_str() ) != nullptr;
  if ( valid ) {
    gst_util_set_object_arg( G_OBJECT( src ), "pattern", pattern.c_str() );
  } else {
    std::stringstream nicks;
    for ( guint i = 0; i < enum_class->n_values; ++i ) {
      nicks << enum_class->values[i].value_nick << " ";
    }
    SERVER_LOG_ERROR( "Unknown videotestsrc pattern: %s. Valid patterns: %s", pattern.c_str(),
                      nicks.str().c_str() );
  }
  g_type_class_unref( enum_class );
  return valid;
}
} // namespace

StreamInput VideoTestSrcInputConfiguration::createInput( const rclcpp::Node::SharedPtr &,
                                                         const std::string & /*camera_id*/ ) const
{
  GstElement *src = gst_element_factory_make( "videotestsrc", "input" );
  if ( !src ) {
    SERVER_LOG_ERROR( "Failed to create videotestsrc element" );
    return {};
  }
  g_object_set( G_OBJECT( src ), "is-live", TRUE, nullptr );
  if ( !setPattern( src, pattern ) ) {
    gst_object_unref( src );
    return {};
  }

  GstCaps *caps = gst_caps_new_simple( "video/x-raw", "width", G_TYPE_INT, width, "height",
                                       G_TYPE_INT, height, nullptr );
  if ( !format.empty() ) {
    if ( gst_video_format_from_string( format.c_str() ) == GST_VIDEO_FORMAT_UNKNOWN ) {
      SERVER_LOG_ERROR( "Unknown videotestsrc format: %s. See GstVideoFormat for valid formats.",
                        format.c_str() );
      gst_caps_unref( caps );
      gst_object_unref( src );
      return {};
    }
    gst_caps_set_simple( caps, "format", G_TYPE_STRING, format.c_str(), nullptr );
  }
  if ( framerate.isValid() ) {
    gst_caps_set_simple( caps, "framerate", GST_TYPE_FRACTION, framerate.numerator,
                         framerate.denominator, nullptr );
  }
  GstElement *capsfilter = gst_element_factory_make( "capsfilter", "capsfilter" );
  g_object_set( G_OBJECT( capsfilter ), "caps", caps, nullptr );
  gst_caps_unref( caps );

  auto *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  gst_bin_add_many( input_bin, src, capsfilter, nullptr );
  if ( !gst_element_link( src, capsfilter ) ) {
    SERVER_LOG_ERROR( "Failed to link elements in input bin for videotestsrc input" );
    gst_object_unref( input_bin );
    return {};
  }
  GstPad *input_pad = gst_element_get_static_pad( capsfilter, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", input_pad ) );
  gst_object_unref( input_pad );

  return { input_bin, StreamFormat::RAW };
}
} // namespace ros_camera_server
