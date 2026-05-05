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

#include "ros_camera_server/outputs/ros2_output.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"

#include <gst/video/video.h>
#include <limits>
#include <unordered_map>

namespace ros_camera_server
{

namespace
{
// Mapping from ROS encoding to GStreamer format
static const std::unordered_map<std::string, GstVideoFormat> ros_to_gst_map = {
    { "rgb8", GST_VIDEO_FORMAT_RGB },
    { "bgr8", GST_VIDEO_FORMAT_BGR },
    { "rgba8", GST_VIDEO_FORMAT_RGBA },
    { "bgra8", GST_VIDEO_FORMAT_BGRA },
    { "mono8", GST_VIDEO_FORMAT_GRAY8 },
    { "mono16", GST_VIDEO_FORMAT_GRAY16_LE },
    { "uyvy", GST_VIDEO_FORMAT_UYVY },
    { "yuv422", GST_VIDEO_FORMAT_UYVY }, // deprecated alias
    { "yuyv", GST_VIDEO_FORMAT_YUY2 },
    { "yuv422_yuy2", GST_VIDEO_FORMAT_YUY2 }, // deprecated alias
    { "nv21", GST_VIDEO_FORMAT_NV21 },
    { "nv24", GST_VIDEO_FORMAT_NV24 },
    // OpenCV type aliases
    { "8UC1", GST_VIDEO_FORMAT_GRAY8 },
    { "8UC3", GST_VIDEO_FORMAT_RGB },  // Assume RGB for 3-channel
    { "8UC4", GST_VIDEO_FORMAT_RGBA }, // Assume RGBA for 4-channel
    { "16UC1", GST_VIDEO_FORMAT_GRAY16_LE },
};
} // namespace

class Ros2Output : public PipelineOutput
{
public:
  using PipelineOutput::PipelineOutput;

  void build( int index, const Ros2OutputConfiguration &config );

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const override;

  int getClientCount() const override;

private:
  std::string topic_;
  std::string codec_;
  std::string frame_id_;
  GstElement *ros_sink_ = nullptr;
  int configured_width_ = 0;
  int configured_height_ = 0;
  float configured_fps_ = 0;
};

int Ros2Output::getClientCount() const
{
  if ( ros_sink_ == nullptr )
    return 0;
  gint subscriber_count = -1;
  g_object_get( G_OBJECT( ros_sink_ ), "subscription-count", &subscriber_count, nullptr );
  return subscriber_count;
}

ros_camera_server_msgs::msg::CameraStream
Ros2Output::toCameraStreamMsg( const CameraServerConfiguration & ) const
{
  ros_camera_server_msgs::msg::CameraStream msg;
  msg.transport = Ros2OutputConfiguration::TYPE;
  msg.uri = topic_;

  // Query runtime caps, fall back to configured values
  auto stats = const_cast<Ros2Output *>( this )->statistics();
  auto codec = stats.codec.empty() ? codec_ : stats.codec;
  if ( codec == "raw" ) {
    msg.codec = "raw";
  } else if ( codec == "jpeg" || codec == "png" ) {
    msg.codec = "compressed";
  } else {
    msg.codec = codec; // unknown/unsupported codec, just pass it through
  }
  msg.width = stats.width > 0 ? stats.width : configured_width_;
  msg.height = stats.height > 0 ? stats.height : configured_height_;
  msg.framerate = stats.fps > 0 ? stats.fps : configured_fps_;

  return msg;
}

Ros2OutputConfiguration::Ros2OutputConfiguration() : OutputConfiguration( TYPE ) { }

std::shared_ptr<Ros2OutputConfiguration>
Ros2OutputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<Ros2OutputConfiguration>();
  result->loadSharedFromYaml( config );
  result->topic = config["topic"].as<std::string>();
  result->codec = config["codec"].as<std::string>( "auto" );
  if ( result->codec == "auto" ) {
    result->supported_input_formats = { StreamFormat::RAW, StreamFormat::JPEG, StreamFormat::PNG };
  } else if ( result->codec == "raw" ) {
    result->supported_input_formats = { StreamFormat::RAW };
  } else if ( result->codec == "compressed" ) {
    result->supported_input_formats = { StreamFormat::JPEG, StreamFormat::PNG };
  } else {
    throw PipelineBuildError( "Invalid codec type for ros2 output: " + result->codec );
  }
  result->format = config["format"].as<std::string>( "" );
  result->frame_id = config["frame_id"].as<std::string>( "" );
  result->camera_info_url = config["camera_info_url"].as<std::string>( "" );
  return result;
}

YAML::Node Ros2OutputConfiguration::toYaml() const
{
  YAML::Node result;
  result["type"] = "ros2";
  result["topic"] = topic;
  result["codec"] = codec;
  if ( !frame_id.empty() )
    result["frame_id"] = frame_id;
  if ( !format.empty() )
    result["format"] = format;
  if ( !camera_info_url.empty() )
    result["camera_info_url"] = camera_info_url;
  if ( framerate.isValid() )
    result["framerate"] = framerate.toString();
  if ( width != std::numeric_limits<int>::max() )
    result["width"] = width;
  if ( height != std::numeric_limits<int>::max() )
    result["height"] = height;
  return result;
}

PipelineOutput::Ptr Ros2OutputConfiguration::createOutput( const rclcpp::Node::SharedPtr &node,
                                                           const std::string &camera_id,
                                                           int index ) const
{
  auto output = std::make_unique<Ros2Output>( node, camera_id, index );
  output->build( index, *this );
  return output;
}

void Ros2Output::build( int index, const Ros2OutputConfiguration &config )
{
  topic_ = config.topic;
  frame_id_ = config.frame_id;
  // config.codec for ROS2 is the transport type (auto/raw/compressed), not a media codec.
  // Use selected_input_format which was resolved by the pipeline graph.
  codec_ = to_string( config.selected_input_format );
  if ( config.width > 0 && config.width != std::numeric_limits<int>::max() )
    configured_width_ = config.width;
  if ( config.height > 0 && config.height != std::numeric_limits<int>::max() )
    configured_height_ = config.height;
  configured_fps_ = static_cast<float>( config.framerate.toFps() );

  std::string name = "ros_output_bin_" + std::to_string( index );
  GstBin *ros_output_bin = GST_BIN( gst_bin_new( name.c_str() ) );
  GstElement *queue_ros = gst_element_factory_make( "queue", ( name + "_queue" ).c_str() );
  g_object_set( G_OBJECT( queue_ros ), "max-size-buffers", 1, "leaky", 2 /* downstream */, nullptr );

  ros_sink_ = gst_element_factory_make( "rbfimagesink", ( name + "_output" ).c_str() );
  if ( !ros_sink_ ) {
    gst_object_unref( ros_output_bin );
    throw PipelineBuildError( "Failed to create rbfimagesink element for ROS2 output" );
  }
  assert( node().get() != nullptr );
  g_object_set( G_OBJECT( ros_sink_ ), "node", (gpointer)node().get(), "topic",
                config.topic.c_str(), "frame-id", config.frame_id.c_str(), "camera-info-url",
                config.camera_info_url.c_str(), "async", FALSE, nullptr );

  GstElement *capsfilter = nullptr;
  GstElement *videoconvert = nullptr;
  if ( codec_ == "raw" && !config.format.empty() ) {
    // Convert ROS encoding to GStreamer format and create capsfilter
    auto it = ros_to_gst_map.find( config.format );
    if ( it == ros_to_gst_map.end() ) {
      gst_object_unref( ros_output_bin );
      throw PipelineBuildError( "Invalid ROS encoding format for ros2 output: " + config.format );
    }
    const char *format_str = gst_video_format_to_string( it->second );
    if ( !format_str ) {
      gst_object_unref( ros_output_bin );
      throw PipelineBuildError( "Failed to convert ROS encoding to GStreamer format string: " +
                                config.format );
    }
    GstCaps *caps =
        gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, format_str, nullptr );
    capsfilter = gst_element_factory_make( "capsfilter", ( name + "_capsfilter" ).c_str() );
    if ( !capsfilter ) {
      gst_caps_unref( caps );
      gst_object_unref( ros_output_bin );
      throw PipelineBuildError( "Failed to create capsfilter element for ROS2 output" );
    }
    g_object_set( G_OBJECT( capsfilter ), "caps", caps, nullptr );
    gst_caps_unref( caps );
    // Insert videoconvert so that upstream formats/memory features (e.g. NV12 +
    // memory:VAMemory from a HW scaler) are converted to the requested sysmem RGB/etc.
    videoconvert = gst_element_factory_make( "videoconvert", ( name + "_videoconvert" ).c_str() );
    if ( !videoconvert ) {
      gst_object_unref( ros_output_bin );
      throw PipelineBuildError( "Failed to create videoconvert element for ROS2 output" );
    }
  }

  if ( capsfilter ) {
    gst_bin_add_many( ros_output_bin, queue_ros, videoconvert, capsfilter, ros_sink_, nullptr );
    gst_element_link_many( queue_ros, videoconvert, capsfilter, ros_sink_, nullptr );
  } else {
    gst_bin_add_many( ros_output_bin, queue_ros, ros_sink_, nullptr );
    gst_element_link_many( queue_ros, ros_sink_, nullptr );
  }
  GstPad *ros_input_pad = gst_element_get_static_pad( queue_ros, "sink" );
  gst_element_add_pad( GST_ELEMENT( ros_output_bin ), gst_ghost_pad_new( "sink", ros_input_pad ) );
  gst_object_unref( ros_input_pad );

  GstPad *ros_sink_pad = gst_element_get_static_pad( ros_sink_, "sink" );
  gst_pad_add_probe(
      ros_sink_pad, GstPadProbeType( GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST ),
      []( GstPad *, GstPadProbeInfo *info, gpointer user_data ) -> GstPadProbeReturn {
        auto *self = static_cast<Ros2Output *>( user_data );
        if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER_LIST ) {
          GstBufferList *buffer_list = GST_PAD_PROBE_INFO_BUFFER_LIST( info );
          for ( guint i = 0; i < gst_buffer_list_length( buffer_list ); ++i ) {
            GstBuffer *buffer = gst_buffer_list_get( buffer_list, i );
            self->reportBufferSent( buffer );
          }
        } else if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
          GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
          self->reportBufferSent( buffer );
        }
        return GST_PAD_PROBE_OK;
      },
      this, nullptr );
  gst_object_unref( ros_sink_pad );
  bin = ros_output_bin;
}

} // namespace ros_camera_server
