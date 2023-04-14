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

#include "ros_camera_server/outputs/srt_output.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include "ros_camera_server/helpers/rtp_timestamp_extension.hpp"

#include "../logging.hpp"
#include <atomic>
#include <gst/rtp/gstrtpbuffer.h>
#include <limits>

namespace ros_camera_server
{

class SrtOutput : public PipelineOutput
{
public:
  using PipelineOutput::PipelineOutput;
  ~SrtOutput() override;

  bool build( int index, const SrtOutputConfiguration &config );

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const override;

  int getClientCount() const override { return active_client_count_.load(); }

private:
  int port_ = 0;
  std::string codec_;
  int configured_width_ = 0;
  int configured_height_ = 0;
  float configured_fps_ = 0;
  GstElement *sink_ = nullptr;
  gulong caller_added_handler_ = 0;
  gulong caller_removed_handler_ = 0;
  std::atomic<int> active_client_count_{ 0 };

  static void onCallerAdded( GstElement *element, gint caller_socket, gpointer address,
                             gpointer user_data );
  static void onCallerRemoved( GstElement *element, gint caller_socket, gpointer address,
                               gpointer user_data );
};

SrtOutput::~SrtOutput()
{
  if ( sink_ && caller_added_handler_ )
    g_signal_handler_disconnect( sink_, caller_added_handler_ );
  if ( sink_ && caller_removed_handler_ )
    g_signal_handler_disconnect( sink_, caller_removed_handler_ );
}

ros_camera_server_msgs::msg::CameraStream
SrtOutput::toCameraStreamMsg( const CameraServerConfiguration &config ) const
{
  ros_camera_server_msgs::msg::CameraStream msg;
  msg.transport = SrtOutputConfiguration::TYPE;
  msg.uri = "srt://" + config.address + ":" + std::to_string( port_ );

  // Query runtime caps, fall back to configured values
  auto stats = const_cast<SrtOutput *>( this )->statistics();
  msg.codec = stats.codec.empty() ? codec_ : stats.codec;
  msg.width = stats.width > 0 ? stats.width : configured_width_;
  msg.height = stats.height > 0 ? stats.height : configured_height_;
  msg.framerate = stats.fps > 0 ? stats.fps : configured_fps_;

  return msg;
}

void SrtOutput::onCallerAdded( GstElement *, gint, gpointer, gpointer user_data )
{
  auto *self = static_cast<SrtOutput *>( user_data );
  self->active_client_count_++;
  SERVER_LOG_INFO( "SRT: Client connected to port %d. Total clients: %d", self->port_,
                   self->active_client_count_.load() );
}

void SrtOutput::onCallerRemoved( GstElement *, gint, gpointer, gpointer user_data )
{
  auto *self = static_cast<SrtOutput *>( user_data );
  if ( self->active_client_count_ > 0 )
    self->active_client_count_--;
  SERVER_LOG_INFO( "SRT: Client disconnected from port %d. Total clients: %d", self->port_,
                   self->active_client_count_.load() );
}

SrtOutputConfiguration::SrtOutputConfiguration() : OutputConfiguration( TYPE )
{
  // Defaults; from_yaml_shared narrows supported_input_formats to the user's codec.
  supported_input_formats = { StreamFormat::H264 };
  codec = "h264"; // Default codec
}

std::shared_ptr<SrtOutputConfiguration>
SrtOutputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<SrtOutputConfiguration>();
  result->codec = config["codec"].as<std::string>( "h264" );
  result->supported_input_formats = { stream_format_from_codec( result->codec ) };
  result->encoder = config["encoder"].as<std::string>( "auto" );
  result->framerate = Framerate( config["framerate"].as<std::string>( "" ) );
  if ( config["width"] )
    result->width = config["width"].as<int>();
  if ( config["height"] )
    result->height = config["height"].as<int>();
  if ( config["bitrate"] )
    result->bitrate = config["bitrate"].as<int>();
  if ( config["latency_ms"] )
    result->latency_ms = config["latency_ms"].as<int>();
  result->port = config["port"].as<int>();
  return result;
}

YAML::Node SrtOutputConfiguration::toYaml() const
{
  YAML::Node result;
  result["type"] = "srt";
  result["codec"] = codec;
  result["port"] = port;
  return result;
}

PipelineOutput::Ptr SrtOutputConfiguration::createOutput( const rclcpp::Node::SharedPtr &node,
                                                          const std::string &camera_id,
                                                          int index ) const
{
  auto output = std::make_unique<SrtOutput>( node, camera_id, index );
  if ( !output->build( index, *this ) )
    return nullptr;
  return output;
}

bool SrtOutput::build( int index, const SrtOutputConfiguration &config )
{
  port_ = config.port;
  codec_ = config.codec;
  if ( config.width > 0 && config.width != std::numeric_limits<int>::max() )
    configured_width_ = config.width;
  if ( config.height > 0 && config.height != std::numeric_limits<int>::max() )
    configured_height_ = config.height;
  configured_fps_ = static_cast<float>( config.framerate.toFps() );

  std::string name = "srt_output_bin_" + std::to_string( index );
  GstBin *srt_output_bin = GST_BIN( gst_bin_new( name.c_str() ) );

  GstElement *sink = gst_element_factory_make( "srtsink", ( name + "_output" ).c_str() );
  std::string uri = "srt://:" + std::to_string( config.port ) + "?mode=listener&messageapi=true";
  g_object_set( G_OBJECT( sink ), "wait-for-connection", FALSE, "async", FALSE, "sync", FALSE,
                "uri", uri.c_str(), nullptr );
  if ( config.latency_ms > 0 )
    g_object_set( G_OBJECT( sink ), "latency", config.latency_ms, nullptr );

  // Connect SRT client tracking signals
  sink_ = sink;
  caller_added_handler_ = g_signal_connect( sink, "caller-added", G_CALLBACK( onCallerAdded ), this );
  caller_removed_handler_ =
      g_signal_connect( sink, "caller-removed", G_CALLBACK( onCallerRemoved ), this );

  // Select RTP payloader based on codec
  GstElement *rtppay = nullptr;
  if ( config.codec == "h264" ) {
    rtppay = gst_element_factory_make( "rtph264pay", ( name + "_rtppay" ).c_str() );
    // config-interval=-1: emit SPS/PPS in-band with every IDR so late-joining SRT clients
    // (caller mode) can decode without an out-of-band SDP.
    g_object_set( G_OBJECT( rtppay ), "timestamp-offset", 0, "mtu", 1300, "aggregate-mode",
                  0 /* zero-latency */, "config-interval", -1, nullptr );
  } else if ( config.codec == "h265" ) {
    rtppay = gst_element_factory_make( "rtph265pay", ( name + "_rtppay" ).c_str() );
    g_object_set( G_OBJECT( rtppay ), "timestamp-offset", 0, "mtu", 1300, "config-interval", -1,
                  nullptr );
  } else {
    SERVER_LOG_ERROR( "SRT output: unsupported codec '%s'", config.codec.c_str() );
    gst_object_unref( srt_output_bin );
    return false;
  }

  if ( !rtppay ) {
    SERVER_LOG_ERROR( "Failed to create RTP payloader for codec: %s", config.codec.c_str() );
    gst_object_unref( srt_output_bin );
    return false;
  }

  // Transport-only pipeline: rtppay → srtsink
  gst_bin_add_many( srt_output_bin, rtppay, sink, nullptr );
  gst_element_link_many( rtppay, sink, nullptr );

  GstPad *srt_input_pad = gst_element_get_static_pad( rtppay, "sink" );
  gst_element_add_pad( GST_ELEMENT( srt_output_bin ), gst_ghost_pad_new( "sink", srt_input_pad ) );
  gst_object_unref( srt_input_pad );

  GstPad *srtsink_pad = gst_element_get_static_pad( sink, "sink" );
  gst_pad_add_probe(
      srtsink_pad, GstPadProbeType( GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST ),
      []( GstPad *, GstPadProbeInfo *info, gpointer user_data ) -> GstPadProbeReturn {
        auto *self = static_cast<SrtOutput *>( user_data );

        if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER_LIST ) {
          GstBufferList *buffer_list = GST_PAD_PROBE_INFO_BUFFER_LIST( info );
          buffer_list = gst_buffer_list_make_writable( buffer_list );
          gst_buffer_list_foreach(
              buffer_list,
              []( GstBuffer **buf, guint, gpointer user_data ) -> int {
                auto self = static_cast<SrtOutput *>( user_data );
                gint64 capture_time_ns = self->reportBufferSent( *buf );
                if ( capture_time_ns != 0 ) {
                  *buf = gst_buffer_make_writable( *buf );
                  if ( !rtp_buffer_add_timestamp_extension( *buf, capture_time_ns ) ) {
                    g_warning( "Failed to add timestamp extension to RTP packet" );
                  }
                }
                return TRUE;
              },
              self );
          GST_PAD_PROBE_INFO_DATA( info ) = buffer_list;
        } else if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
          GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
          gint64 capture_time_ns = self->reportBufferSent( buffer );
          if ( capture_time_ns != 0 ) {
            buffer = gst_buffer_make_writable( buffer );
            if ( !rtp_buffer_add_timestamp_extension( buffer, capture_time_ns ) ) {
              g_warning( "Failed to add timestamp extension to RTP packet" );
            }
            GST_PAD_PROBE_INFO_DATA( info ) = buffer;
          }
        }
        return GST_PAD_PROBE_OK;
      },
      this, nullptr );
  gst_object_unref( srtsink_pad );

  bin = srt_output_bin;
  return true;
}
} // namespace ros_camera_server
