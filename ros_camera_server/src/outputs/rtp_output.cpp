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

#include "ros_camera_server/outputs/rtp_output.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include "ros_camera_server/helpers/rtp_timestamp_extension.hpp"

#include "../logging.hpp"
#include <atomic>
#include <gio/gio.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <limits>

namespace ros_camera_server
{

namespace
{
const char *rtpPayloaderForCodec( const std::string &codec )
{
  if ( codec == "h264" )
    return "rtph264pay";
  if ( codec == "h265" )
    return "rtph265pay";
  if ( codec == "jpeg" )
    return "rtpjpegpay";
  if ( codec == "raw" )
    return "rtpvrawpay";
  return nullptr;
}
} // namespace

class RtpOutput : public PipelineOutput
{
public:
  using PipelineOutput::PipelineOutput;
  ~RtpOutput() override = default;

  bool build( int index, const RtpOutputConfiguration &config );

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const override;

private:
  std::string host_;
  int port_ = 0;
  std::string codec_;
  bool embed_capture_timestamp_ = true;
  int configured_width_ = 0;
  int configured_height_ = 0;
  float configured_fps_ = 0;
  // Failure to add the timestamp extension is a systemic condition (writable buffer
  // or RTP map fails). Log once per output, not once per packet, to avoid flooding.
  std::atomic<bool> extension_failure_warned_{ false };
};

RtpOutputConfiguration::RtpOutputConfiguration() : OutputConfiguration( TYPE )
{
  // Defaults; from_yaml_shared narrows supported_input_formats to the user's codec.
  supported_input_formats = { StreamFormat::H264 };
  codec = "h264";
}

std::shared_ptr<RtpOutputConfiguration>
RtpOutputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<RtpOutputConfiguration>();
  result->codec = config["codec"].as<std::string>( "h264" );
  if ( !rtpPayloaderForCodec( result->codec ) ) {
    throw ConfigurationLoadError( "Unsupported codec for rtp output: " + result->codec );
  }
  result->supported_input_formats = { stream_format_from_codec( result->codec ) };

  result->loadSharedFromYaml( config );

  if ( !config["host"] ) {
    throw ConfigurationLoadError( "rtp output: 'host' is required" );
  }
  result->host = config["host"].as<std::string>();
  if ( !config["port"] ) {
    throw ConfigurationLoadError( "rtp output: 'port' is required" );
  }
  result->port = config["port"].as<int>();
  result->multicast = config["multicast"].as<bool>( false );
  result->ttl = config["ttl"].as<int>( 16 );
  result->payload_type = config["payload_type"].as<int>( 96 );
  result->embed_capture_timestamp = config["embed_capture_timestamp"].as<bool>( true );
  return result;
}

YAML::Node RtpOutputConfiguration::toYaml() const
{
  YAML::Node result;
  result["type"] = TYPE;
  result["codec"] = codec;
  result["host"] = host;
  result["port"] = port;
  result["multicast"] = multicast;
  if ( multicast )
    result["ttl"] = ttl;
  result["payload_type"] = payload_type;
  result["embed_capture_timestamp"] = embed_capture_timestamp;
  return result;
}

PipelineOutput::Ptr RtpOutputConfiguration::createOutput( const rclcpp::Node::SharedPtr &node,
                                                          const std::string &camera_id,
                                                          int index ) const
{
  auto output = std::make_unique<RtpOutput>( node, camera_id, index );
  if ( !output->build( index, *this ) )
    return nullptr;
  return output;
}

ros_camera_server_msgs::msg::CameraStream
RtpOutput::toCameraStreamMsg( const CameraServerConfiguration & ) const
{
  ros_camera_server_msgs::msg::CameraStream msg;
  msg.transport = RtpOutputConfiguration::TYPE;
  msg.uri = "rtp://" + host_ + ":" + std::to_string( port_ );

  auto stats = const_cast<RtpOutput *>( this )->statistics();
  msg.codec = stats.codec.empty() ? codec_ : stats.codec;
  msg.width = stats.width > 0 ? stats.width : configured_width_;
  msg.height = stats.height > 0 ? stats.height : configured_height_;
  msg.framerate = stats.fps > 0 ? stats.fps : configured_fps_;
  return msg;
}

bool RtpOutput::build( int index, const RtpOutputConfiguration &config )
{
  host_ = config.host;
  port_ = config.port;
  codec_ = config.codec;
  embed_capture_timestamp_ = config.embed_capture_timestamp;
  if ( config.width > 0 && config.width != std::numeric_limits<int>::max() )
    configured_width_ = config.width;
  if ( config.height > 0 && config.height != std::numeric_limits<int>::max() )
    configured_height_ = config.height;
  configured_fps_ = static_cast<float>( config.framerate.toFps() );

  const char *pay_factory = rtpPayloaderForCodec( codec_ );
  if ( !pay_factory ) {
    SERVER_LOG_ERROR( "RTP output: unsupported codec '%s'", codec_.c_str() );
    return false;
  }

  std::string name = "rtp_output_bin_" + std::to_string( index );
  GstBin *output_bin = GST_BIN( gst_bin_new( name.c_str() ) );

  GstElement *pay = gst_element_factory_make( pay_factory, ( name + "_pay" ).c_str() );
  GstElement *rtpbin = gst_element_factory_make( "rtpbin", ( name + "_rtpbin" ).c_str() );
  GstElement *udp_data = gst_element_factory_make( "udpsink", ( name + "_udpsink_data" ).c_str() );
  GstElement *udp_rtcp = gst_element_factory_make( "udpsink", ( name + "_udpsink_rtcp" ).c_str() );
  GstElement *src_rtcp = gst_element_factory_make( "udpsrc", ( name + "_udpsrc_rtcp" ).c_str() );

  if ( !pay || !rtpbin || !udp_data || !udp_rtcp || !src_rtcp ) {
    SERVER_LOG_ERROR( "RTP output: failed to create elements" );
    if ( pay )
      gst_object_unref( pay );
    if ( rtpbin )
      gst_object_unref( rtpbin );
    if ( udp_data )
      gst_object_unref( udp_data );
    if ( udp_rtcp )
      gst_object_unref( udp_rtcp );
    if ( src_rtcp )
      gst_object_unref( src_rtcp );
    gst_object_unref( output_bin );
    return false;
  }

  // config-interval=-1 keeps SPS/PPS/VPS in-band on every IDR for codecs that support it,
  // which is necessary for receivers joining mid-stream. aggregate-mode=0 (zero-latency)
  // only exists on rtph264pay/rtph265pay.
  g_object_set( G_OBJECT( pay ), "timestamp-offset", 0, "mtu", 1300, "pt", config.payload_type,
                nullptr );
  if ( codec_ == "h264" || codec_ == "h265" ) {
    g_object_set( G_OBJECT( pay ), "aggregate-mode", 0, "config-interval", -1, nullptr );
  }

  // RTCP shares a single UDP socket between the recv-RR udpsrc and the send-RTCP udpsink,
  // so the RTCP source port equals port+1 and receivers' RRs land back on udpsrc
  // (RFC 3550). Bind on udpsrc first by transitioning it to READY, then hand its
  // socket to udpsink.
  g_object_set( G_OBJECT( src_rtcp ), "port", port_ + 1, "reuse", TRUE, nullptr );
  if ( config.multicast ) {
    g_object_set( G_OBJECT( src_rtcp ), "auto-multicast", TRUE, "multicast-group", host_.c_str(),
                  nullptr );
  }
  if ( gst_element_set_state( src_rtcp, GST_STATE_READY ) == GST_STATE_CHANGE_FAILURE ) {
    SERVER_LOG_ERROR( "RTP output: failed to bind RTCP socket on port %d", port_ + 1 );
    gst_element_set_state( src_rtcp, GST_STATE_NULL );
    gst_object_unref( pay );
    gst_object_unref( rtpbin );
    gst_object_unref( udp_data );
    gst_object_unref( udp_rtcp );
    gst_object_unref( src_rtcp );
    gst_object_unref( output_bin );
    return false;
  }
  GSocket *rtcp_socket = nullptr;
  g_object_get( G_OBJECT( src_rtcp ), "used-socket", &rtcp_socket, nullptr );
  if ( !rtcp_socket ) {
    SERVER_LOG_ERROR( "RTP output: udpsrc has no bound socket" );
    gst_element_set_state( src_rtcp, GST_STATE_NULL );
    gst_object_unref( pay );
    gst_object_unref( rtpbin );
    gst_object_unref( udp_data );
    gst_object_unref( udp_rtcp );
    gst_object_unref( src_rtcp );
    gst_object_unref( output_bin );
    return false;
  }

  // udp_rtcp reuses src_rtcp's socket; close-socket=FALSE keeps udpsrc as the sole
  // owner so it closes the fd on shutdown.
  g_object_set( G_OBJECT( udp_data ), "host", host_.c_str(), "port", port_, "sync", FALSE, "async",
                FALSE, nullptr );
  g_object_set( G_OBJECT( udp_rtcp ), "host", host_.c_str(), "port", port_ + 1, "socket",
                rtcp_socket, "close-socket", FALSE, "sync", FALSE, "async", FALSE, nullptr );
  g_object_unref( rtcp_socket );

  if ( config.multicast ) {
    g_object_set( G_OBJECT( udp_data ), "auto-multicast", TRUE, "ttl-mc", config.ttl, nullptr );
    // udpsrc already joined the RTCP multicast group on the shared socket;
    // udpsink only needs the outgoing TTL.
    g_object_set( G_OBJECT( udp_rtcp ), "auto-multicast", FALSE, "ttl-mc", config.ttl, nullptr );
  }

  gst_bin_add_many( output_bin, pay, rtpbin, udp_data, udp_rtcp, src_rtcp, nullptr );

  // pay → rtpbin.send_rtp_sink_0
  GstPad *pay_src = gst_element_get_static_pad( pay, "src" );
  GstPad *rtpbin_sink = gst_element_request_pad_simple( rtpbin, "send_rtp_sink_0" );
  if ( !pay_src || !rtpbin_sink || gst_pad_link( pay_src, rtpbin_sink ) != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "RTP output: failed to link payloader to rtpbin" );
    if ( pay_src )
      gst_object_unref( pay_src );
    if ( rtpbin_sink )
      gst_object_unref( rtpbin_sink );
    gst_object_unref( output_bin );
    return false;
  }
  gst_object_unref( pay_src );
  gst_object_unref( rtpbin_sink );

  // Requesting send_rtp_sink_0 creates send_rtp_src_0 immediately; link it now.
  GstPad *rtpbin_data_src = gst_element_get_static_pad( rtpbin, "send_rtp_src_0" );
  GstPad *udp_data_sink_pad = gst_element_get_static_pad( udp_data, "sink" );
  if ( !rtpbin_data_src || !udp_data_sink_pad ||
       gst_pad_link( rtpbin_data_src, udp_data_sink_pad ) != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "RTP output: failed to link rtpbin data source to udpsink" );
    if ( rtpbin_data_src )
      gst_object_unref( rtpbin_data_src );
    if ( udp_data_sink_pad )
      gst_object_unref( udp_data_sink_pad );
    gst_object_unref( output_bin );
    return false;
  }
  gst_object_unref( rtpbin_data_src );
  gst_object_unref( udp_data_sink_pad );

  // rtpbin.send_rtcp_src_0 → udp_rtcp
  GstPad *rtcp_src = gst_element_request_pad_simple( rtpbin, "send_rtcp_src_0" );
  GstPad *udp_rtcp_sink = gst_element_get_static_pad( udp_rtcp, "sink" );
  if ( !rtcp_src || !udp_rtcp_sink || gst_pad_link( rtcp_src, udp_rtcp_sink ) != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "RTP output: failed to link rtcp send to udpsink" );
    if ( rtcp_src )
      gst_object_unref( rtcp_src );
    if ( udp_rtcp_sink )
      gst_object_unref( udp_rtcp_sink );
    gst_object_unref( output_bin );
    return false;
  }
  gst_object_unref( rtcp_src );
  gst_object_unref( udp_rtcp_sink );

  // src_rtcp → rtpbin.recv_rtcp_sink_0  (RR feedback channel)
  GstPad *src_rtcp_src = gst_element_get_static_pad( src_rtcp, "src" );
  GstPad *recv_rtcp_sink = gst_element_request_pad_simple( rtpbin, "recv_rtcp_sink_0" );
  if ( !src_rtcp_src || !recv_rtcp_sink ||
       gst_pad_link( src_rtcp_src, recv_rtcp_sink ) != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "RTP output: failed to link rtcp recv to rtpbin" );
    if ( src_rtcp_src )
      gst_object_unref( src_rtcp_src );
    if ( recv_rtcp_sink )
      gst_object_unref( recv_rtcp_sink );
    gst_object_unref( output_bin );
    return false;
  }
  gst_object_unref( src_rtcp_src );
  gst_object_unref( recv_rtcp_sink );

  // ghost sink pad on payloader's sink
  GstPad *pay_sink = gst_element_get_static_pad( pay, "sink" );
  gst_element_add_pad( GST_ELEMENT( output_bin ), gst_ghost_pad_new( "sink", pay_sink ) );
  gst_object_unref( pay_sink );

  // Pad probe on the data udpsink: report stats and (optionally) embed capture time.
  GstPad *udp_data_sink = gst_element_get_static_pad( udp_data, "sink" );
  gst_pad_add_probe(
      udp_data_sink, GstPadProbeType( GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST ),
      []( GstPad *, GstPadProbeInfo *info, gpointer user_data ) -> GstPadProbeReturn {
        auto *self = static_cast<RtpOutput *>( user_data );

        if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER_LIST ) {
          GstBufferList *buffer_list = GST_PAD_PROBE_INFO_BUFFER_LIST( info );
          buffer_list = gst_buffer_list_make_writable( buffer_list );
          gst_buffer_list_foreach(
              buffer_list,
              []( GstBuffer **buf, guint, gpointer user_data ) -> int {
                auto *self = static_cast<RtpOutput *>( user_data );
                gint64 capture_time_ns = self->reportBufferSent( *buf );
                if ( self->embed_capture_timestamp_ && capture_time_ns != 0 ) {
                  *buf = gst_buffer_make_writable( *buf );
                  if ( !rtp_buffer_add_timestamp_extension( *buf, capture_time_ns ) &&
                       !self->extension_failure_warned_.exchange( true ) ) {
                    SERVER_LOG_WARN( "RTP output: failed to add timestamp extension; "
                                     "suppressing further occurrences." );
                  }
                }
                return TRUE;
              },
              self );
          GST_PAD_PROBE_INFO_DATA( info ) = buffer_list;
        } else if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
          GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
          gint64 capture_time_ns = self->reportBufferSent( buffer );
          if ( self->embed_capture_timestamp_ && capture_time_ns != 0 ) {
            buffer = gst_buffer_make_writable( buffer );
            if ( !rtp_buffer_add_timestamp_extension( buffer, capture_time_ns ) &&
                 !self->extension_failure_warned_.exchange( true ) ) {
              SERVER_LOG_WARN( "RTP output: failed to add timestamp extension; "
                               "suppressing further occurrences." );
            }
            GST_PAD_PROBE_INFO_DATA( info ) = buffer;
          }
        }
        return GST_PAD_PROBE_OK;
      },
      this, nullptr );
  gst_object_unref( udp_data_sink );

  bin = output_bin;
  return true;
}

} // namespace ros_camera_server
