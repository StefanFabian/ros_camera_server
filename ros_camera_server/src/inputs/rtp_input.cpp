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

#include "ros_camera_server/inputs/rtp_input.hpp"
#include "../helpers/video_codec_info.hpp"
#include "../logging.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

namespace ros_camera_server
{

std::shared_ptr<RtpInputConfiguration>
RtpInputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<RtpInputConfiguration>();
  result->type = "rtp";
  result->codec = config["codec"].as<std::string>( "" );
  VideoCodecInfo info = lookupVideoCodec( result->codec );
  if ( info.format == StreamFormat::INVALID || info.rtp_depayloader == nullptr ||
       info.rtp_encoding_name == nullptr ) {
    throw ConfigurationLoadError( "rtp input: 'codec' must be one of h264|h265|jpeg, got '" +
                                  result->codec + "'" );
  }
  if ( !config["port"] ) {
    throw ConfigurationLoadError( "rtp input: 'port' is required" );
  }
  result->port = config["port"].as<int>();
  result->address = config["address"].as<std::string>( "0.0.0.0" );
  result->multicast = config["multicast"].as<bool>( false );
  result->payload_type = config["payload_type"].as<int>( 96 );
  result->latency_ms = config["latency_ms"].as<int>( 200 );
  result->drop_on_latency = config["drop_on_latency"].as<bool>( false );
  result->loadSharedFromYaml( config );
  return result;
}

StreamInput RtpInputConfiguration::createInput( const rclcpp::Node::SharedPtr &,
                                                const std::string & /*camera_id*/ ) const
{
  VideoCodecInfo info = lookupVideoCodec( codec );

  GstBin *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  GstElement *udpsrc = gst_element_factory_make( "udpsrc", "input_udpsrc" );
  GstElement *jitter = gst_element_factory_make( "rtpjitterbuffer", "input_jitter" );
  GstElement *depay = gst_element_factory_make( info.rtp_depayloader, "input_depay" );
  GstElement *parser =
      info.parser ? gst_element_factory_make( info.parser, "input_parser" ) : nullptr;

  if ( !udpsrc || !jitter || !depay || ( info.parser && !parser ) ) {
    SERVER_LOG_ERROR( "RTP input: failed to create elements (codec=%s)", codec.c_str() );
    if ( udpsrc )
      gst_object_unref( udpsrc );
    if ( jitter )
      gst_object_unref( jitter );
    if ( depay )
      gst_object_unref( depay );
    if ( parser )
      gst_object_unref( parser );
    gst_object_unref( input_bin );
    return { nullptr, StreamFormat::INVALID, {} };
  }

  // RTP video clock-rate is 90000 (RFC 3551).
  GstCaps *caps =
      gst_caps_new_simple( "application/x-rtp", "media", G_TYPE_STRING, "video", "encoding-name",
                           G_TYPE_STRING, info.rtp_encoding_name, "clock-rate", G_TYPE_INT, 90000,
                           "payload", G_TYPE_INT, payload_type, nullptr );
  g_object_set( G_OBJECT( udpsrc ), "port", port, "caps", caps, nullptr );
  gst_caps_unref( caps );
  if ( multicast ) {
    g_object_set( G_OBJECT( udpsrc ), "auto-multicast", TRUE, "multicast-group", address.c_str(),
                  nullptr );
  } else if ( !address.empty() && address != "0.0.0.0" ) {
    g_object_set( G_OBJECT( udpsrc ), "address", address.c_str(), nullptr );
  }

  g_object_set( G_OBJECT( jitter ), "latency", latency_ms, "drop-on-latency",
                drop_on_latency ? TRUE : FALSE, nullptr );

  if ( codec == "h264" || codec == "h265" ) {
    g_object_set( G_OBJECT( parser ), "config-interval", -1, nullptr );
  }

  // Probe before depay so we can read the RTP header extension.
  GstPad *jitter_src = gst_element_get_static_pad( jitter, "src" );
  gst_pad_add_probe( jitter_src, GST_PAD_PROBE_TYPE_BUFFER, extractRtpTimestampProbe, nullptr,
                     nullptr );
  gst_object_unref( jitter_src );

  if ( parser ) {
    gst_bin_add_many( input_bin, udpsrc, jitter, depay, parser, nullptr );
    if ( !gst_element_link_many( udpsrc, jitter, depay, parser, nullptr ) ) {
      SERVER_LOG_ERROR( "RTP input: failed to link elements" );
      gst_object_unref( input_bin );
      return { nullptr, StreamFormat::INVALID, {} };
    }
  } else {
    gst_bin_add_many( input_bin, udpsrc, jitter, depay, nullptr );
    if ( !gst_element_link_many( udpsrc, jitter, depay, nullptr ) ) {
      SERVER_LOG_ERROR( "RTP input: failed to link elements" );
      gst_object_unref( input_bin );
      return { nullptr, StreamFormat::INVALID, {} };
    }
  }

  GstElement *src_element = parser ? parser : depay;
  GstPad *src_pad = gst_element_get_static_pad( src_element, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( src_pad );

  return { input_bin, info.format, {} };
}

} // namespace ros_camera_server
