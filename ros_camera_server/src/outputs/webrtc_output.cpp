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

#include "ros_camera_server/outputs/webrtc_output.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include "ros_camera_server/helpers/rtp_timestamp_extension.hpp"

#include "../logging.hpp"
#include "../webrtc/signaling_server.hpp"
#include "../webrtc/webrtc_peer.hpp"

#include <atomic>
#include <gst/rtp/gstrtpbuffer.h>
#include <limits>
#include <map>

namespace ros_camera_server
{

/// Returns the GStreamer RTP payloader factory name for a given codec.
/// Extensible: add new codecs here (e.g., vp8 -> rtpvp8pay, vp9 -> rtpvp9pay).
static const char *rtpPayloaderForCodec( const std::string &codec )
{
  if ( codec == "h264" )
    return "rtph264pay";
  if ( codec == "h265" )
    return "rtph265pay";
  // Future: if (codec == "vp8") return "rtpvp8pay";
  // Future: if (codec == "vp9") return "rtpvp9pay";
  return nullptr;
}

class WebrtcOutput : public PipelineOutput
{
public:
  using PipelineOutput::PipelineOutput;
  ~WebrtcOutput() override;

  bool build( int index, const WebrtcOutputConfiguration &config );

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const override;

  int getClientCount() const override { return static_cast<int>( sessions_.size() ); }

  /// Called by CameraServer after pipelines are built to wire up signaling.
  void setSignalingServer( SignalingServer *server, const std::string &path );

private:
  struct PeerSession {
    GstElement *queue = nullptr;
    GstElement *rtppay = nullptr;
    GstElement *webrtcbin = nullptr;
    std::unique_ptr<WebRTCPeer> peer;
    GstPad *tee_src_pad = nullptr;
  };

  GstElement *tee_ = nullptr;
  GstElement *fakesink_ = nullptr;
  GstElement *fakesink_queue_ = nullptr;
  std::string codec_;
  int configured_width_ = 0;
  int configured_height_ = 0;
  float configured_fps_ = 0;
  int index_ = 0;
  SignalingServer *signaling_server_ = nullptr;
  std::string signaling_path_;
  std::map<SoupWebsocketConnection *, PeerSession> sessions_;

  void onClientConnected( SoupWebsocketConnection *conn );
  void onClientMessage( SoupWebsocketConnection *conn, const nlohmann::json &msg );
  void onClientDisconnected( SoupWebsocketConnection *conn );

  void addPeerBranch( SoupWebsocketConnection *conn );
  void removePeerBranch( SoupWebsocketConnection *conn );

  /// Tear down a session's GStreamer state: release the tee request pad,
  /// set per-peer elements to NULL, remove them from the output bin.
  /// The session's `peer` is reset first so signaling handlers are detached.
  void tearDownSession( PeerSession &session );
};

// ============================================================================
// WebrtcOutputConfiguration
// ============================================================================

WebrtcOutputConfiguration::WebrtcOutputConfiguration() : OutputConfiguration( TYPE )
{
  // Defaults; from_yaml_shared narrows supported_input_formats to the user's codec.
  supported_input_formats = { StreamFormat::H264 };
  codec = "h264";
}

std::shared_ptr<WebrtcOutputConfiguration>
WebrtcOutputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<WebrtcOutputConfiguration>();
  result->loadSharedFromYaml( config );
  result->codec = config["codec"].as<std::string>( "h264" );
  result->supported_input_formats = { stream_format_from_codec( result->codec ) };
  return result;
}

YAML::Node WebrtcOutputConfiguration::toYaml() const
{
  YAML::Node result;
  result["type"] = "webrtc";
  result["codec"] = codec;
  return result;
}

PipelineOutput::Ptr WebrtcOutputConfiguration::createOutput( const rclcpp::Node::SharedPtr &node,
                                                             const std::string &camera_id,
                                                             int index ) const
{
  auto output = std::make_unique<WebrtcOutput>( node, camera_id, index );
  if ( !output->build( index, *this ) )
    return nullptr;
  return output;
}

// ============================================================================
// WebrtcOutput
// ============================================================================

WebrtcOutput::~WebrtcOutput()
{
  // Tear down any sessions still alive — the signaling close path may not
  // have run (async websocket close races with server shutdown), so we must
  // release request pads and remove peer elements from the bin here.
  for ( auto &[conn, session] : sessions_ ) { tearDownSession( session ); }
  sessions_.clear();
}

void WebrtcOutput::tearDownSession( PeerSession &session )
{
  // Detach signaling handlers before touching webrtcbin state.
  session.peer.reset();

  GstBin *output_bin = GST_BIN( bin );

  if ( session.tee_src_pad ) {
    GstPad *queue_sink_pad = gst_element_get_static_pad( session.queue, "sink" );
    if ( queue_sink_pad ) {
      gst_pad_unlink( session.tee_src_pad, queue_sink_pad );
      gst_object_unref( queue_sink_pad );
    }
    gst_element_release_request_pad( tee_, session.tee_src_pad );
    gst_object_unref( session.tee_src_pad );
    session.tee_src_pad = nullptr;
  }

  if ( session.webrtcbin )
    gst_element_set_state( session.webrtcbin, GST_STATE_NULL );
  if ( session.rtppay )
    gst_element_set_state( session.rtppay, GST_STATE_NULL );
  if ( session.queue )
    gst_element_set_state( session.queue, GST_STATE_NULL );

  if ( session.queue && session.rtppay && session.webrtcbin ) {
    gst_bin_remove_many( output_bin, session.queue, session.rtppay, session.webrtcbin, nullptr );
  }
  session.queue = nullptr;
  session.rtppay = nullptr;
  session.webrtcbin = nullptr;
}

ros_camera_server_msgs::msg::CameraStream
WebrtcOutput::toCameraStreamMsg( const CameraServerConfiguration &config ) const
{
  ros_camera_server_msgs::msg::CameraStream msg;
  msg.transport = WebrtcOutputConfiguration::TYPE;
  msg.codec = codec_;

  // Full WebSocket URI with signaling path
  msg.uri = "ws://" + config.address + ":" + std::to_string( config.signaling_port ) + "/" +
            cameraId() + "/" + std::to_string( outputIndex() );

  // Query runtime caps, fall back to configured values
  auto stats = const_cast<WebrtcOutput *>( this )->statistics();
  msg.width = stats.width > 0 ? stats.width : configured_width_;
  msg.height = stats.height > 0 ? stats.height : configured_height_;
  msg.framerate = stats.fps > 0 ? stats.fps : configured_fps_;

  return msg;
}

bool WebrtcOutput::build( int index, const WebrtcOutputConfiguration &config )
{
  index_ = index;
  codec_ = config.codec;
  if ( config.width > 0 && config.width != std::numeric_limits<int>::max() )
    configured_width_ = config.width;
  if ( config.height > 0 && config.height != std::numeric_limits<int>::max() )
    configured_height_ = config.height;
  configured_fps_ = static_cast<float>( config.framerate.toFps() );

  // Verify codec is supported
  if ( !rtpPayloaderForCodec( codec_ ) ) {
    SERVER_LOG_ERROR( "WebRTC output: unsupported codec '%s'", codec_.c_str() );
    return false;
  }

  std::string name = "webrtc_output_bin_" + std::to_string( index );
  GstBin *output_bin = GST_BIN( gst_bin_new( name.c_str() ) );

  // Create tee for fan-out to multiple viewers
  tee_ = gst_element_factory_make( "tee", ( name + "_tee" ).c_str() );
  g_object_set( G_OBJECT( tee_ ), "allow-not-linked", TRUE, nullptr );

  // Create a fakesink branch to keep the pipeline flowing when no clients are connected
  fakesink_queue_ = gst_element_factory_make( "queue", ( name + "_fakesink_queue" ).c_str() );
  g_object_set( G_OBJECT( fakesink_queue_ ), "max-size-buffers", 1, "leaky", 2 /* downstream */,
                nullptr );
  fakesink_ = gst_element_factory_make( "fakesink", ( name + "_fakesink" ).c_str() );
  g_object_set( G_OBJECT( fakesink_ ), "async", FALSE, "sync", FALSE, nullptr );

  if ( !tee_ || !fakesink_queue_ || !fakesink_ ) {
    SERVER_LOG_ERROR( "WebRTC output: failed to create pipeline elements" );
    gst_object_unref( output_bin );
    return false;
  }

  gst_bin_add_many( output_bin, tee_, fakesink_queue_, fakesink_, nullptr );
  gst_element_link_many( tee_, fakesink_queue_, fakesink_, nullptr );

  // Create ghost sink pad from tee's sink
  GstPad *tee_sink_pad = gst_element_get_static_pad( tee_, "sink" );
  gst_element_add_pad( GST_ELEMENT( output_bin ), gst_ghost_pad_new( "sink", tee_sink_pad ) );
  gst_object_unref( tee_sink_pad );

  bin = output_bin;
  return true;
}

void WebrtcOutput::setSignalingServer( SignalingServer *server, const std::string &path )
{
  signaling_server_ = server;
  signaling_path_ = path;

  SignalingServer::CameraInfo info;
  info.name = cameraId();
  info.codec = codec_;
  info.width = configured_width_;
  info.height = configured_height_;
  info.framerate = configured_fps_;

  server->registerEndpoint(
      path, info, [this]( SoupWebsocketConnection *conn ) { onClientConnected( conn ); },
      [this]( SoupWebsocketConnection *conn, const nlohmann::json &msg ) {
        onClientMessage( conn, msg );
      },
      [this]( SoupWebsocketConnection *conn ) { onClientDisconnected( conn ); } );
}

void WebrtcOutput::onClientConnected( SoupWebsocketConnection *conn )
{
  SERVER_LOG_INFO( "WebRTC: new client connected on %s", signaling_path_.c_str() );
  addPeerBranch( conn );
}

void WebrtcOutput::onClientMessage( SoupWebsocketConnection *conn, const nlohmann::json &msg )
{
  auto it = sessions_.find( conn );
  if ( it != sessions_.end() && it->second.peer ) {
    it->second.peer->handleSignalingMessage( msg );
  }
}

void WebrtcOutput::onClientDisconnected( SoupWebsocketConnection *conn )
{
  SERVER_LOG_INFO( "WebRTC: client disconnected from %s", signaling_path_.c_str() );
  removePeerBranch( conn );
}

void WebrtcOutput::addPeerBranch( SoupWebsocketConnection *conn )
{
  static std::atomic<int> peer_counter{ 0 };
  int peer_id = peer_counter.fetch_add( 1 );
  std::string peer_name = "webrtc_peer_" + std::to_string( peer_id );

  // Create per-peer elements
  GstElement *queue = gst_element_factory_make( "queue", ( peer_name + "_queue" ).c_str() );
  g_object_set( G_OBJECT( queue ), "max-size-time", (guint64)10'000'000 /* 100ms */,
                "max-size-buffers", 5, "max-size-bytes", 0, "leaky", 2 /* downstream */, nullptr );

  const char *pay_factory = rtpPayloaderForCodec( codec_ );
  GstElement *rtppay = gst_element_factory_make( pay_factory, ( peer_name + "_rtppay" ).c_str() );
  if ( codec_ == "h264" ) {
    g_object_set( G_OBJECT( rtppay ), "timestamp-offset", 0, "mtu", 1300, "aggregate-mode",
                  0 /* zero-latency */, "config-interval", -1, nullptr );
  } else if ( codec_ == "h265" ) {
    g_object_set( G_OBJECT( rtppay ), "timestamp-offset", 0, "mtu", 1300, "aggregate-mode",
                  0 /* zero-latency */, nullptr );
  }

  GstElement *webrtcbin =
      gst_element_factory_make( "webrtcbin", ( peer_name + "_webrtcbin" ).c_str() );
  g_object_set( G_OBJECT( webrtcbin ), "bundle-policy", 3 /* max-bundle */, nullptr );

  if ( !queue || !rtppay || !webrtcbin ) {
    SERVER_LOG_ERROR( "WebRTC: failed to create peer elements" );
    if ( queue )
      gst_object_unref( queue );
    if ( rtppay )
      gst_object_unref( rtppay );
    if ( webrtcbin )
      gst_object_unref( webrtcbin );
    return;
  }

  // Create the WebRTC peer BEFORE linking to webrtcbin, so the on-negotiation-needed
  // signal handler is connected before the link triggers it.
  auto peer = std::make_unique<WebRTCPeer>( WebRTCPeer::Role::OFFERER, conn, webrtcbin );

  // Add elements to the output bin (same bin as the tee so pad linking works)
  GstBin *output_bin = GST_BIN( bin );
  gst_bin_add_many( output_bin, queue, rtppay, webrtcbin, nullptr );
  gst_element_link( queue, rtppay );

  // Request a new src pad from the tee and link to the queue
  GstPad *tee_src_pad = gst_element_request_pad_simple( tee_, "src_%u" );
  GstPad *queue_sink_pad = gst_element_get_static_pad( queue, "sink" );
  GstPadLinkReturn link_ret = gst_pad_link( tee_src_pad, queue_sink_pad );
  gst_object_unref( queue_sink_pad );

  if ( link_ret != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "WebRTC: failed to link tee to queue (error: %d)", link_ret );
    gst_element_release_request_pad( tee_, tee_src_pad );
    gst_object_unref( tee_src_pad );
    peer.reset();
    gst_element_set_state( webrtcbin, GST_STATE_NULL );
    gst_element_set_state( rtppay, GST_STATE_NULL );
    gst_element_set_state( queue, GST_STATE_NULL );
    gst_bin_remove_many( output_bin, queue, rtppay, webrtcbin, nullptr );
    return;
  }

  // Sync states BEFORE linking to webrtcbin so that on-negotiation-needed fires
  // (webrtcbin only emits the signal when in PLAYING state)
  gst_element_sync_state_with_parent( queue );
  gst_element_sync_state_with_parent( rtppay );
  gst_element_sync_state_with_parent( webrtcbin );

  // Fixate the RTP caps before handing them to webrtcbin: rtph264pay/rtph265pay advertise
  // payload as a range [96,127] in their src template.
  // We fix it to 96 to prevent a warning from webrtc.
  const char *encoding_name = ( codec_ == "h265" ) ? "H265" : "H264";
  GstCaps *rtp_caps = gst_caps_new_simple( "application/x-rtp", "media", G_TYPE_STRING, "video",
                                           "encoding-name", G_TYPE_STRING, encoding_name, "payload",
                                           G_TYPE_INT, 96, "clock-rate", G_TYPE_INT, 90000, nullptr );
  gboolean linked = gst_element_link_filtered( rtppay, webrtcbin, rtp_caps );
  gst_caps_unref( rtp_caps );
  if ( !linked ) {
    SERVER_LOG_ERROR( "WebRTC: failed to link rtppay to webrtcbin" );
    gst_element_release_request_pad( tee_, tee_src_pad );
    gst_object_unref( tee_src_pad );
    peer.reset();
    gst_element_set_state( webrtcbin, GST_STATE_NULL );
    gst_element_set_state( rtppay, GST_STATE_NULL );
    gst_element_set_state( queue, GST_STATE_NULL );
    gst_bin_remove_many( output_bin, queue, rtppay, webrtcbin, nullptr );
    return;
  }

  // Add pad probe on rtppay src to report stats and add RTP timestamp extension
  GstPad *rtppay_src_pad = gst_element_get_static_pad( rtppay, "src" );
  gst_pad_add_probe(
      rtppay_src_pad, GstPadProbeType( GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST ),
      []( GstPad *, GstPadProbeInfo *info, gpointer user_data ) -> GstPadProbeReturn {
        auto *self = static_cast<WebrtcOutput *>( user_data );

        if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER_LIST ) {
          GstBufferList *buffer_list = GST_PAD_PROBE_INFO_BUFFER_LIST( info );
          buffer_list = gst_buffer_list_make_writable( buffer_list );
          gst_buffer_list_foreach(
              buffer_list,
              []( GstBuffer **buf, guint, gpointer user_data ) -> int {
                auto *self = static_cast<WebrtcOutput *>( user_data );
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
  gst_object_unref( rtppay_src_pad );

  // Store session
  PeerSession session;
  session.queue = queue;
  session.rtppay = rtppay;
  session.webrtcbin = webrtcbin;
  session.peer = std::move( peer );
  session.tee_src_pad = tee_src_pad;
  sessions_[conn] = std::move( session );
}

void WebrtcOutput::removePeerBranch( SoupWebsocketConnection *conn )
{
  auto it = sessions_.find( conn );
  if ( it == sessions_.end() )
    return;

  tearDownSession( it->second );
  sessions_.erase( it );
  SERVER_LOG_INFO( "WebRTC: removed peer branch from %s (%zu clients remaining)",
                   signaling_path_.c_str(), sessions_.size() );
}

void webrtc_output_set_signaling_server( PipelineOutput *output, SignalingServer *server,
                                         const std::string &path )
{
  auto *webrtc_output = dynamic_cast<WebrtcOutput *>( output );
  if ( webrtc_output ) {
    webrtc_output->setSignalingServer( server, path );
  }
}

} // namespace ros_camera_server
