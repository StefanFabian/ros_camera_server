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
#include "../webrtc/webrtc_callback_context.hpp"
#include "../webrtc/webrtc_peer.hpp"

#include <atomic>
#include <gst/rtp/gstrtpbuffer.h>
#include <gst/video/video.h>
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

namespace
{

/// If a peer has not established its transport within this time, its signaling
/// connection is closed so the session is torn down and the client can retry.
constexpr guint PEER_ESTABLISH_TIMEOUT_SECONDS = 10;

/// Remove the per-peer elements from the output bin, then set them to NULL.
/// Order matters: removing before NULL prevents a concurrent pipeline state cascade
/// (e.g. a restart on another thread) from bumping an already-NULL child back to READY
/// before the final unref, which triggers "disposed in READY" criticals. Refs are held
/// across the removal so the elements survive until we are done with them.
void removePeerElements( GstBin *output_bin, GstElement *queue, GstElement *rtppay,
                         GstElement *webrtcbin )
{
  gst_object_ref( queue );
  gst_object_ref( rtppay );
  gst_object_ref( webrtcbin );
  gst_bin_remove_many( output_bin, queue, rtppay, webrtcbin, nullptr );
  gst_element_set_state( webrtcbin, GST_STATE_NULL );
  gst_element_set_state( rtppay, GST_STATE_NULL );
  gst_element_set_state( queue, GST_STATE_NULL );
  gst_object_unref( webrtcbin );
  gst_object_unref( rtppay );
  gst_object_unref( queue );
}

/// Log a webrtcbin transport-state transition and, if it reached its failed value, close
/// the signaling connection so the session is torn down. Shared by the connection-state and
/// ice-connection-state watchers so their failure handling cannot drift apart.
void closeSignalingOnFailure( GstElement *webrtcbin, SoupWebsocketConnection *conn,
                              const char *property, GType enum_type, gint failed_value,
                              const char *what )
{
  gint state = 0;
  g_object_get( webrtcbin, property, &state, nullptr );
  gchar *state_name = g_enum_to_string( enum_type, state );
  SERVER_LOG_DEBUG( "WebRTC: %s %s changed to %s", GST_ELEMENT_NAME( webrtcbin ), property,
                    state_name );
  g_free( state_name );
  if ( state == failed_value ) {
    SERVER_LOG_WARN( "WebRTC: %s %s failed, closing signaling connection",
                     GST_ELEMENT_NAME( webrtcbin ), what );
    SignalingServer::dispatchToContext( conn, []( SoupWebsocketConnection *c ) {
      SignalingServer::closeIfOpen( c, SOUP_WEBSOCKET_CLOSE_NORMAL, "WebRTC transport failed" );
    } );
  }
}

void onConnectionStateChanged( GstElement *webrtcbin, GParamSpec *, gpointer user_data )
{
  auto *watch = static_cast<WebrtcCallbackContext *>( user_data );
  closeSignalingOnFailure( webrtcbin, watch->conn, "connection-state",
                           GST_TYPE_WEBRTC_PEER_CONNECTION_STATE,
                           GST_WEBRTC_PEER_CONNECTION_STATE_FAILED, "connection" );
}

void onIceConnectionStateChanged( GstElement *webrtcbin, GParamSpec *, gpointer user_data )
{
  auto *watch = static_cast<WebrtcCallbackContext *>( user_data );
  closeSignalingOnFailure( webrtcbin, watch->conn, "ice-connection-state",
                           GST_TYPE_WEBRTC_ICE_CONNECTION_STATE,
                           GST_WEBRTC_ICE_CONNECTION_STATE_FAILED, "ICE" );
}

/// Runs on the signaling context. Closes the connection if the peer transport has
/// not established within PEER_ESTABLISH_TIMEOUT_SECONDS. This catches sessions
/// where negotiation silently stalled (e.g. lost signaling message): without it
/// the session lives forever with its send path blocked, reporting a connected
/// client but transmitting nothing.
gboolean onEstablishTimeout( gpointer user_data )
{
  auto *watch = static_cast<WebrtcCallbackContext *>( user_data );
  // g_object_get writes a gint for enum properties; reading into the enum-typed variables
  // directly would corrupt the stack under -fshort-enums. Read as gint, then narrow.
  gint ice_state_raw = 0;
  gint conn_state_raw = 0;
  g_object_get( watch->webrtcbin, "ice-connection-state", &ice_state_raw, "connection-state",
                &conn_state_raw, nullptr );
  auto ice_state = static_cast<GstWebRTCICEConnectionState>( ice_state_raw );
  auto conn_state = static_cast<GstWebRTCPeerConnectionState>( conn_state_raw );
  bool established = conn_state == GST_WEBRTC_PEER_CONNECTION_STATE_CONNECTED ||
                     ice_state == GST_WEBRTC_ICE_CONNECTION_STATE_CONNECTED ||
                     ice_state == GST_WEBRTC_ICE_CONNECTION_STATE_COMPLETED;
  if ( !established ) {
    gchar *ice_name = g_enum_to_string( GST_TYPE_WEBRTC_ICE_CONNECTION_STATE, ice_state );
    gchar *conn_name = g_enum_to_string( GST_TYPE_WEBRTC_PEER_CONNECTION_STATE, conn_state );
    SERVER_LOG_WARN( "WebRTC: %s did not establish within %u s (ice: %s, connection: %s), closing "
                     "signaling connection so the client can retry",
                     GST_ELEMENT_NAME( watch->webrtcbin ), PEER_ESTABLISH_TIMEOUT_SECONDS, ice_name,
                     conn_name );
    g_free( ice_name );
    g_free( conn_name );
    SignalingServer::closeIfOpen( watch->conn, SOUP_WEBSOCKET_CLOSE_NORMAL,
                                  "WebRTC transport not established" );
  }
  return G_SOURCE_REMOVE;
}

} // namespace

class WebrtcOutput : public PipelineOutput
{
public:
  using PipelineOutput::PipelineOutput;
  ~WebrtcOutput() override;

  bool build( int index, const WebrtcOutputConfiguration &config );

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const override;

  // Read from the ROS thread (statistics()) while sessions_ is mutated on the signaling
  // thread, so the count is mirrored into an atomic instead of reading sessions_.size()
  // (a concurrent std::map read/write would be a data race).
  int getClientCount() const override { return client_count_.load( std::memory_order_relaxed ); }

  void onPipelineStopping() override;

  /// Called by CameraServer after pipelines are built to wire up signaling.
  void setSignalingServer( SignalingServer *server, const std::string &path );

private:
  struct PeerSession {
    GstElement *queue = nullptr;
    GstElement *rtppay = nullptr;
    GstElement *webrtcbin = nullptr;
    std::unique_ptr<WebRTCPeer> peer;
    GstPad *tee_src_pad = nullptr;
    gulong connection_state_handler = 0;
    gulong ice_state_handler = 0;
    GSource *establish_timeout = nullptr;
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
  /// Mirrors sessions_.size() for lock-free reads from the ROS thread; only written on the
  /// signaling thread right after sessions_ is mutated.
  std::atomic<int> client_count_{ 0 };

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

  if ( session.establish_timeout != nullptr ) {
    g_source_destroy( session.establish_timeout );
    g_source_unref( session.establish_timeout );
    session.establish_timeout = nullptr;
  }
  if ( session.webrtcbin != nullptr ) {
    if ( session.connection_state_handler != 0 )
      g_signal_handler_disconnect( session.webrtcbin, session.connection_state_handler );
    if ( session.ice_state_handler != 0 )
      g_signal_handler_disconnect( session.webrtcbin, session.ice_state_handler );
  }
  session.connection_state_handler = 0;
  session.ice_state_handler = 0;

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

  if ( session.queue && session.rtppay && session.webrtcbin ) {
    removePeerElements( output_bin, session.queue, session.rtppay, session.webrtcbin );
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

void WebrtcOutput::onPipelineStopping()
{
  // WebRTC transports (DTLS/ICE) do not survive the pipeline's NULL state change.
  // Close the signaling connections so clients reconnect and renegotiate instead of
  // watching a frozen stream. Dispatched to the signaling context: libsoup is not
  // thread-safe and sessions_ must only be touched from there.
  GMainContext *context = SignalingServer::mainContext();
  if ( context == nullptr )
    return;
  g_main_context_invoke_full(
      context, G_PRIORITY_DEFAULT,
      []( gpointer data ) -> gboolean {
        auto *self = static_cast<WebrtcOutput *>( data );
        for ( const auto &[conn, session] : self->sessions_ ) {
          SignalingServer::closeIfOpen( conn, SOUP_WEBSOCKET_CLOSE_GOING_AWAY, "Pipeline restarting" );
        }
        return G_SOURCE_REMOVE;
      },
      this, nullptr );
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
  g_object_set( G_OBJECT( queue ), "max-size-time", (guint64)40'000'000 /* 40ms */,
                "max-size-buffers", 5, "max-size-bytes", 0, "leaky", 2 /* downstream */, nullptr );

  const char *pay_factory = rtpPayloaderForCodec( codec_ );
  GstElement *rtppay = gst_element_factory_make( pay_factory, ( peer_name + "_rtppay" ).c_str() );
  // rtph264pay and rtph265pay share these properties; guard by codec so a future
  // payloader (e.g. vp8) does not get H.26x-specific properties set on it.
  if ( codec_ == "h264" || codec_ == "h265" ) {
    g_object_set( G_OBJECT( rtppay ), "timestamp-offset", 0, "mtu", 1300, "aggregate-mode",
                  0 /* zero-latency */, "config-interval", -1, nullptr );
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
    // No session is stored, so nothing else will ever close this connection: close it now so
    // the client stops waiting for an offer and reconnects instead of lingering forever.
    SignalingServer::closeIfOpen( conn, SOUP_WEBSOCKET_CLOSE_NORMAL, "WebRTC setup failed" );
    return;
  }

  // libnice's ICE agent runs UPnP-IGD discovery by default, sending SSDP M-SEARCH
  // packets to 239.255.255.250 to find a router for automatic port mapping. We rely
  // solely on STUN/signaling for connectivity, not router port mapping, so disable UPnP.
  {
    GObject *ice = nullptr;
    g_object_get( webrtcbin, "ice-agent", &ice, nullptr );
    if ( ice != nullptr ) {
      GObject *nice_agent = nullptr;
      g_object_get( ice, "agent", &nice_agent, nullptr );
      if ( nice_agent != nullptr ) {
        // The property only exists when libnice was built with GUPnP
        if ( g_object_class_find_property( G_OBJECT_GET_CLASS( nice_agent ), "upnp" ) != nullptr ) {
          g_object_set( nice_agent, "upnp", FALSE, nullptr );
        }
        g_object_unref( nice_agent );
      }
      g_object_unref( ice );
    }
  }

  // Create the WebRTC peer BEFORE linking to webrtcbin, so the on-negotiation-needed
  // signal handler is connected before the link triggers it.
  auto peer = std::make_unique<WebRTCPeer>( WebRTCPeer::Role::OFFERER, conn, webrtcbin );

  // Add elements to the output bin (same bin as the tee so pad linking works)
  GstBin *output_bin = GST_BIN( bin );
  gst_bin_add_many( output_bin, queue, rtppay, webrtcbin, nullptr );
  gst_element_link( queue, rtppay );

  // Tears the half-built peer branch back down. Used by every failure path below so a future
  // per-peer element is unwound in one place, not several. The tee pad is passed in because
  // the tee is only linked at the very end; earlier failure paths have no pad to release.
  auto cleanup_partial_branch = [&]( GstPad *tee_src_pad ) {
    if ( tee_src_pad != nullptr ) {
      gst_element_release_request_pad( tee_, tee_src_pad );
      gst_object_unref( tee_src_pad );
    }
    peer.reset();
    removePeerElements( output_bin, queue, rtppay, webrtcbin );
    // No session is stored on these paths, so close the connection here; otherwise the client
    // never gets an offer and the socket stays open forever with no one left to close it.
    SignalingServer::closeIfOpen( conn, SOUP_WEBSOCKET_CLOSE_NORMAL, "WebRTC setup failed" );
  };

  // Sync webrtcbin BEFORE linking rtppay to it: it defers on-negotiation-needed while
  // still in NULL, so it must have left NULL for the link to trigger it.
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
    cleanup_partial_branch( nullptr );
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

  // Activate the remaining branch elements, then link the tee LAST, when the whole branch
  // is linked and active. Linking the tee earlier opens a window where it pushes a buffer
  // into a still-inactive pad; the resulting GST_FLOW_FLUSHING is fatal for the tee
  // (allow-not-linked only forgives NOT_LINKED), latches into every queue upstream and
  // silently pauses the feeding streaming thread - the entire output freezes at 0 fps
  // without any bus error, for every current and future session.
  gst_element_sync_state_with_parent( rtppay );
  gst_element_sync_state_with_parent( queue );

  GstPad *tee_src_pad = gst_element_request_pad_simple( tee_, "src_%u" );
  GstPad *queue_sink_pad = gst_element_get_static_pad( queue, "sink" );
  GstPadLinkReturn link_ret = gst_pad_link( tee_src_pad, queue_sink_pad );
  gst_object_unref( queue_sink_pad );
  if ( link_ret != GST_PAD_LINK_OK ) {
    SERVER_LOG_ERROR( "WebRTC: failed to link tee to queue (error: %d)", link_ret );
    cleanup_partial_branch( tee_src_pad );
    return;
  }

  // Request an immediate keyframe so the new client can start decoding right away
  // instead of waiting for the next periodic IDR. Sources that cannot comply
  // (e.g. cameras delivering pre-encoded H.264) ignore the event.
  gst_pad_send_event( tee_src_pad,
                      gst_video_event_new_upstream_force_key_unit( GST_CLOCK_TIME_NONE, TRUE, 0 ) );

  // Store session
  PeerSession session;
  session.queue = queue;
  session.rtppay = rtppay;
  session.webrtcbin = webrtcbin;
  session.peer = std::move( peer );
  session.tee_src_pad = tee_src_pad;

  // Monitor transport state: log transitions, close the signaling connection on
  // failure or when the transport never establishes, so the session is torn down
  // instead of staying blocked forever while reporting a connected client.
  session.connection_state_handler = g_signal_connect_data(
      webrtcbin, "notify::connection-state", G_CALLBACK( onConnectionStateChanged ),
      WebrtcCallbackContext::create( webrtcbin, conn ), WebrtcCallbackContext::destroyClosure,
      static_cast<GConnectFlags>( 0 ) );
  session.ice_state_handler = g_signal_connect_data(
      webrtcbin, "notify::ice-connection-state", G_CALLBACK( onIceConnectionStateChanged ),
      WebrtcCallbackContext::create( webrtcbin, conn ), WebrtcCallbackContext::destroyClosure,
      static_cast<GConnectFlags>( 0 ) );

  if ( SignalingServer::mainContext() != nullptr ) {
    GSource *timeout = g_timeout_source_new_seconds( PEER_ESTABLISH_TIMEOUT_SECONDS );
    g_source_set_callback( timeout, onEstablishTimeout,
                           WebrtcCallbackContext::create( webrtcbin, conn ),
                           WebrtcCallbackContext::destroy );
    g_source_attach( timeout, SignalingServer::mainContext() );
    session.establish_timeout = timeout;
  }

  sessions_[conn] = std::move( session );
  client_count_.store( static_cast<int>( sessions_.size() ), std::memory_order_relaxed );
}

void WebrtcOutput::removePeerBranch( SoupWebsocketConnection *conn )
{
  auto it = sessions_.find( conn );
  if ( it == sessions_.end() )
    return;

  tearDownSession( it->second );
  sessions_.erase( it );
  client_count_.store( static_cast<int>( sessions_.size() ), std::memory_order_relaxed );
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
