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

#ifndef ROS_CAMERA_SERVER_TEST_WEBRTC_TEST_CLIENT_HPP
#define ROS_CAMERA_SERVER_TEST_WEBRTC_TEST_CLIENT_HPP

#include "ros_camera_server/helpers/json.hpp"

#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>

#include <gio/gio.h>
#include <libsoup/soup.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace ros_camera_server_test
{

/// A self-contained WebRTC test client that connects to the camera server's signaling
/// websocket, answers the server's offer and receives the RTP stream.
///
/// Each instance runs its own GMainContext on a dedicated thread; all libsoup operations
/// are funneled onto that thread (libsoup is not thread-safe). The client performs its own
/// signaling instead of reusing the server's classes so client traffic never touches the
/// server's GMainContext.
///
/// Failure-injection support:
///  - Mode::MUTE never answers the offer (negotiation stalls; the server's establish
///    timeout must eventually close the connection).
///  - Mode::CLOSE_AFTER_OFFER closes cleanly right after the offer arrives
///    (disconnect mid-negotiation).
///  - disconnectAbrupt() kills the TCP stream without a websocket close frame
///    (client process death / network loss).
class WebRtcTestClient
{
public:
  enum class Mode { FULL, MUTE, CLOSE_AFTER_OFFER };

  WebRtcTestClient( int port, const std::string &path, Mode mode = Mode::FULL )
      : mode_( mode ), uri_( "ws://127.0.0.1:" + std::to_string( port ) + path )
  {
    if ( mode_ != Mode::MUTE ) {
      static std::atomic<int> counter{ 0 };
      std::string name = "wrtc_test_client_" + std::to_string( counter.fetch_add( 1 ) );
      std::string launch = "webrtcbin name=" + name +
                           " bundle-policy=max-bundle ! rtph264depay ! fakesink name=" + name +
                           "_sink signal-handoffs=true sync=false";
      pipeline_ = gst_parse_launch( launch.c_str(), nullptr );
      webrtc_ = gst_bin_get_by_name( GST_BIN( pipeline_ ), name.c_str() );
      GstElement *sink = gst_bin_get_by_name( GST_BIN( pipeline_ ), ( name + "_sink" ).c_str() );
      g_signal_connect( sink, "handoff", G_CALLBACK( onHandoff ), this );
      gst_object_unref( sink );
      g_signal_connect( webrtc_, "on-ice-candidate", G_CALLBACK( onIceCandidate ), this );
      disableUpnp();
      gst_element_set_state( pipeline_, GST_STATE_PLAYING );
    }

    context_ = g_main_context_new();
    loop_ = g_main_loop_new( context_, FALSE );
    thread_ = std::thread( [this]() {
      g_main_context_push_thread_default( context_ );
      connectWebsocket();
      g_main_loop_run( loop_ );
      // Drain pending sources (e.g. queued sends) before dropping the context.
      while ( g_main_context_pending( context_ ) ) g_main_context_iteration( context_, FALSE );
      if ( conn_ != nullptr ) {
        g_object_unref( conn_ );
        conn_ = nullptr;
      }
      if ( session_ != nullptr ) {
        g_object_unref( session_ );
        session_ = nullptr;
      }
      g_main_context_pop_thread_default( context_ );
    } );
  }

  ~WebRtcTestClient()
  {
    // Stop the media pipeline first so webrtcbin threads stop producing ICE callbacks.
    if ( pipeline_ != nullptr ) {
      gst_element_set_state( pipeline_, GST_STATE_NULL );
    }
    invoke( [this]() {
      if ( conn_ != nullptr &&
           soup_websocket_connection_get_state( conn_ ) == SOUP_WEBSOCKET_STATE_OPEN ) {
        soup_websocket_connection_close( conn_, SOUP_WEBSOCKET_CLOSE_NORMAL, "test done" );
      }
      g_main_loop_quit( loop_ );
    } );
    thread_.join();
    g_main_loop_unref( loop_ );
    g_main_context_unref( context_ );
    if ( webrtc_ != nullptr )
      gst_object_unref( webrtc_ );
    if ( pipeline_ != nullptr )
      gst_object_unref( pipeline_ );
  }

  WebRtcTestClient( const WebRtcTestClient & ) = delete;
  WebRtcTestClient &operator=( const WebRtcTestClient & ) = delete;

  bool waitConnected( std::chrono::milliseconds timeout )
  {
    return waitFor( timeout, [this]() { return connected_ || connect_failed_; } ) && connected_;
  }

  bool connectFailed() const { return connect_failed_; }

  /// Wait until the client received at least n RTP buffers.
  bool waitBuffers( int n, std::chrono::milliseconds timeout )
  {
    return waitFor( timeout, [this, n]() { return buffer_count_.load() >= n; } );
  }

  /// Wait until the connection is closed (by either side).
  bool waitClosed( std::chrono::milliseconds timeout )
  {
    return waitFor( timeout, [this]() { return closed_; } );
  }

  int bufferCount() const { return buffer_count_.load(); }
  bool offerReceived() const { return offer_received_; }

  /// Close the websocket with a proper close handshake (clean disconnect).
  void closeClean()
  {
    invoke( [this]() {
      if ( conn_ != nullptr &&
           soup_websocket_connection_get_state( conn_ ) == SOUP_WEBSOCKET_STATE_OPEN ) {
        soup_websocket_connection_close( conn_, SOUP_WEBSOCKET_CLOSE_NORMAL, "bye" );
      }
    } );
  }

  /// Send an arbitrary raw text frame. Used to inject malformed / out-of-protocol
  /// signaling (non-JSON, wrong message types, garbage SDP/ICE) to fuzz the server.
  /// No-op if the connection is not currently open.
  void sendText( const std::string &text )
  {
    invoke( [this, text]() {
      if ( conn_ != nullptr &&
           soup_websocket_connection_get_state( conn_ ) == SOUP_WEBSOCKET_STATE_OPEN ) {
        soup_websocket_connection_send_text( conn_, text.c_str() );
      }
    } );
  }

  /// Kill the underlying TCP stream without sending a websocket close frame.
  /// The server sees the connection drop like a crashed client process.
  void disconnectAbrupt()
  {
    invoke( [this]() {
      if ( conn_ != nullptr ) {
        GIOStream *stream = soup_websocket_connection_get_io_stream( conn_ );
        if ( stream != nullptr )
          g_io_stream_close( stream, nullptr, nullptr );
      }
    } );
  }

private:
  /// Disable UPnP-IGD on the webrtcbin's nice agent, mirroring the server's peer setup.
  /// Besides the unwanted SSDP traffic, a discovered IGD deadlocks agent teardown: the agent's
  /// dispose waits for the GUPnP-IGD worker thread while that worker is blocked inside a
  /// port-mapping signal callback into the agent being disposed.
  void disableUpnp()
  {
    GObject *ice = nullptr;
    g_object_get( webrtc_, "ice-agent", &ice, nullptr );
    if ( ice == nullptr )
      return;
    GObject *nice_agent = nullptr;
    g_object_get( ice, "agent", &nice_agent, nullptr );
    if ( nice_agent != nullptr ) {
      if ( g_object_class_find_property( G_OBJECT_GET_CLASS( nice_agent ), "upnp" ) != nullptr ) {
        g_object_set( nice_agent, "upnp", FALSE, nullptr );
      }
      g_object_unref( nice_agent );
    }
    g_object_unref( ice );
  }

  template<typename Predicate>
  bool waitFor( std::chrono::milliseconds timeout, Predicate &&predicate )
  {
    std::unique_lock<std::mutex> lock( mutex_ );
    return cv_.wait_for( lock, timeout, std::forward<Predicate>( predicate ) );
  }

  /// Run fn on the client's GMainContext thread (required for all libsoup operations).
  void invoke( std::function<void()> fn )
  {
    auto *boxed = new std::function<void()>( std::move( fn ) );
    g_main_context_invoke_full(
        context_, G_PRIORITY_DEFAULT,
        []( gpointer data ) -> gboolean {
          ( *static_cast<std::function<void()> *>( data ) )();
          return G_SOURCE_REMOVE;
        },
        boxed, []( gpointer data ) { delete static_cast<std::function<void()> *>( data ); } );
  }

  void notify( std::function<void()> update )
  {
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      update();
    }
    cv_.notify_all();
  }

  void connectWebsocket()
  {
    session_ = soup_session_new();
    SoupMessage *msg = soup_message_new( "GET", uri_.c_str() );
    soup_session_websocket_connect_async(
        session_, msg, nullptr, nullptr, G_PRIORITY_DEFAULT, nullptr,
        []( GObject *session, GAsyncResult *res, gpointer user_data ) {
          auto *self = static_cast<WebRtcTestClient *>( user_data );
          GError *error = nullptr;
          SoupWebsocketConnection *conn =
              soup_session_websocket_connect_finish( SOUP_SESSION( session ), res, &error );
          if ( error != nullptr ) {
            g_error_free( error );
            self->notify( [self]() { self->connect_failed_ = true; } );
            return;
          }
          self->conn_ = conn;
          g_signal_connect( conn, "message", G_CALLBACK( onMessage ), self );
          g_signal_connect( conn, "closed", G_CALLBACK( onClosed ), self );
          self->notify( [self]() { self->connected_ = true; } );
        },
        this );
    g_object_unref( msg );
  }

  void sendJson( const nlohmann::json &msg ) { sendText( msg.dump() ); }

  static void onHandoff( GstElement *, GstBuffer *, GstPad *, gpointer user_data )
  {
    auto *self = static_cast<WebRtcTestClient *>( user_data );
    self->buffer_count_.fetch_add( 1 );
    // waitBuffers polls the atomic through the cv; wake it up.
    self->cv_.notify_all();
  }

  static void onClosed( SoupWebsocketConnection *, gpointer user_data )
  {
    auto *self = static_cast<WebRtcTestClient *>( user_data );
    self->notify( [self]() { self->closed_ = true; } );
  }

  static void onMessage( SoupWebsocketConnection *, gint type, GBytes *message, gpointer user_data )
  {
    auto *self = static_cast<WebRtcTestClient *>( user_data );
    if ( type != SOUP_WEBSOCKET_DATA_TEXT )
      return;
    gsize len = 0;
    const char *data = static_cast<const char *>( g_bytes_get_data( message, &len ) );
    nlohmann::json msg;
    try {
      msg = nlohmann::json::parse( std::string( data, len ) );
    } catch ( const std::exception & ) {
      return;
    }
    if ( self->mode_ == Mode::MUTE )
      return;
    if ( msg.contains( "type" ) && msg["type"] == "offer" ) {
      self->notify( [self]() { self->offer_received_ = true; } );
      if ( self->mode_ == Mode::CLOSE_AFTER_OFFER ) {
        self->closeClean();
        return;
      }
      self->handleOffer( msg["sdp"].get<std::string>() );
    } else if ( msg.contains( "ice" ) ) {
      const auto &ice = msg["ice"];
      if ( ice.contains( "candidate" ) && ice.contains( "sdpMLineIndex" ) ) {
        g_signal_emit_by_name( self->webrtc_, "add-ice-candidate", ice["sdpMLineIndex"].get<guint>(),
                               ice["candidate"].get<std::string>().c_str() );
      }
    }
  }

  void handleOffer( const std::string &sdp_text )
  {
    GstSDPMessage *sdp = nullptr;
    gst_sdp_message_new( &sdp );
    if ( gst_sdp_message_parse_buffer( reinterpret_cast<const guint8 *>( sdp_text.c_str() ),
                                       sdp_text.size(), sdp ) != GST_SDP_OK ) {
      gst_sdp_message_free( sdp );
      return;
    }
    GstWebRTCSessionDescription *offer =
        gst_webrtc_session_description_new( GST_WEBRTC_SDP_TYPE_OFFER, sdp );
    GstPromise *promise = gst_promise_new_with_change_func(
        []( GstPromise *promise, gpointer user_data ) {
          auto *self = static_cast<WebRtcTestClient *>( user_data );
          gst_promise_unref( promise );
          self->createAnswer();
        },
        this, nullptr );
    g_signal_emit_by_name( webrtc_, "set-remote-description", offer, promise );
    gst_webrtc_session_description_free( offer );
  }

  void createAnswer()
  {
    GstPromise *promise = gst_promise_new_with_change_func(
        []( GstPromise *promise, gpointer user_data ) {
          auto *self = static_cast<WebRtcTestClient *>( user_data );
          GstWebRTCSessionDescription *answer = nullptr;
          if ( gst_promise_wait( promise ) == GST_PROMISE_RESULT_REPLIED ) {
            const GstStructure *reply = gst_promise_get_reply( promise );
            if ( reply != nullptr )
              gst_structure_get( reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer,
                                 nullptr );
          }
          gst_promise_unref( promise );
          if ( answer == nullptr )
            return;
          g_signal_emit_by_name( self->webrtc_, "set-local-description", answer, nullptr );
          gchar *text = gst_sdp_message_as_text( answer->sdp );
          self->sendJson( { { "type", "answer" }, { "sdp", text } } );
          g_free( text );
          gst_webrtc_session_description_free( answer );
        },
        this, nullptr );
    g_signal_emit_by_name( webrtc_, "create-answer", nullptr, promise );
  }

  static void onIceCandidate( GstElement *, guint mline_index, gchar *candidate, gpointer user_data )
  {
    auto *self = static_cast<WebRtcTestClient *>( user_data );
    self->sendJson( { { "ice", { { "candidate", candidate }, { "sdpMLineIndex", mline_index } } } } );
  }

  Mode mode_;
  std::string uri_;
  GstElement *pipeline_ = nullptr;
  GstElement *webrtc_ = nullptr;
  GMainContext *context_ = nullptr;
  GMainLoop *loop_ = nullptr;
  std::thread thread_;
  SoupSession *session_ = nullptr;
  SoupWebsocketConnection *conn_ = nullptr;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<int> buffer_count_{ 0 };
  bool connected_ = false;
  bool connect_failed_ = false;
  bool closed_ = false;
  bool offer_received_ = false;
};

} // namespace ros_camera_server_test

#endif // ROS_CAMERA_SERVER_TEST_WEBRTC_TEST_CLIENT_HPP
