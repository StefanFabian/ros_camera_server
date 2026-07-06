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

#include "signaling_server.hpp"
#include "../logging.hpp"
#include "webrtc_test_page.hpp"

#include <algorithm>

namespace ros_camera_server
{

GMainContext *SignalingServer::context_ = nullptr;

SignalingServer::SignalingServer( int port ) : port_( port ) { }

SignalingServer::~SignalingServer() { stop(); }

bool SignalingServer::start()
{
  if ( server_ )
    return true;

  server_ = soup_server_new( nullptr, nullptr );

  // The soup server attaches to the thread-default context of the calling thread.
  // Remember it so sendMessage can dispatch sends from other threads onto it.
  if ( context_ != nullptr )
    g_main_context_unref( context_ );
  context_ = g_main_context_ref_thread_default();

  // Serve test page at root
  soup_server_add_handler( server_, "/", onHttpRequest, this, nullptr );

  // Handle WebSocket upgrades for all paths
  soup_server_add_websocket_handler( server_, nullptr, nullptr, nullptr, onWebSocketOpened, this,
                                     nullptr );

  GError *error = nullptr;
  if ( !soup_server_listen_all( server_, port_, static_cast<SoupServerListenOptions>( 0 ), &error ) ) {
    SERVER_LOG_ERROR( "WebRTC signaling server failed to listen on port %d: %s", port_,
                      error->message );
    g_error_free( error );
    g_object_unref( server_ );
    server_ = nullptr;
    // Release the context ref taken above; otherwise it leaks and mainContext() keeps
    // reporting a live context for a server that never started.
    g_main_context_unref( context_ );
    context_ = nullptr;
    return false;
  }
  if ( port_ == 0 ) {
    GSList *uris = soup_server_get_uris( server_ );
    for ( GSList *l = uris; l != nullptr; l = l->next ) {
      GUri *uri = static_cast<GUri *>( l->data );
      int actual = g_uri_get_port( uri );
      if ( actual > 0 ) {
        port_ = actual;
        break;
      }
    }
    g_slist_free_full( uris, reinterpret_cast<GDestroyNotify>( g_uri_unref ) );
  }
  SERVER_LOG_INFO( "WebRTC signaling server listening on ws://0.0.0.0:%d", port_ );
  return true;
}

void SignalingServer::stop()
{
  if ( !server_ )
    return;

  for ( auto *conn : connections_ ) {
    closeIfOpen( conn, SOUP_WEBSOCKET_CLOSE_GOING_AWAY, "Server shutting down" );
    g_object_unref( conn );
  }
  connections_.clear();

  soup_server_disconnect( server_ );
  g_object_unref( server_ );
  server_ = nullptr;

  // Release the context ref taken in start() and clear it
  if ( context_ != nullptr ) {
    g_main_context_unref( context_ );
    context_ = nullptr;
  }
  SERVER_LOG_INFO( "WebRTC signaling server stopped" );
}

void SignalingServer::registerEndpoint( const std::string &path, const CameraInfo &info,
                                        ConnectHandler on_connect, MessageHandler on_message,
                                        DisconnectHandler on_disconnect )
{
  endpoints_[path] = { std::move( on_connect ), std::move( on_message ), std::move( on_disconnect ),
                       info };
  SERVER_LOG_INFO( "WebRTC signaling endpoint registered: %s", path.c_str() );
}

void SignalingServer::sendMessage( SoupWebsocketConnection *conn, const nlohmann::json &msg )
{
  if ( !conn )
    return;

  // Fast path: already on the server's context (or no context known, e.g. tests
  // driving everything from one thread).
  if ( context_ == nullptr || g_main_context_is_owner( context_ ) ) {
    if ( soup_websocket_connection_get_state( conn ) == SOUP_WEBSOCKET_STATE_OPEN ) {
      soup_websocket_connection_send_text( conn, msg.dump().c_str() );
    }
    return;
  }

  // libsoup is not thread-safe: dispatch the send to the server's context.
  // Same-priority invokes are processed in order, so SDP/ICE message order is preserved.
  dispatchToContext( conn, [text = msg.dump()]( SoupWebsocketConnection *c ) {
    if ( soup_websocket_connection_get_state( c ) == SOUP_WEBSOCKET_STATE_OPEN ) {
      soup_websocket_connection_send_text( c, text.c_str() );
    }
  } );
}

void SignalingServer::dispatchToContext( SoupWebsocketConnection *conn,
                                         std::function<void( SoupWebsocketConnection * )> action )
{
  if ( conn == nullptr || context_ == nullptr )
    return;

  struct Dispatch {
    SoupWebsocketConnection *conn;
    std::function<void( SoupWebsocketConnection * )> action;
  };
  auto *dispatch = new Dispatch{ conn, std::move( action ) };
  g_object_ref( conn );
  g_main_context_invoke_full(
      context_, G_PRIORITY_DEFAULT,
      []( gpointer data ) -> gboolean {
        auto *d = static_cast<Dispatch *>( data );
        d->action( d->conn );
        return G_SOURCE_REMOVE;
      },
      dispatch,
      []( gpointer data ) {
        auto *d = static_cast<Dispatch *>( data );
        g_object_unref( d->conn );
        delete d;
      } );
}

void SignalingServer::closeIfOpen( SoupWebsocketConnection *conn, unsigned short code,
                                   const char *reason )
{
  if ( conn == nullptr )
    return;
  if ( soup_websocket_connection_get_state( conn ) == SOUP_WEBSOCKET_STATE_OPEN )
    soup_websocket_connection_close( conn, code, reason );
}

void SignalingServer::onHttpRequest( SoupServer *, SoupServerMessage *msg, const char *path,
                                     GHashTable *, gpointer user_data )
{
  // This handler is registered at "/" and therefore also runs for WebSocket handshake
  // requests on every path. libsoup only proceeds with the handshake if the handler leaves
  // the status unset, so upgrade requests must pass through untouched.
  SoupMessageHeaders *request_headers = soup_server_message_get_request_headers( msg );
  if ( soup_message_headers_header_contains( request_headers, "Upgrade", "websocket" ) )
    return;

  const char *method = soup_server_message_get_method( msg );
  if ( g_strcmp0( method, "GET" ) != 0 ) {
    // Without an explicit status libsoup responds with 500.
    soup_server_message_set_status( msg, SOUP_STATUS_METHOD_NOT_ALLOWED, nullptr );
    return;
  }

  // Serve camera list
  if ( std::string( path ) == "/api/cameras" ) {
    auto *self = static_cast<SignalingServer *>( user_data );
    nlohmann::json cameras = nlohmann::json::array();

    for ( const auto &[endpoint_path, callbacks] : self->endpoints_ ) {
      nlohmann::json cam;
      cam["path"] = endpoint_path;
      cam["name"] = callbacks.info.name;
      cam["codec"] = callbacks.info.codec;
      cam["width"] = callbacks.info.width;
      cam["height"] = callbacks.info.height;
      cam["framerate"] = callbacks.info.framerate;
      cameras.push_back( cam );
    }

    std::string response = cameras.dump();
    SoupMessageBody *body = soup_server_message_get_response_body( msg );
    SoupMessageHeaders *headers = soup_server_message_get_response_headers( msg );
    soup_message_headers_replace( headers, "Content-Type", "application/json" );
    soup_message_body_append( body, SOUP_MEMORY_COPY, response.c_str(), response.length() );
    soup_server_message_set_status( msg, 200, nullptr );
    return;
  }

  // Only serve the test page for GET requests to "/"
  if ( std::string( path ) != "/" ) {
    soup_server_message_set_status( msg, SOUP_STATUS_NOT_FOUND, nullptr );
    return;
  }

  SoupMessageBody *body = soup_server_message_get_response_body( msg );
  SoupMessageHeaders *headers = soup_server_message_get_response_headers( msg );
  soup_message_headers_replace( headers, "Content-Type", "text/html; charset=utf-8" );
  soup_message_body_append( body, SOUP_MEMORY_STATIC, WEBRTC_TEST_PAGE, strlen( WEBRTC_TEST_PAGE ) );
  soup_server_message_set_status( msg, 200, nullptr );
}

void SignalingServer::onWebSocketOpened( SoupServer *, SoupServerMessage *, const char *path,
                                         SoupWebsocketConnection *conn, gpointer user_data )
{
  auto *self = static_cast<SignalingServer *>( user_data );
  std::string path_str = path ? path : "/";

  SERVER_LOG_INFO( "WebRTC client connected on path: %s", path_str.c_str() );

  g_object_ref( conn );
  self->connections_.push_back( conn );

  // Store connection info for callbacks
  auto *info = new ConnectionInfo{ path_str, self };
  g_object_set_data_full( G_OBJECT( conn ), "connection-info", info,
                          []( gpointer data ) { delete static_cast<ConnectionInfo *>( data ); } );

  g_signal_connect( conn, "message", G_CALLBACK( onMessage ), user_data );
  g_signal_connect( conn, "closed", G_CALLBACK( onClosed ), user_data );

  // Notify endpoint handler
  auto it = self->endpoints_.find( path_str );
  if ( it != self->endpoints_.end() && it->second.on_connect ) {
    it->second.on_connect( conn );
  } else {
    SERVER_LOG_WARN( "WebRTC client connected to unregistered path: %s", path_str.c_str() );
    // No endpoint will ever service this connection; close it instead of keeping the socket open forever.
    closeIfOpen( conn, SOUP_WEBSOCKET_CLOSE_NORMAL, "Unknown signaling path" );
  }
}

void SignalingServer::onMessage( SoupWebsocketConnection *conn, gint type, GBytes *message,
                                 gpointer user_data )
{
  if ( type != SOUP_WEBSOCKET_DATA_TEXT )
    return;

  auto *self = static_cast<SignalingServer *>( user_data );
  auto *info =
      static_cast<ConnectionInfo *>( g_object_get_data( G_OBJECT( conn ), "connection-info" ) );
  if ( !info )
    return;

  gsize len;
  const char *data = static_cast<const char *>( g_bytes_get_data( message, &len ) );
  std::string text( data, len );

  try {
    nlohmann::json j = nlohmann::json::parse( text );
    auto it = self->endpoints_.find( info->path );
    if ( it != self->endpoints_.end() && it->second.on_message ) {
      it->second.on_message( conn, j );
    }
  } catch ( const std::exception &e ) {
    SERVER_LOG_WARN( "WebRTC signaling: failed to parse JSON message: %s", e.what() );
  }
}

void SignalingServer::onClosed( SoupWebsocketConnection *conn, gpointer user_data )
{
  auto *self = static_cast<SignalingServer *>( user_data );
  auto *info =
      static_cast<ConnectionInfo *>( g_object_get_data( G_OBJECT( conn ), "connection-info" ) );

  if ( info ) {
    SERVER_LOG_INFO( "WebRTC client disconnected from path: %s", info->path.c_str() );
    auto it = self->endpoints_.find( info->path );
    if ( it != self->endpoints_.end() && it->second.on_disconnect ) {
      it->second.on_disconnect( conn );
    }
  }

  auto conn_it = std::find( self->connections_.begin(), self->connections_.end(), conn );
  if ( conn_it != self->connections_.end() ) {
    g_object_unref( *conn_it );
    self->connections_.erase( conn_it );
  }
}

} // namespace ros_camera_server
