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

#ifndef ROS_CAMERA_SERVER_SIGNALING_SERVER_HPP
#define ROS_CAMERA_SERVER_SIGNALING_SERVER_HPP

#include "ros_camera_server/helpers/json.hpp"

#include <functional>
#include <libsoup/soup.h>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ros_camera_server
{

class SignalingServer
{
public:
  using MessageHandler = std::function<void( SoupWebsocketConnection *, const nlohmann::json & )>;
  using ConnectHandler = std::function<void( SoupWebsocketConnection * )>;
  using DisconnectHandler = std::function<void( SoupWebsocketConnection * )>;

  explicit SignalingServer( int port );
  ~SignalingServer();

  SignalingServer( const SignalingServer & ) = delete;
  SignalingServer &operator=( const SignalingServer & ) = delete;

  bool start();
  void stop();

  /// Register an endpoint at a given path. When a WebSocket client connects to this path,
  /// the corresponding handlers are called.
  struct CameraInfo {
    std::string name;
    std::string codec;
    int width;
    int height;
    float framerate; // fps
  };

  bool hasEndpoint( const std::string &path ) const { return endpoints_.count( path ) > 0; }

  /// Register an endpoint at a given path using a CameraInfo to populate "/api/cameras".
  void registerEndpoint( const std::string &path, const CameraInfo &info, ConnectHandler on_connect,
                         MessageHandler on_message, DisconnectHandler on_disconnect );

  /// Send a JSON message to a specific WebSocket connection.
  /// Thread-safe: libsoup is not thread-safe, so when called from a thread that does not
  /// own the server's GMainContext (e.g. GStreamer/webrtcbin threads), the send is
  /// dispatched to that context instead of writing to the connection directly.
  static void sendMessage( SoupWebsocketConnection *conn, const nlohmann::json &msg );

  /// Run `action` on the signaling server's GMainContext with `conn` kept alive for the
  /// duration of the call. libsoup is not thread-safe, so any connection operation issued
  /// from a thread that does not own the context (GStreamer/webrtcbin/ICE threads) must be
  /// funneled through here. No-op if `conn` is null or no context exists yet.
  static void dispatchToContext( SoupWebsocketConnection *conn,
                                 std::function<void( SoupWebsocketConnection * )> action );

  /// Close `conn` with the given close code and reason if it is currently open. Centralizes the
  /// open-check + close so every teardown path closes consistently. libsoup is not thread-safe,
  /// so this must run on the signaling GMainContext; callers on other threads route through
  /// dispatchToContext. No-op if `conn` is null or not open.
  static void closeIfOpen( SoupWebsocketConnection *conn, unsigned short code, const char *reason );

  /// The GMainContext the signaling server (and all its connections) runs on.
  /// Null until start() has been called.
  static GMainContext *mainContext() { return context_; }

  int port() const { return port_; }

private:
  struct EndpointCallbacks {
    ConnectHandler on_connect;
    MessageHandler on_message;
    DisconnectHandler on_disconnect;
    CameraInfo info;
  };

  struct ConnectionInfo {
    std::string path;
    SignalingServer *server;
  };

  int port_;
  SoupServer *server_ = nullptr;
  std::map<std::string, EndpointCallbacks> endpoints_;
  std::vector<SoupWebsocketConnection *> connections_;
  /// Context the soup server is attached to (the thread-default context at start()).
  /// Static so the static sendMessage can dispatch to it; there is only one signaling
  /// server per process.
  static GMainContext *context_;

  static void onHttpRequest( SoupServer *server, SoupServerMessage *msg, const char *path,
                             GHashTable *query, gpointer user_data );
  static void onWebSocketOpened( SoupServer *server, SoupServerMessage *msg, const char *path,
                                 SoupWebsocketConnection *conn, gpointer user_data );
  static void onMessage( SoupWebsocketConnection *conn, gint type, GBytes *message,
                         gpointer user_data );
  static void onClosed( SoupWebsocketConnection *conn, gpointer user_data );
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_SIGNALING_SERVER_HPP
