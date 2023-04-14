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
  static void sendMessage( SoupWebsocketConnection *conn, const nlohmann::json &msg );

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
