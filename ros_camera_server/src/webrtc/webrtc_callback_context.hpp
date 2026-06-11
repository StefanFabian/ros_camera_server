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

#ifndef ROS_CAMERA_SERVER_WEBRTC_CALLBACK_CONTEXT_HPP
#define ROS_CAMERA_SERVER_WEBRTC_CALLBACK_CONTEXT_HPP

#include <gst/gst.h>
#include <libsoup/soup.h>

namespace ros_camera_server
{

/// Ref-holding user data for asynchronous webrtcbin callbacks: signal handlers, GStreamer
/// promises and timeouts. These run on GStreamer/ICE threads and can fire while the owning
/// peer/session is being torn down on the signaling thread, so they must not reference the
/// owner. Holding its own refs on the webrtcbin and signaling connection keeps both alive for
/// the duration of the callback. destroy() matches the GDestroyNotify signature expected by
/// gst_promise_* and g_source_set_callback; destroyClosure() adapts it to the GClosureNotify
/// signature g_signal_connect_data expects.
struct WebrtcCallbackContext {
  GstElement *webrtcbin = nullptr;
  SoupWebsocketConnection *conn = nullptr;

  static WebrtcCallbackContext *create( GstElement *webrtcbin, SoupWebsocketConnection *conn )
  {
    auto *ctx = new WebrtcCallbackContext;
    ctx->webrtcbin = GST_ELEMENT( gst_object_ref( webrtcbin ) );
    ctx->conn = SOUP_WEBSOCKET_CONNECTION( g_object_ref( conn ) );
    return ctx;
  }

  static void destroy( gpointer data )
  {
    auto *ctx = static_cast<WebrtcCallbackContext *>( data );
    gst_object_unref( ctx->webrtcbin );
    g_object_unref( ctx->conn );
    delete ctx;
  }

  /// GClosureNotify adapter for g_signal_connect_data, which passes the closure as a second
  /// argument that destroy() (a GDestroyNotify) cannot accept.
  static void destroyClosure( gpointer data, GClosure * ) { destroy( data ); }
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_WEBRTC_CALLBACK_CONTEXT_HPP
