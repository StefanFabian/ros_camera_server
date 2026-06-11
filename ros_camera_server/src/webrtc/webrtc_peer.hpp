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

#ifndef ROS_CAMERA_SERVER_WEBRTC_PEER_HPP
#define ROS_CAMERA_SERVER_WEBRTC_PEER_HPP

#include "ros_camera_server/helpers/json.hpp"

#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>
#include <libsoup/soup.h>
#include <string>

namespace ros_camera_server
{

/// Manages a single WebRTC peer connection (webrtcbin).
/// Handles SDP offer/answer negotiation and ICE candidate exchange
/// over a WebSocket connection provided by the SignalingServer.
/// Async webrtcbin callbacks (signals and promises) hold their own refs on the
/// webrtcbin and the connection instead of referencing this peer, so they remain
/// safe if they fire while the peer is destroyed during session teardown.
class WebRTCPeer
{
public:
  enum class Role { OFFERER, ANSWERER };

  /// Create a WebRTC peer.
  /// @param role Whether this peer creates offers (OFFERER) or answers (ANSWERER).
  /// @param ws_conn The WebSocket connection for signaling (not owned).
  /// @param webrtcbin The webrtcbin GStreamer element (not owned, must be added to a pipeline).
  WebRTCPeer( Role role, SoupWebsocketConnection *ws_conn, GstElement *webrtcbin );
  ~WebRTCPeer();

  WebRTCPeer( const WebRTCPeer & ) = delete;
  WebRTCPeer &operator=( const WebRTCPeer & ) = delete;

  GstElement *webrtcbin() const { return webrtc_; }

  /// Process an incoming signaling message from the remote peer.
  void handleSignalingMessage( const nlohmann::json &msg );

  /// For OFFERER role: explicitly trigger offer creation.
  /// Usually not needed as webrtcbin emits "on-negotiation-needed" automatically.
  void createOffer();

private:
  Role role_;
  GstElement *webrtc_;               // not owned
  SoupWebsocketConnection *ws_conn_; // not owned

  void handleSdp( const std::string &type, const std::string &sdp_str );
  void handleIceCandidate( guint mline_index, const std::string &candidate );

  gulong negotiation_needed_handler_ = 0;
  gulong ice_candidate_handler_ = 0;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_WEBRTC_PEER_HPP
