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

#include "webrtc_peer.hpp"
#include "../logging.hpp"

namespace ros_camera_server
{

WebRTCPeer::WebRTCPeer( Role role, SoupWebsocketConnection *ws_conn, GstElement *webrtcbin )
    : role_( role ), webrtc_( webrtcbin ), ws_conn_( ws_conn )
{
  if ( role_ == Role::OFFERER ) {
    negotiation_needed_handler_ = g_signal_connect( webrtc_, "on-negotiation-needed",
                                                    G_CALLBACK( onNegotiationNeeded ), this );
  }
  ice_candidate_handler_ =
      g_signal_connect( webrtc_, "on-ice-candidate", G_CALLBACK( onIceCandidate ), this );
}

WebRTCPeer::~WebRTCPeer()
{
  if ( webrtc_ ) {
    if ( negotiation_needed_handler_ )
      g_signal_handler_disconnect( webrtc_, negotiation_needed_handler_ );
    if ( ice_candidate_handler_ )
      g_signal_handler_disconnect( webrtc_, ice_candidate_handler_ );
  }
}

void WebRTCPeer::handleSignalingMessage( const nlohmann::json &msg )
{
  if ( msg.contains( "type" ) && msg.contains( "sdp" ) ) {
    std::string type = msg["type"].get<std::string>();
    std::string sdp = msg["sdp"].get<std::string>();
    handleSdp( type, sdp );
  } else if ( msg.contains( "ice" ) ) {
    const auto &ice = msg["ice"];
    if ( ice.contains( "candidate" ) && ice.contains( "sdpMLineIndex" ) ) {
      std::string candidate = ice["candidate"].get<std::string>();
      guint mline_index = ice["sdpMLineIndex"].get<guint>();
      handleIceCandidate( mline_index, candidate );
    }
  }
}

void WebRTCPeer::createOffer()
{
  GstPromise *promise = gst_promise_new_with_change_func( onOfferCreated, this, nullptr );
  g_signal_emit_by_name( webrtc_, "create-offer", nullptr, promise );
}

void WebRTCPeer::handleSdp( const std::string &type, const std::string &sdp_str )
{
  if ( !webrtc_ )
    return;

  GstSDPMessage *sdp;
  if ( gst_sdp_message_new( &sdp ) != GST_SDP_OK ) {
    SERVER_LOG_ERROR( "WebRTC: failed to create SDP message" );
    return;
  }
  if ( gst_sdp_message_parse_buffer( reinterpret_cast<const guint8 *>( sdp_str.c_str() ),
                                     sdp_str.size(), sdp ) != GST_SDP_OK ) {
    SERVER_LOG_ERROR( "WebRTC: failed to parse SDP" );
    gst_sdp_message_free( sdp );
    return;
  }

  GstWebRTCSDPType type_enum =
      ( type == "offer" ) ? GST_WEBRTC_SDP_TYPE_OFFER : GST_WEBRTC_SDP_TYPE_ANSWER;
  GstWebRTCSessionDescription *desc = gst_webrtc_session_description_new( type_enum, sdp );

  GstPromise *promise = gst_promise_new();
  g_signal_emit_by_name( webrtc_, "set-remote-description", desc, promise );
  gst_promise_unref( promise );
  gst_webrtc_session_description_free( desc );

  // If we're the answerer and received an offer, create an answer
  if ( role_ == Role::ANSWERER && type == "offer" ) {
    GstPromise *answer_promise = gst_promise_new_with_change_func( onAnswerCreated, this, nullptr );
    g_signal_emit_by_name( webrtc_, "create-answer", nullptr, answer_promise );
  }
}

void WebRTCPeer::handleIceCandidate( guint mline_index, const std::string &candidate )
{
  if ( webrtc_ ) {
    g_signal_emit_by_name( webrtc_, "add-ice-candidate", mline_index, candidate.c_str() );
  }
}

void WebRTCPeer::sendSdp( const std::string &type, const std::string &sdp_str )
{
  nlohmann::json msg;
  msg["type"] = type;
  msg["sdp"] = sdp_str;
  SignalingServer::sendMessage( ws_conn_, msg );
}

void WebRTCPeer::sendIceCandidate( guint mline_index, const gchar *candidate )
{
  nlohmann::json msg;
  msg["ice"] = { { "candidate", candidate }, { "sdpMLineIndex", mline_index } };
  SignalingServer::sendMessage( ws_conn_, msg );
}

void WebRTCPeer::onNegotiationNeeded( GstElement *, gpointer user_data )
{
  auto *self = static_cast<WebRTCPeer *>( user_data );
  SERVER_LOG_DEBUG( "WebRTC: negotiation needed" );
  self->createOffer();
}

void WebRTCPeer::onIceCandidate( GstElement *, guint mline_index, gchar *candidate,
                                 gpointer user_data )
{
  auto *self = static_cast<WebRTCPeer *>( user_data );
  self->sendIceCandidate( mline_index, candidate );
}

void WebRTCPeer::onOfferCreated( GstPromise *promise, gpointer user_data )
{
  auto *self = static_cast<WebRTCPeer *>( user_data );
  const GstStructure *reply = gst_promise_get_reply( promise );
  GstWebRTCSessionDescription *offer = nullptr;
  gst_structure_get( reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr );
  gst_promise_unref( promise );

  if ( !offer ) {
    SERVER_LOG_ERROR( "WebRTC: failed to create offer" );
    return;
  }

  g_signal_emit_by_name( self->webrtc_, "set-local-description", offer, nullptr );

  gchar *sdp_string = gst_sdp_message_as_text( offer->sdp );
  self->sendSdp( "offer", sdp_string );
  g_free( sdp_string );
  gst_webrtc_session_description_free( offer );
}

void WebRTCPeer::onAnswerCreated( GstPromise *promise, gpointer user_data )
{
  auto *self = static_cast<WebRTCPeer *>( user_data );
  const GstStructure *reply = gst_promise_get_reply( promise );
  GstWebRTCSessionDescription *answer = nullptr;
  gst_structure_get( reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr );
  gst_promise_unref( promise );

  if ( !answer ) {
    SERVER_LOG_ERROR( "WebRTC: failed to create answer" );
    return;
  }

  g_signal_emit_by_name( self->webrtc_, "set-local-description", answer, nullptr );

  gchar *sdp_string = gst_sdp_message_as_text( answer->sdp );
  self->sendSdp( "answer", sdp_string );
  g_free( sdp_string );
  gst_webrtc_session_description_free( answer );
}

} // namespace ros_camera_server
