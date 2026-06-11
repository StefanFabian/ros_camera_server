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
#include "signaling_server.hpp"
#include "webrtc_callback_context.hpp"

namespace ros_camera_server
{

namespace
{

void sendSdpMessage( SoupWebsocketConnection *conn, const char *type, const gchar *sdp )
{
  nlohmann::json msg;
  msg["type"] = type;
  msg["sdp"] = sdp;
  SignalingServer::sendMessage( conn, msg );
}

/// Shared implementation for the create-offer / create-answer promise callbacks.
/// The promise owns its WebrtcCallbackContext and destroys it on the final promise unref.
void onLocalDescriptionCreated( GstPromise *promise, gpointer user_data, const char *type )
{
  auto *ctx = static_cast<WebrtcCallbackContext *>( user_data );
  // The promise is interrupted (not replied) when webrtcbin goes to NULL because the session
  // was torn down while the offer/answer was being created. gst_promise_get_reply asserts the
  // promise was replied
  GstWebRTCSessionDescription *desc = nullptr;
  if ( gst_promise_wait( promise ) == GST_PROMISE_RESULT_REPLIED ) {
    const GstStructure *reply = gst_promise_get_reply( promise );
    if ( reply != nullptr )
      gst_structure_get( reply, type, GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &desc, nullptr );
  }
  if ( desc != nullptr ) {
    g_signal_emit_by_name( ctx->webrtcbin, "set-local-description", desc, nullptr );

    gchar *sdp_string = gst_sdp_message_as_text( desc->sdp );
    sendSdpMessage( ctx->conn, type, sdp_string );
    g_free( sdp_string );
    gst_webrtc_session_description_free( desc );
  } else {
    SERVER_LOG_ERROR( "WebRTC: failed to create %s", type );
  }
  gst_promise_unref( promise );
}

void onOfferCreated( GstPromise *promise, gpointer user_data )
{ onLocalDescriptionCreated( promise, user_data, "offer" ); }

void onAnswerCreated( GstPromise *promise, gpointer user_data )
{ onLocalDescriptionCreated( promise, user_data, "answer" ); }

void createOfferFor( GstElement *webrtc, SoupWebsocketConnection *conn )
{
  GstPromise *promise = gst_promise_new_with_change_func(
      onOfferCreated, WebrtcCallbackContext::create( webrtc, conn ), WebrtcCallbackContext::destroy );
  g_signal_emit_by_name( webrtc, "create-offer", nullptr, promise );
}

void onNegotiationNeeded( GstElement *, gpointer user_data )
{
  auto *ctx = static_cast<WebrtcCallbackContext *>( user_data );
  SERVER_LOG_DEBUG( "WebRTC: negotiation needed" );
  createOfferFor( ctx->webrtcbin, ctx->conn );
}

void onIceCandidate( GstElement *, guint mline_index, gchar *candidate, gpointer user_data )
{
  auto *ctx = static_cast<WebrtcCallbackContext *>( user_data );
  nlohmann::json msg;
  msg["ice"] = { { "candidate", candidate }, { "sdpMLineIndex", mline_index } };
  SignalingServer::sendMessage( ctx->conn, msg );
}

} // namespace

WebRTCPeer::WebRTCPeer( Role role, SoupWebsocketConnection *ws_conn, GstElement *webrtcbin )
    : role_( role ), webrtc_( webrtcbin ), ws_conn_( ws_conn )
{
  if ( role_ == Role::OFFERER ) {
    negotiation_needed_handler_ = g_signal_connect_data(
        webrtc_, "on-negotiation-needed", G_CALLBACK( onNegotiationNeeded ),
        WebrtcCallbackContext::create( webrtcbin, ws_conn ), WebrtcCallbackContext::destroyClosure,
        static_cast<GConnectFlags>( 0 ) );
  }
  ice_candidate_handler_ =
      g_signal_connect_data( webrtc_, "on-ice-candidate", G_CALLBACK( onIceCandidate ),
                             WebrtcCallbackContext::create( webrtcbin, ws_conn ),
                             WebrtcCallbackContext::destroyClosure, static_cast<GConnectFlags>( 0 ) );
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

void WebRTCPeer::createOffer() { createOfferFor( webrtc_, ws_conn_ ); }

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

  // Log failures; a silently rejected remote description leaves the connection
  // stuck in "connecting" forever with no trace of why.
  GstPromise *promise = gst_promise_new_with_change_func(
      []( GstPromise *promise, gpointer ) {
        // The promise is interrupted (not replied) if webrtcbin went to NULL during teardown;
        // gst_promise_get_reply would assert, so only read the reply once it was replied.
        if ( gst_promise_wait( promise ) == GST_PROMISE_RESULT_REPLIED ) {
          const GstStructure *reply = gst_promise_get_reply( promise );
          if ( reply != nullptr && gst_structure_has_field( reply, "error" ) ) {
            GError *error = nullptr;
            gst_structure_get( reply, "error", G_TYPE_ERROR, &error, nullptr );
            SERVER_LOG_ERROR( "WebRTC: set-remote-description failed: %s",
                              error != nullptr ? error->message : "unknown error" );
            g_clear_error( &error );
          }
        }
        gst_promise_unref( promise );
      },
      nullptr, nullptr );
  g_signal_emit_by_name( webrtc_, "set-remote-description", desc, promise );
  gst_webrtc_session_description_free( desc );

  // If we're the answerer and received an offer, create an answer
  if ( role_ == Role::ANSWERER && type == "offer" ) {
    GstPromise *answer_promise = gst_promise_new_with_change_func(
        onAnswerCreated, WebrtcCallbackContext::create( webrtc_, ws_conn_ ),
        WebrtcCallbackContext::destroy );
    g_signal_emit_by_name( webrtc_, "create-answer", nullptr, answer_promise );
  }
}

void WebRTCPeer::handleIceCandidate( guint mline_index, const std::string &candidate )
{
  if ( webrtc_ ) {
    g_signal_emit_by_name( webrtc_, "add-ice-candidate", mline_index, candidate.c_str() );
  }
}

} // namespace ros_camera_server
