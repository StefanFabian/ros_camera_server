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

#include "webrtc/signaling_server.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <libsoup/soup.h>
#include <thread>

using namespace ros_camera_server;

class SignalingServerTest : public ::testing::Test
{
protected:
  void SetUp() override { server = std::make_unique<SignalingServer>( 12345 ); }

  void TearDown() override
  {
    if ( server )
      server->stop();
  }

  std::unique_ptr<SignalingServer> server;

  struct RequestResult {
    bool success;
    guint status_code;
    std::string body;
    std::string error_msg;
  };

  RequestResult performRequest( const std::string &url )
  {
    RequestResult result{ false, 0, "", "" };
    GMainLoop *loop = g_main_loop_new( nullptr, FALSE );

    std::thread client_thread( [&]() {
      // Use a separate context for the client thread to avoid interfering with the server's context
      GMainContext *context = g_main_context_new();
      g_main_context_push_thread_default( context );

      SoupSession *session = soup_session_new();
      SoupMessage *msg = soup_message_new( "GET", url.c_str() );

      GError *error = nullptr;
      GBytes *body = soup_session_send_and_read( session, msg, nullptr, &error );

      result.status_code = soup_message_get_status( msg );

      if ( error ) {
        result.error_msg = error->message;
        g_error_free( error );
      } else {
        result.success = true;
        if ( body ) {
          gsize len;
          const char *data = (const char *)g_bytes_get_data( body, &len );
          result.body = std::string( data, len );
          g_bytes_unref( body );
        }
      }

      g_object_unref( msg );
      g_object_unref( session );

      g_main_loop_quit( loop );

      g_main_context_pop_thread_default( context );
      g_main_context_unref( context );
    } );

    g_main_loop_run( loop );
    if ( client_thread.joinable() ) {
      client_thread.join();
    }
    g_main_loop_unref( loop );

    return result;
  }
};

TEST_F( SignalingServerTest, CameraListEndpoint )
{
  ASSERT_TRUE( server->start() );

  // Register a camera
  SignalingServer::CameraInfo info;
  info.name = "TestCamera";
  info.codec = "h264";
  info.width = 1920;
  info.height = 1080;
  info.framerate = 30.0;

  server->registerEndpoint( "/cam0", info, nullptr, nullptr, nullptr );

  // Make HTTP request in separate thread
  RequestResult result = performRequest( "http://localhost:12345/api/cameras" );

  if ( !result.success ) {
    FAIL() << "HTTP Request failed: " << result.error_msg;
  }

  ASSERT_EQ( result.status_code, 200 );

  // Parse JSON
  try {
    auto j = nlohmann::json::parse( result.body );
    ASSERT_TRUE( j.is_array() );
    ASSERT_EQ( j.size(), 1 );
    auto cam = j[0];
    EXPECT_EQ( cam["name"], "TestCamera" );
    EXPECT_EQ( cam["path"], "/cam0" );
    EXPECT_EQ( cam["width"], 1920 );
    EXPECT_EQ( cam["height"], 1080 );
    EXPECT_EQ( cam["framerate"], 30.0 );
    EXPECT_EQ( cam["codec"], "h264" );
  } catch ( const std::exception &e ) {
    FAIL() << "JSON parse error: " << e.what() << " Body: " << result.body;
  }
}

// Test that normal requests return 404 or default page
TEST_F( SignalingServerTest, DefaultPage )
{
  ASSERT_TRUE( server->start() );

  RequestResult result = performRequest( "http://localhost:12345/" );

  ASSERT_TRUE( result.success ) << "Request failed: " << result.error_msg;
  ASSERT_EQ( result.status_code, 200 );

  EXPECT_NE( result.body.find( "ROS Camera Server" ), std::string::npos ); // Part of title
}
