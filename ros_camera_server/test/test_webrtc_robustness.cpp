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

/// Robustness tests for the WebRTC output: clients that vanish without a close frame,
/// stall negotiation, disconnect mid-negotiation, reconnect in tight loops, and pipeline
/// restarts / server shutdown racing live sessions. Each scenario ends by asserting the
/// server still serves a fresh streaming client, so a wedged signaling thread or a dead
/// send path fails the test instead of going unnoticed.

#include "camera_pipeline.hpp"
#include "ros_camera_server/camera_server.hpp"
#include "webrtc/signaling_server.hpp"
#include "webrtc_test_client.hpp"

#include <gst/gst.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros_camera_server/inputs/videotestsrc_input.hpp>
#include <ros_camera_server/outputs/webrtc_output.hpp>

#include <chrono>
#include <optional>
#include <thread>

using namespace ros_camera_server;
using namespace ros_camera_server_test;
using namespace std::chrono_literals;

namespace
{

template<typename Predicate>
bool waitForCondition( std::chrono::milliseconds timeout, Predicate &&predicate )
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while ( std::chrono::steady_clock::now() < deadline ) {
    if ( predicate() )
      return true;
    std::this_thread::sleep_for( 10ms );
  }
  return predicate();
}

} // namespace

class WebRtcRobustnessTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    gst_init( nullptr, nullptr );
    rclcpp::init( 0, nullptr );
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    static int test_counter = 0;
    node_ = std::make_shared<rclcpp::Node>( "webrtc_robustness_node_" +
                                            std::to_string( test_counter++ ) );
    node_->declare_parameter( "max_processing_time", 10.0 );
    executor_.emplace();
    executor_->add_node( node_ );
    executor_thread_ = std::thread( [this]() { executor_->spin(); } );
    startServer();
  }

  void TearDown() override
  {
    server_.reset();
    executor_->cancel();
    executor_thread_.join();
    executor_.reset();
    node_.reset();
  }

  void startServer()
  {
    CameraConfiguration camera;
    camera.id = "robust_cam";
    camera.name = "Robustness Camera";
    auto input = std::make_shared<VideoTestSrcInputConfiguration>();
    input->type = "videotestsrc";
    input->width = 320;
    input->height = 240;
    input->framerate = Framerate( 30, 1 );
    camera.input = input;

    auto webrtc_out = std::make_shared<WebrtcOutputConfiguration>();
    webrtc_out->codec = "h264";
    webrtc_out->width = 320;
    webrtc_out->height = 240;
    webrtc_out->supported_input_formats = { StreamFormat::H264 };
    webrtc_out->encoder = "x264";
    camera.outputs.push_back( webrtc_out );

    CameraServerConfiguration config;
    config.signaling_port = 0; // OS picks a free port
    config.address = "127.0.0.1";
    config.cameras.push_back( camera );

    server_.emplace( node_, config );
    ASSERT_TRUE( waitForCondition( 10s, [this]() { return server_->isInitialized(); } ) );
    port_ = server_->signalingPort();
    ASSERT_GT( port_, 0 );
    // Wait until the pipeline is PLAYING with active data flow so the encoder has
    // negotiated caps before clients connect (see FullWebRTCPipeline).
    ASSERT_FALSE( server_->pipelines().empty() );
    ASSERT_TRUE( waitForCondition( 10s, [this]() {
      const auto &pipeline = server_->pipelines().front();
      return pipeline->getState() == GST_STATE_PLAYING && pipeline->statistics().input_fps > 0.0f;
    } ) );
  }

  /// Restart the pipeline the way production does: dispatched onto the GStreamer thread's
  /// main context (checkAndRepair never calls restart() from another thread).
  void restartPipelineOnGstThread()
  {
    GMainContext *context = SignalingServer::mainContext();
    ASSERT_NE( context, nullptr );
    auto *pipeline = server_->pipelines().front().get();
    g_main_context_invoke(
        context,
        []( gpointer data ) -> gboolean {
          static_cast<CameraPipeline *>( data )->restart();
          return G_SOURCE_REMOVE;
        },
        pipeline );
  }

  /// The end-of-scenario health check: a brand-new client must connect, negotiate and
  /// receive media. Fails if the signaling thread is wedged or the send path is dead.
  void expectFreshClientStreams( const char *when )
  {
    WebRtcTestClient probe( port_, "/robust_cam/0" );
    EXPECT_TRUE( probe.waitConnected( 5s ) ) << when << ": probe client could not connect";
    EXPECT_TRUE( probe.waitBuffers( 10, 15s ) )
        << when << ": probe client did not receive media (got " << probe.bufferCount()
        << " buffers)";
  }

  rclcpp::Node::SharedPtr node_;
  std::optional<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::optional<CameraServer> server_;
  int port_ = 0;
};

// A client that connects but never answers the offer leaves the peer's send path blocked
// inside webrtcbin. The server must close such sessions via its establish timeout and stay
// fully functional.
TEST_F( WebRtcRobustnessTest, MuteClientClosedByEstablishTimeout )
{
  WebRtcTestClient mute( port_, "/robust_cam/0", WebRtcTestClient::Mode::MUTE );
  ASSERT_TRUE( mute.waitConnected( 5s ) );
  // PEER_ESTABLISH_TIMEOUT_SECONDS is 10s; allow slack for the close handshake.
  EXPECT_TRUE( mute.waitClosed( 15s ) ) << "server did not close the stalled session";
  expectFreshClientStreams( "after mute client teardown" );
}

// A client whose process dies mid-stream sends no websocket close frame; the server sees
// the TCP stream drop. Its session must be torn down and new clients must keep working.
TEST_F( WebRtcRobustnessTest, AbruptClientDeathMidStream )
{
  for ( int i = 0; i < 3; ++i ) {
    WebRtcTestClient client( port_, "/robust_cam/0" );
    ASSERT_TRUE( client.waitConnected( 5s ) ) << "iteration " << i;
    ASSERT_TRUE( client.waitBuffers( 10, 15s ) ) << "iteration " << i;
    client.disconnectAbrupt();
    EXPECT_TRUE( client.waitClosed( 5s ) ) << "iteration " << i;
  }
  expectFreshClientStreams( "after abrupt client deaths" );
}

// Disconnecting between offer and answer tears the session down mid-negotiation while
// webrtcbin is still working on the offer.
TEST_F( WebRtcRobustnessTest, DisconnectDuringNegotiation )
{
  for ( int i = 0; i < 3; ++i ) {
    WebRtcTestClient client( port_, "/robust_cam/0", WebRtcTestClient::Mode::CLOSE_AFTER_OFFER );
    ASSERT_TRUE( client.waitConnected( 5s ) ) << "iteration " << i;
    ASSERT_TRUE( client.waitClosed( 10s ) ) << "iteration " << i;
    EXPECT_TRUE( client.offerReceived() ) << "iteration " << i;
  }
  expectFreshClientStreams( "after mid-negotiation disconnects" );
}

// Tight reconnect loop: stream briefly, vanish without close frame, reconnect immediately.
// Mimics a flaky client (e.g. the QML viewer on an unstable link) hammering the server.
TEST_F( WebRtcRobustnessTest, ReconnectStorm )
{
  for ( int i = 0; i < 6; ++i ) {
    WebRtcTestClient client( port_, "/robust_cam/0" );
    ASSERT_TRUE( client.waitConnected( 5s ) ) << "iteration " << i;
    ASSERT_TRUE( client.waitBuffers( 1, 15s ) ) << "iteration " << i;
    if ( i % 2 == 0 ) {
      client.disconnectAbrupt();
    }
    // Odd iterations: destructor closes cleanly while media is flowing.
  }
  expectFreshClientStreams( "after reconnect storm" );
}

// Pipeline restarts (checkAndRepair does this on the ROS timer thread when it detects
// stalls or overload) must coexist with live sessions: sessions are closed so clients
// renegotiate, nothing deadlocks, and new clients stream afterwards.
TEST_F( WebRtcRobustnessTest, PipelineRestartWhileStreaming )
{
  for ( int i = 0; i < 3; ++i ) {
    WebRtcTestClient client( port_, "/robust_cam/0" );
    ASSERT_TRUE( client.waitConnected( 5s ) ) << "iteration " << i;
    ASSERT_TRUE( client.waitBuffers( 10, 15s ) ) << "iteration " << i;
    restartPipelineOnGstThread();
    // The restart closes the signaling connection (transports do not survive NULL).
    EXPECT_TRUE( client.waitClosed( 10s ) ) << "iteration " << i;
    ASSERT_TRUE( waitForCondition( 10s,
                                   [this]() {
                                     const auto &pipeline = server_->pipelines().front();
                                     return pipeline->getState() == GST_STATE_PLAYING &&
                                            pipeline->statistics().input_fps > 0.0f;
                                   } ) )
        << "pipeline did not come back after restart " << i;
  }
  expectFreshClientStreams( "after pipeline restarts" );
}

// The nasty combination: a session whose transport never established has its send path
// pad-blocked inside webrtcbin. A pipeline restart landing in that window must not
// deadlock the state change against the blocked streaming thread.
TEST_F( WebRtcRobustnessTest, PipelineRestartWithStalledPeer )
{
  for ( int i = 0; i < 3; ++i ) {
    WebRtcTestClient mute( port_, "/robust_cam/0", WebRtcTestClient::Mode::MUTE );
    ASSERT_TRUE( mute.waitConnected( 5s ) ) << "iteration " << i;
    // Streaming client so data flows into the stalled peer's branch.
    WebRtcTestClient streamer( port_, "/robust_cam/0" );
    ASSERT_TRUE( streamer.waitConnected( 5s ) ) << "iteration " << i;
    ASSERT_TRUE( streamer.waitBuffers( 5, 15s ) ) << "iteration " << i;

    restartPipelineOnGstThread();

    EXPECT_TRUE( mute.waitClosed( 15s ) ) << "iteration " << i;
    EXPECT_TRUE( streamer.waitClosed( 15s ) ) << "iteration " << i;
    ASSERT_TRUE( waitForCondition( 10s,
                                   [this]() {
                                     const auto &pipeline = server_->pipelines().front();
                                     return pipeline->getState() == GST_STATE_PLAYING &&
                                            pipeline->statistics().input_fps > 0.0f;
                                   } ) )
        << "pipeline did not come back after restart " << i;
  }
  expectFreshClientStreams( "after restarts with stalled peers" );
}

// Drive the real self-repair path: checkAndRepair() (ROS timer thread) restarts the pipeline
// whenever an output exceeds max_processing_time, dispatched onto the GStreamer thread. With
// the limit forced to 1µs every tick that sees a streaming client restarts, while clients
// keep connecting, streaming and vanishing.
TEST_F( WebRtcRobustnessTest, RepairLoopRestartsUnderClientChurn )
{
  node_->set_parameter( rclcpp::Parameter( "max_processing_time", 0.000001 ) );
  const auto deadline = std::chrono::steady_clock::now() + 25s;
  while ( std::chrono::steady_clock::now() < deadline ) {
    WebRtcTestClient client( port_, "/robust_cam/0" );
    if ( !client.waitConnected( 5s ) )
      continue;
    // Produce processing-time samples so the repair rule fires; the restart may close the
    // session at any point, so no assertion on the buffers.
    client.waitBuffers( 5, 5s );
  }
  node_->set_parameter( rclcpp::Parameter( "max_processing_time", 10.0 ) );
  ASSERT_TRUE( waitForCondition( 15s,
                                 [this]() {
                                   const auto &pipeline = server_->pipelines().front();
                                   return pipeline->getState() == GST_STATE_PLAYING &&
                                          pipeline->statistics().input_fps > 0.0f;
                                 } ) )
      << "pipeline did not settle after repair-loop restarts";
  expectFreshClientStreams( "after repair-loop restarts" );
}

// Destroying the server with live sessions (streaming + stalled) must not hang, and a new
// server instance must serve reconnecting clients. Covers the "server died and the client
// reconnected" case from the operator's perspective.
TEST_F( WebRtcRobustnessTest, ServerShutdownWithLiveClientsAndReconnect )
{
  auto mute =
      std::make_unique<WebRtcTestClient>( port_, "/robust_cam/0", WebRtcTestClient::Mode::MUTE );
  ASSERT_TRUE( mute->waitConnected( 5s ) );
  auto streamer = std::make_unique<WebRtcTestClient>( port_, "/robust_cam/0" );
  ASSERT_TRUE( streamer->waitConnected( 5s ) );
  ASSERT_TRUE( streamer->waitBuffers( 10, 15s ) );

  // Hangs here (e.g. a state change deadlocking against a blocked streaming thread or the
  // signaling context) fail the test via the suite timeout.
  server_.reset();

  // Both clients observe the connection drop (no close frame is guaranteed).
  EXPECT_TRUE( streamer->waitClosed( 10s ) );
  EXPECT_TRUE( mute->waitClosed( 10s ) );
  streamer.reset();
  mute.reset();

  // Restart the server (new port: the OS assigns a fresh one) and reconnect.
  startServer();
  expectFreshClientStreams( "after server restart" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
