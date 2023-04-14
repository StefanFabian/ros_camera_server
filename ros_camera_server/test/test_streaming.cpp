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

#include "camera_pipeline.hpp"
#include "diagnostic_context.hpp"
#include "pipeline_monitor.hpp"
#include "ros_camera_server/camera_server.hpp"
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gtest/gtest.h>
#include <ros_camera_server/factories/pipeline_input_factory.hpp>
#include <ros_camera_server/factories/pipeline_output_factory.hpp>

// Include transport headers
#include "webrtc/webrtc_peer.hpp"
#include <ros_camera_server/inputs/ros2_input.hpp>
#include <ros_camera_server/outputs/ros2_output.hpp>
#include <ros_camera_server/outputs/srt_output.hpp>
#include <ros_camera_server/outputs/webrtc_output.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

using namespace ros_camera_server;

// -----------------------------------------------------------------------------
// Test Context (Avoids static state)
// -----------------------------------------------------------------------------
struct PipelineTestContext {
  std::atomic<int> buffer_count{ 0 };
  std::mutex mutex;
  std::condition_variable cv;

  void callback( GstBuffer * )
  {
    buffer_count++;
    cv.notify_all();
  }
};

struct RosPairContext {
  std::mutex mutex;
  std::condition_variable cv;
  int image_count = 0;
  int camera_info_count = 0;
  sensor_msgs::msg::Image::ConstSharedPtr last_image;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info;
  std::unordered_map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> pending_images;
  std::unordered_map<int64_t, sensor_msgs::msg::CameraInfo::ConstSharedPtr> pending_camera_infos;

  static int64_t stampKey( const std_msgs::msg::Header::_stamp_type &stamp )
  { return static_cast<int64_t>( stamp.sec ) * 1000000000LL + stamp.nanosec; }

  void onImage( const sensor_msgs::msg::Image::ConstSharedPtr &image )
  {
    std::lock_guard<std::mutex> lock( mutex );
    image_count++;
    if ( last_image != nullptr && last_camera_info != nullptr ) {
      return;
    }
    int64_t key = stampKey( image->header.stamp );
    auto camera_info_it = pending_camera_infos.find( key );
    if ( camera_info_it != pending_camera_infos.end() ) {
      last_image = image;
      last_camera_info = camera_info_it->second;
      pending_camera_infos.erase( camera_info_it );
    } else {
      pending_images[key] = image;
    }
    cv.notify_all();
  }

  void onCameraInfo( const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info )
  {
    std::lock_guard<std::mutex> lock( mutex );
    camera_info_count++;
    if ( last_image != nullptr && last_camera_info != nullptr ) {
      return;
    }
    int64_t key = stampKey( camera_info->header.stamp );
    auto image_it = pending_images.find( key );
    if ( image_it != pending_images.end() ) {
      last_image = image_it->second;
      last_camera_info = camera_info;
      pending_images.erase( image_it );
    } else {
      pending_camera_infos[key] = camera_info;
    }
    cv.notify_all();
  }
};

std::string createCalibrationUrl( const std::string &name, int width = 640, int height = 480 )
{
  auto path = std::filesystem::temp_directory_path() / ( name + ".yaml" );
  std::ofstream file( path );
  file << "image_width: " << width << "\n";
  file << "image_height: " << height << "\n";
  file << "camera_name: " << name << "\n";
  file << "camera_matrix:\n";
  file << "  rows: 3\n";
  file << "  cols: 3\n";
  file << "  data: [500.0, 0.0, 320.0, 0.0, 510.0, 240.0, 0.0, 0.0, 1.0]\n";
  file << "distortion_model: plumb_bob\n";
  file << "distortion_coefficients:\n";
  file << "  rows: 1\n";
  file << "  cols: 5\n";
  file << "  data: [0.1, -0.05, 0.001, 0.0, 0.0]\n";
  file << "rectification_matrix:\n";
  file << "  rows: 3\n";
  file << "  cols: 3\n";
  file << "  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n";
  file << "projection_matrix:\n";
  file << "  rows: 3\n";
  file << "  cols: 4\n";
  file << "  data: [500.0, 0.0, 320.0, 0.0, 0.0, 510.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]\n";
  file.close();
  return "file://" + path.string();
}

std::string getCameraInfoTopic( const std::string &resolved_image_topic )
{
  auto separator = resolved_image_topic.find_last_of( '/' );
  if ( separator == std::string::npos )
    return "camera_info";
  if ( separator == 0 )
    return "/camera_info";
  return resolved_image_topic.substr( 0, separator + 1 ) + "camera_info";
}

static void handoff_callback( GstElement *, GstBuffer *buf, GstPad *, gpointer user_data )
{
  auto *ctx = static_cast<PipelineTestContext *>( user_data );
  ctx->callback( buf );
}

template<typename Predicate>
bool waitForCondition( std::chrono::milliseconds timeout, Predicate &&predicate )
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while ( std::chrono::steady_clock::now() < deadline ) {
    if ( predicate() ) {
      return true;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
  return predicate();
}

// -----------------------------------------------------------------------------
// Mock Input
// -----------------------------------------------------------------------------
class TestInputConfiguration : public InputConfiguration
{
public:
  TestInputConfiguration( int w = 640, int h = 480, int num_buffers = 100 )
      : width( w ), height( h ), num_buffers( num_buffers )
  { type = "test_src"; }

  int width;
  int height;
  int num_buffers;

  StreamInput createInput( const rclcpp::Node::SharedPtr &, const std::string & ) const override
  {
    StreamInput input;
    input.format = StreamFormat::RAW;

    // Create bin: videotestsrc ! capsfilter
    static int input_counter = 0;
    std::string name = "test_input_bin_" + std::to_string( input_counter++ );
    input.bin = GST_BIN( gst_bin_new( name.c_str() ) );

    GstElement *src = gst_element_factory_make( "videotestsrc", ( name + "_src" ).c_str() );
    GstElement *capsfilter = gst_element_factory_make( "capsfilter", ( name + "_caps" ).c_str() );
    g_object_set( src, "num-buffers", num_buffers, "is-live", TRUE, nullptr );

    std::string caps_str = "video/x-raw, width=" + std::to_string( width ) +
                           ", height=" + std::to_string( height ) + ", framerate=30/1, format=RGB";
    GstCaps *caps = gst_caps_from_string( caps_str.c_str() );
    g_object_set( capsfilter, "caps", caps, nullptr );
    gst_caps_unref( caps );

    gst_bin_add_many( GST_BIN( input.bin ), src, capsfilter, nullptr );
    gst_element_link( src, capsfilter );

    GstPad *pad = gst_element_get_static_pad( capsfilter, "src" );
    gst_element_add_pad( GST_ELEMENT( input.bin ), gst_ghost_pad_new( "src", pad ) );
    gst_object_unref( pad );

    return input;
  }
};

// -----------------------------------------------------------------------------
// Mock Output
// -----------------------------------------------------------------------------
class TestOutput : public PipelineOutput
{
public:
  TestOutput( const rclcpp::Node::SharedPtr &node, const std::string &camera_id, int output_index,
              PipelineTestContext *ctx )
      : PipelineOutput( node, camera_id, output_index )
  {
    static int counter = 0;
    std::string name = "test_output_bin_" + std::to_string( counter++ );
    bin = GST_BIN( gst_bin_new( name.c_str() ) );

    sink = gst_element_factory_make( "fakesink", ( name + "_sink" ).c_str() );
    g_object_set( sink, "signal-handoffs", TRUE, "sync", FALSE, nullptr );
    g_signal_connect( sink, "handoff", G_CALLBACK( handoff_callback ), ctx );

    gst_bin_add( GST_BIN( bin ), sink );

    GstPad *pad = gst_element_get_static_pad( sink, "sink" );
    gst_element_add_pad( GST_ELEMENT( bin ), gst_ghost_pad_new( "sink", pad ) );
    gst_object_unref( pad );
  }

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration & ) const override
  { return ros_camera_server_msgs::msg::CameraStream(); }

  GstElement *sink;
};

class TestOutputConfiguration : public OutputConfiguration
{
public:
  PipelineTestContext *context_;

  TestOutputConfiguration( PipelineTestContext *ctx )
      : OutputConfiguration( "test_sink" ), context_( ctx )
  { supported_input_formats = { StreamFormat::RAW }; }

  YAML::Node toYaml() const override { return YAML::Node(); }

  PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &node,
                                    const std::string &camera_id, int index ) const override
  { return std::make_unique<TestOutput>( node, camera_id, index, context_ ); }
};

// -----------------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------------
class StreamingTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    gst_init( nullptr, nullptr );
    rclcpp::init( 0, nullptr );
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  static std::shared_ptr<PipelineMonitor> makeMonitor()
  { return std::make_shared<PipelineMonitor>(); }
};

TEST_F( StreamingTest, TestSimplePipeline )
{
  PipelineTestContext context;

  CameraConfiguration config;
  config.id = "test_camera";
  config.name = "Test Camera";
  config.input = std::make_shared<TestInputConfiguration>();
  config.outputs.push_back( std::make_shared<TestOutputConfiguration>( &context ) );

  auto node = std::make_shared<rclcpp::Node>( "test_node_simple" );
  CameraPipeline pipeline( node, config, makeMonitor() );

  EXPECT_NO_THROW( pipeline.buildPipeline() );
  EXPECT_TRUE( pipeline.isBuilt() );

  pipeline.start();

  std::unique_lock<std::mutex> lock( context.mutex );
  bool received = context.cv.wait_for( lock, std::chrono::seconds( 2 ),
                                       [&context] { return context.buffer_count > 0; } );

  EXPECT_TRUE( received ) << "Did not receive any buffers in 2 seconds";
  EXPECT_GT( context.buffer_count, 0 );

  pipeline.stop();
}

TEST_F( StreamingTest, TestSRTPipeline )
{
  // 1. Camera Server Pipeline: TestSrc -> SRT Sink (Listener)
  constexpr int port = 9876;
  constexpr int srt_latency_ms = 200;

  // Software x264 is the slowest part of this test on CI; use a tiny resolution so the encoder
  // has plenty of headroom to produce frames before the test deadline.
  constexpr int width = 160;
  constexpr int height = 120;

  CameraConfiguration config;
  config.id = "srt_server_camera";
  config.name = "SRT Server Camera";
  config.input = std::make_shared<TestInputConfiguration>( width, height );

  auto srt_out = std::make_shared<SrtOutputConfiguration>();
  srt_out->port = port;
  srt_out->codec = "h264";
  srt_out->encoder = "x264"; // Use x264 for reliable software encoding
  srt_out->width = width;
  srt_out->height = height;
  srt_out->bitrate = 512;
  srt_out->latency_ms = srt_latency_ms;
  srt_out->supported_input_formats = { StreamFormat::H264 };

  config.outputs.push_back( srt_out );

  auto node = std::make_shared<rclcpp::Node>( "test_node_srt" );
  CameraPipeline pipeline( node, config, makeMonitor() );

  pipeline.buildPipeline();

  // Production sets wait-for-connection=FALSE so srtsink discards data when no client is
  // attached (live-streaming semantics). For this test, override that: x264enc is configured
  // with intra-refresh=TRUE which produces an IDR only at stream start, so any client that
  // connects after the encoder has begun running has no decodable keyframe. Blocking the
  // producer until the test client is attached guarantees the first IDR is delivered.
  GstElement *srtsink = gst_bin_get_by_name( pipeline.gstPipeline(), "srt_output_bin_0_output" );
  ASSERT_NE( srtsink, nullptr );
  g_object_set( G_OBJECT( srtsink ), "wait-for-connection", TRUE, nullptr );
  gst_object_unref( srtsink );

  pipeline.start();

  // 2. Client Pipeline: SRT Src (Caller) -> FakeSink (with context)
  PipelineTestContext context;
  // messageapi=true must match the server's srtsink (see srt_output.cpp); without it the
  // receiver's framing differs and no buffers are produced downstream.
  // Stop at h264parse: this test verifies SRT transport + RTP depacketization, not decoding.
  // Going through decodebin would pull in avdec_h264, which on some CI images doesn't support
  // the 4:4:4 chroma profile that x264enc produces from RGB input — that failure is unrelated
  // to what we're checking here.
  std::string pipeline_str =
      "srtsrc uri=srt://127.0.0.1:" + std::to_string( port ) +
      "?mode=caller&messageapi=true latency=" + std::to_string( srt_latency_ms ) +
      " ! application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96 ! "
      "rtph264depay ! h264parse ! fakesink name=client_sink signal-handoffs=true";

  GError *error = nullptr;
  GstElement *client_pipeline = gst_parse_launch( pipeline_str.c_str(), &error );
  if ( !client_pipeline ) {
    GTEST_FAIL() << "Failed to parse client pipeline: "
                 << ( error ? error->message : "Unknown error" );
  }

  GstElement *sink = gst_bin_get_by_name( GST_BIN( client_pipeline ), "client_sink" );
  ASSERT_NE( sink, nullptr );
  g_signal_connect( sink, "handoff", G_CALLBACK( handoff_callback ), &context );
  gst_object_unref( sink );

  gst_element_set_state( client_pipeline, GST_STATE_PLAYING );

  // Wait for data
  std::unique_lock<std::mutex> lock( context.mutex );
  bool received = context.cv.wait_for( lock, std::chrono::seconds( 10 ),
                                       [&context] { return context.buffer_count > 0; } );

  EXPECT_TRUE( received ) << "Client did not receive SRT stream";

  gst_element_set_state( client_pipeline, GST_STATE_NULL );
  gst_object_unref( client_pipeline );
  pipeline.stop();
}

TEST_F( StreamingTest, TestROSPipeline )
{
  std::string topic = "/test_image_topic_ros_pipeline";

  auto node_producer = std::make_shared<rclcpp::Node>( "test_node_ros_producer" );
  auto node_consumer = std::make_shared<rclcpp::Node>( "test_node_ros_consumer" );

  // 1. Producer: TestSrc -> ROS Output
  CameraConfiguration producer_config;
  producer_config.id = "ros_producer_camera";
  producer_config.name = "ROS Producer";
  // Use enough buffers to outlast ROS discovery time
  producer_config.input = std::make_shared<TestInputConfiguration>( 640, 480, 500 );

  auto ros_out = std::make_shared<Ros2OutputConfiguration>();
  ros_out->topic = topic;
  ros_out->supported_input_formats = { StreamFormat::RAW };

  producer_config.outputs.push_back( ros_out );

  CameraPipeline producer_pipeline( node_producer, producer_config, makeMonitor() );
  producer_pipeline.buildPipeline();

  // 2. Consumer: ROS Input -> Mock Output
  CameraConfiguration consumer_config;
  consumer_config.id = "ros_consumer_camera";
  consumer_config.name = "ROS Consumer";

  auto ros_in = std::make_shared<Ros2InputConfiguration>();
  ros_in->topic = topic;
  ros_in->type = "ros2";
  ros_in->format = Ros2InputFormat::RAW;

  consumer_config.input = ros_in;

  PipelineTestContext context;
  consumer_config.outputs.push_back( std::make_shared<TestOutputConfiguration>( &context ) );

  CameraPipeline consumer_pipeline( node_consumer, consumer_config, makeMonitor() );
  consumer_pipeline.buildPipeline();

  // Start the executor only after both pipelines are built. rbfimagesink/rbfimagesrc
  // (gstreamer_ros_babel_fish) hold a raw pointer to the rclcpp::Node and access it from
  // the streaming thread; building while a MultiThreadedExecutor is concurrently spinning
  // the same node has caused aborts in CI.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node( node_producer );
  executor.add_node( node_consumer );
  std::thread spinner( [&executor]() { executor.spin(); } );

  producer_pipeline.start();
  consumer_pipeline.start();

  std::unique_lock<std::mutex> lock( context.mutex );
  bool received = context.cv.wait_for( lock, std::chrono::seconds( 10 ),
                                       [&context] { return context.buffer_count > 0; } );

  executor.cancel();
  if ( spinner.joinable() ) {
    spinner.join();
  }

  consumer_pipeline.stop();
  producer_pipeline.stop();

  EXPECT_TRUE( received ) << "ROS consumer did not receive images";
}

TEST_F( StreamingTest, TestROSPipelinePublishesCameraInfo )
{
  std::string topic = "/test_image_with_info";
  std::string frame_id = "test_camera_optical_frame";
  std::string camera_info_url = createCalibrationUrl( "test_camera_info", 320, 240 );

  auto node_producer = std::make_shared<rclcpp::Node>( "test_node_ros_info_producer" );
  auto node_consumer = std::make_shared<rclcpp::Node>( "test_node_ros_info_consumer" );

  auto resolved_topic = node_consumer->get_node_topics_interface()->resolve_topic_name( topic );

  RosPairContext context;
  auto image_sub = node_consumer->create_subscription<sensor_msgs::msg::Image>(
      topic, 10,
      [&context]( const sensor_msgs::msg::Image::ConstSharedPtr &msg ) { context.onImage( msg ); } );
  auto camera_info_sub = node_consumer->create_subscription<sensor_msgs::msg::CameraInfo>(
      getCameraInfoTopic( resolved_topic ), 10,
      [&context]( const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg ) {
        context.onCameraInfo( msg );
      } );

  CameraConfiguration producer_config;
  producer_config.id = "ros_camera_info_producer";
  producer_config.name = "ROS CameraInfo Producer";
  producer_config.input = std::make_shared<TestInputConfiguration>( 640, 480, 500 );

  auto ros_out = std::make_shared<Ros2OutputConfiguration>();
  ros_out->topic = topic;
  ros_out->frame_id = frame_id;
  ros_out->camera_info_url = camera_info_url;
  ros_out->width = 320;
  ros_out->height = 240;
  ros_out->supported_input_formats = { StreamFormat::RAW };
  producer_config.outputs.push_back( ros_out );

  CameraPipeline producer_pipeline( node_producer, producer_config, makeMonitor() );
  producer_pipeline.buildPipeline();

  // Start the executor only after the pipeline is built (see TestROSPipeline for rationale).
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node( node_producer );
  executor.add_node( node_consumer );
  std::thread spinner( [&executor]() { executor.spin(); } );

  producer_pipeline.start();

  std::unique_lock<std::mutex> lock( context.mutex );
  bool received = context.cv.wait_for( lock, std::chrono::seconds( 5 ), [&context] {
    return context.last_image != nullptr && context.last_camera_info != nullptr;
  } );

  executor.cancel();
  if ( spinner.joinable() ) {
    spinner.join();
  }
  producer_pipeline.stop();
  std::filesystem::remove( camera_info_url.substr( std::string( "file://" ).size() ) );

  ASSERT_TRUE( received ) << "Did not receive both image and CameraInfo";
  ASSERT_NE( context.last_image, nullptr );
  ASSERT_NE( context.last_camera_info, nullptr );
  EXPECT_EQ( context.last_image->header.stamp.sec, context.last_camera_info->header.stamp.sec );
  EXPECT_EQ( context.last_image->header.stamp.nanosec,
             context.last_camera_info->header.stamp.nanosec );
  EXPECT_EQ( context.last_image->header.frame_id, frame_id );
  EXPECT_EQ( context.last_camera_info->header.frame_id, frame_id );
  EXPECT_EQ( context.last_image->width, 320u );
  EXPECT_EQ( context.last_image->height, 240u );
  EXPECT_EQ( context.last_camera_info->width, 320u );
  EXPECT_EQ( context.last_camera_info->height, 240u );
  EXPECT_NEAR( context.last_camera_info->k[0], 500.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->k[2], 320.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->k[4], 510.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->k[5], 240.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->p[0], 500.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->p[2], 320.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->p[5], 510.0, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->p[6], 240.0, 1e-6 );
  ASSERT_EQ( context.last_camera_info->d.size(), 5u );
  EXPECT_NEAR( context.last_camera_info->d[0], 0.1, 1e-6 );
  EXPECT_NEAR( context.last_camera_info->d[1], -0.05, 1e-6 );

  (void)image_sub;
  (void)camera_info_sub;
}

TEST_F( StreamingTest, TestROSPipelinePassesCameraInfoUrlToSink )
{
  std::string topic = "/test_image_passthrough_info";
  std::string camera_info_url = createCalibrationUrl( "test_passthrough_camera_info", 320, 240 );

  auto node = std::make_shared<rclcpp::Node>( "test_node_ros_passthrough_info" );

  CameraConfiguration producer_config;
  producer_config.id = "ros_passthrough_info_producer";
  producer_config.name = "ROS Passthrough CameraInfo Producer";
  producer_config.input = std::make_shared<TestInputConfiguration>( 640, 480, 500 );

  auto ros_out = std::make_shared<Ros2OutputConfiguration>();
  ros_out->topic = topic;
  ros_out->camera_info_url = camera_info_url;
  ros_out->width = 320;
  ros_out->height = 240;
  ros_out->supported_input_formats = { StreamFormat::RAW };
  producer_config.outputs.push_back( ros_out );

  CameraPipeline producer_pipeline( node, producer_config, makeMonitor() );
  producer_pipeline.buildPipeline();
  ASSERT_EQ( producer_pipeline.outputs().size(), 1u );
  GstElement *ros_sink =
      gst_bin_get_by_name( producer_pipeline.outputs()[0]->bin, "ros_output_bin_0_output" );
  ASSERT_NE( ros_sink, nullptr );

  gchar *configured_camera_info_url = nullptr;
  g_object_get( G_OBJECT( ros_sink ), "camera-info-url", &configured_camera_info_url, nullptr );

  std::string configured_value =
      configured_camera_info_url != nullptr ? configured_camera_info_url : "";
  g_free( configured_camera_info_url );
  gst_object_unref( ros_sink );
  std::filesystem::remove( camera_info_url.substr( std::string( "file://" ).size() ) );

  EXPECT_EQ( configured_value, camera_info_url );
}

// Test that multiple RAW outputs share the input via tee
TEST_F( StreamingTest, TestMultipleRawOutputs )
{
  PipelineTestContext context1, context2;

  CameraConfiguration config;
  config.id = "test_camera_multi_raw";
  config.name = "Test Camera Multi Raw";
  config.input = std::make_shared<TestInputConfiguration>();
  config.outputs.push_back( std::make_shared<TestOutputConfiguration>( &context1 ) );
  config.outputs.push_back( std::make_shared<TestOutputConfiguration>( &context2 ) );

  auto node = std::make_shared<rclcpp::Node>( "test_node_multi_raw" );
  CameraPipeline pipeline( node, config, makeMonitor() );

  EXPECT_NO_THROW( pipeline.buildPipeline() );
  EXPECT_TRUE( pipeline.isBuilt() );

  pipeline.start();

  // Wait for both outputs to receive buffers
  std::unique_lock<std::mutex> lock1( context1.mutex );
  bool received1 = context1.cv.wait_for( lock1, std::chrono::seconds( 2 ),
                                         [&context1] { return context1.buffer_count > 0; } );

  std::unique_lock<std::mutex> lock2( context2.mutex );
  bool received2 = context2.cv.wait_for( lock2, std::chrono::seconds( 2 ),
                                         [&context2] { return context2.buffer_count > 0; } );

  EXPECT_TRUE( received1 ) << "Output 1 did not receive any buffers";
  EXPECT_TRUE( received2 ) << "Output 2 did not receive any buffers";
  EXPECT_GT( context1.buffer_count, 0 );
  EXPECT_GT( context2.buffer_count, 0 );

  pipeline.stop();
}

// Test output configuration for H264 encoded stream
class TestEncodedOutput : public PipelineOutput
{
public:
  TestEncodedOutput( const rclcpp::Node::SharedPtr &node, const std::string &camera_id,
                     int output_index, PipelineTestContext *ctx )
      : PipelineOutput( node, camera_id, output_index )
  {
    static int counter = 0;
    std::string name = "test_encoded_output_bin_" + std::to_string( counter++ );
    bin = GST_BIN( gst_bin_new( name.c_str() ) );

    // H264 encoded input needs depay/decode for fakesink
    sink = gst_element_factory_make( "fakesink", ( name + "_sink" ).c_str() );
    g_object_set( sink, "signal-handoffs", TRUE, "sync", FALSE, nullptr );
    g_signal_connect( sink, "handoff", G_CALLBACK( handoff_callback ), ctx );

    gst_bin_add( GST_BIN( bin ), sink );

    GstPad *pad = gst_element_get_static_pad( sink, "sink" );
    gst_element_add_pad( GST_ELEMENT( bin ), gst_ghost_pad_new( "sink", pad ) );
    gst_object_unref( pad );
  }

  ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration & ) const override
  { return ros_camera_server_msgs::msg::CameraStream(); }

  GstElement *sink;
};

class TestEncodedOutputConfiguration : public OutputConfiguration
{
public:
  PipelineTestContext *context_;

  TestEncodedOutputConfiguration( PipelineTestContext *ctx )
      : OutputConfiguration( "test_encoded_sink" ), context_( ctx )
  {
    supported_input_formats = { StreamFormat::H264 };
    codec = "h264";
    encoder = "x264"; // Use x264 for reliable software encoding
    width = 640;
    height = 480;
  }

  YAML::Node toYaml() const override { return YAML::Node(); }

  PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &node,
                                    const std::string &camera_id, int index ) const override
  { return std::make_unique<TestEncodedOutput>( node, camera_id, index, context_ ); }
};

// Test that multiple H264 outputs share a single encoder via tee
TEST_F( StreamingTest, TestMultipleEncodedOutputs )
{
  PipelineTestContext context1, context2;

  CameraConfiguration config;
  config.id = "test_camera_multi_h264";
  config.name = "Test Camera Multi H264";
  config.input = std::make_shared<TestInputConfiguration>();
  config.outputs.push_back( std::make_shared<TestEncodedOutputConfiguration>( &context1 ) );
  config.outputs.push_back( std::make_shared<TestEncodedOutputConfiguration>( &context2 ) );

  auto node = std::make_shared<rclcpp::Node>( "test_node_multi_h264" );
  CameraPipeline pipeline( node, config, makeMonitor() );

  // This should build successfully with encoder + tee
  EXPECT_NO_THROW( pipeline.buildPipeline() );
  EXPECT_TRUE( pipeline.isBuilt() );

  GstElement *encoder = gst_bin_get_by_name( pipeline.gstPipeline(), "encoder_node_2" );
  ASSERT_NE( encoder, nullptr );
  auto encoder_context = ros_camera_server::getDiagnosticContext( G_OBJECT( encoder ) );
  ASSERT_NE( encoder_context, nullptr );
  EXPECT_EQ( encoder_context->camera_id, "test_camera_multi_h264" );
  ASSERT_EQ( encoder_context->affected_outputs.size(), 2u );
  EXPECT_EQ( encoder_context->affected_outputs[0], 0u );
  EXPECT_EQ( encoder_context->affected_outputs[1], 1u );

  GstPad *encoder_src_pad = gst_element_get_static_pad( encoder, "src" );
  ASSERT_NE( encoder_src_pad, nullptr );
  EXPECT_EQ( ros_camera_server::getDiagnosticContext( G_OBJECT( encoder_src_pad ) ), encoder_context );
  gst_object_unref( encoder_src_pad );
  gst_object_unref( encoder );

  pipeline.start();

  // Wait for both encoded outputs to receive buffers
  std::unique_lock<std::mutex> lock1( context1.mutex );
  bool received1 = context1.cv.wait_for( lock1, std::chrono::seconds( 5 ),
                                         [&context1] { return context1.buffer_count > 0; } );

  std::unique_lock<std::mutex> lock2( context2.mutex );
  bool received2 = context2.cv.wait_for( lock2, std::chrono::seconds( 5 ),
                                         [&context2] { return context2.buffer_count > 0; } );

  EXPECT_TRUE( received1 ) << "Encoded output 1 did not receive any buffers";
  EXPECT_TRUE( received2 ) << "Encoded output 2 did not receive any buffers";
  EXPECT_GT( context1.buffer_count, 0 );
  EXPECT_GT( context2.buffer_count, 0 );

  pipeline.stop();
}

TEST_F( StreamingTest, TestWebRTCURI )
{
  auto node = std::make_shared<rclcpp::Node>( "test_node_webrtc_uri" );

  // Use configuration to create output via factory method
  WebrtcOutputConfiguration output_config;
  output_config.codec = "h264";

  auto output = output_config.createOutput( node, "test_camera", 0 );
  ASSERT_NE( output, nullptr );

  CameraServerConfiguration config;
  config.address = "192.168.1.100";
  config.signaling_port = 8443;

  auto msg = output->toCameraStreamMsg( config );

  EXPECT_EQ( msg.transport, "webrtc" );
  // Expected: ws://<address>:<port>/<camera_id>/<output_index>
  EXPECT_EQ( msg.uri, "ws://192.168.1.100:8443/test_camera/0" );
}

// Regression test: a previous version of the GStreamer log handler in CameraServer ran processing
// synchronously on the GStreamer thread, which broke WebRTC negotiation. The handler now only
// enqueues records; processing happens on a separate thread via
// PipelineMonitor::processLogMessages(). This test drives a WebRTC stream end-to-end through
// CameraServer (which installs the log handler) and verifies the client receives buffers.
TEST_F( StreamingTest, FullWebRTCPipeline )
{
  CameraConfiguration camera;
  camera.id = "webrtc_camera";
  camera.name = "WebRTC Camera";
  camera.input = std::make_shared<TestInputConfiguration>( 640, 480, 500 );

  auto webrtc_out = std::make_shared<WebrtcOutputConfiguration>();
  webrtc_out->codec = "h264";
  webrtc_out->width = 640;
  webrtc_out->height = 480;
  webrtc_out->supported_input_formats = { StreamFormat::H264 };
  webrtc_out->encoder = "x264";
  camera.outputs.push_back( webrtc_out );

  CameraServerConfiguration config;
  config.signaling_port = 0; // OS picks a free port
  config.address = "127.0.0.1";
  config.cameras.push_back( camera );

  auto node = std::make_shared<rclcpp::Node>( "test_node_webrtc_camera_server" );
  node->declare_parameter( "max_processing_time", 2.0 );
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node( node );
  std::thread executor_thread( [&executor]() { executor.spin(); } );

  std::optional<CameraServer> server;
  server.emplace( node, config );

  const bool server_ready = waitForCondition( std::chrono::seconds( 10 ),
                                              [&server]() { return server->isInitialized(); } );
  ASSERT_TRUE( server_ready );

  const int port = server->signalingPort();
  ASSERT_GT( port, 0 ) << "Signaling server failed to start (port == 0 after isInitialized)";

  // pipeline->start() initiates the state change asynchronously; wait until the pipeline is
  // actually PLAYING and producing buffers. Otherwise the encoder hasn't propagated caps when the
  // client connects and webrtcbin's create-offer fails with "payload type range" because the
  // freshly-added peer branch sees only template caps.
  ASSERT_FALSE( server->pipelines().empty() );
  const bool playing = waitForCondition( std::chrono::seconds( 5 ), [&server]() {
    const auto &pipeline = server->pipelines().front();
    return pipeline->getState() == GST_STATE_PLAYING && pipeline->statistics().input_fps > 0.0f;
  } );
  ASSERT_TRUE( playing ) << "Pipeline did not reach PLAYING state with active data flow";

  // Run the default GMainContext on a dedicated thread so soup async callbacks fire.
  GMainLoop *loop = g_main_loop_new( nullptr, FALSE );
  std::thread loop_thread( [loop]() { g_main_loop_run( loop ); } );

  PipelineTestContext context;
  std::string client_launch =
      "webrtcbin name=client_webrtc bundle-policy=max-bundle ! rtph264depay ! h264parse ! fakesink "
      "name=client_sink signal-handoffs=true";

  GError *error = nullptr;
  GstElement *client_pipeline = gst_parse_launch( client_launch.c_str(), &error );
  ASSERT_NE( client_pipeline, nullptr )
      << "Failed to parse client pipeline: " << ( error ? error->message : "Unknown" );

  GstElement *client_webrtc = gst_bin_get_by_name( GST_BIN( client_pipeline ), "client_webrtc" );
  GstElement *client_sink = gst_bin_get_by_name( GST_BIN( client_pipeline ), "client_sink" );
  ASSERT_NE( client_webrtc, nullptr );
  ASSERT_NE( client_sink, nullptr );

  g_signal_connect( client_sink, "handoff", G_CALLBACK( handoff_callback ), &context );
  gst_element_set_state( client_pipeline, GST_STATE_PLAYING );

  struct ClientState {
    std::unique_ptr<WebRTCPeer> peer;
    std::string error_message;
    bool connected = false;
    std::mutex mutex;
    std::condition_variable cv;
  } state;

  SoupSession *session = soup_session_new();
  std::string uri = "ws://127.0.0.1:" + std::to_string( port ) + "/webrtc_camera/0";
  SoupMessage *msg = soup_message_new( "GET", uri.c_str() );
  g_object_set_data( G_OBJECT( session ), "webrtc_elem", client_webrtc );

  soup_session_websocket_connect_async(
      session, msg, nullptr, nullptr, G_PRIORITY_DEFAULT, nullptr,
      []( GObject *session_obj, GAsyncResult *res, gpointer user_data ) {
        auto *s = static_cast<ClientState *>( user_data );
        auto *client_webrtc_elem =
            static_cast<GstElement *>( g_object_get_data( session_obj, "webrtc_elem" ) );

        GError *err = nullptr;
        SoupWebsocketConnection *conn =
            soup_session_websocket_connect_finish( SOUP_SESSION( session_obj ), res, &err );

        if ( err ) {
          {
            std::lock_guard<std::mutex> lock( s->mutex );
            s->error_message = err->message == nullptr ? "unknown error" : err->message;
          }
          g_error_free( err );
          s->cv.notify_all();
          return;
        }

        std::lock_guard<std::mutex> lock( s->mutex );
        s->peer =
            std::make_unique<WebRTCPeer>( WebRTCPeer::Role::ANSWERER, conn, client_webrtc_elem );
        g_signal_connect( conn, "message",
                          G_CALLBACK( +[]( SoupWebsocketConnection *, gint type, GBytes *message,
                                           gpointer user_data ) {
                            if ( type != SOUP_WEBSOCKET_DATA_TEXT )
                              return;
                            auto *peer = static_cast<WebRTCPeer *>( user_data );

                            gsize len;
                            const char *data = (const char *)g_bytes_get_data( message, &len );
                            std::string msg_str( data, len );

                            try {
                              auto json = nlohmann::json::parse( msg_str );
                              peer->handleSignalingMessage( json );
                            } catch ( ... ) {
                            }
                          } ),
                          s->peer.get() );

        s->connected = true;
        s->cv.notify_all();
      },
      &state );

  {
    std::unique_lock<std::mutex> lock( state.mutex );
    const bool connected =
        state.cv.wait_for( lock, std::chrono::seconds( 5 ), [&state] { return state.connected; } );
    ASSERT_TRUE( connected ) << "Failed to establish WebSocket connection: " << state.error_message;
  }

  // Wait for data
  {
    std::unique_lock<std::mutex> lock( context.mutex );
    const bool received = context.cv.wait_for( lock, std::chrono::seconds( 15 ),
                                               [&context] { return context.buffer_count > 0; } );
    EXPECT_TRUE( received ) << "Client did not receive WebRTC stream through CameraServer";
  }

  // Teardown (LIFO).
  gst_element_set_state( client_pipeline, GST_STATE_NULL );
  {
    std::lock_guard<std::mutex> lock( state.mutex );
    state.peer.reset();
  }
  gst_object_unref( client_sink );
  gst_object_unref( client_webrtc );
  gst_object_unref( client_pipeline );
  g_object_unref( session );

  g_main_loop_quit( loop );
  loop_thread.join();
  g_main_loop_unref( loop );

  // Destroy CameraServer while the executor is still spinning so any in-flight ROS timer
  // callback can finish cleanly.
  server.reset();

  executor.cancel();
  executor_thread.join();
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
