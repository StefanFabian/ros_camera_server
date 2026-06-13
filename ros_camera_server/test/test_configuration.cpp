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

#include <gst/gst.h>
#include <gtest/gtest.h>
#include <ros_camera_server/configuration.hpp>
#include <ros_camera_server/inputs/ros2_input.hpp>
#include <ros_camera_server/inputs/rtp_input.hpp>
#include <ros_camera_server/inputs/videotestsrc_input.hpp>
#include <ros_camera_server/outputs/ros2_output.hpp>
#include <ros_camera_server/outputs/rtp_output.hpp>
#include <yaml-cpp/yaml.h>

TEST( FramerateTest, TestConstructors )
{
  ros_camera_server::Framerate fr_default;
  EXPECT_EQ( fr_default.numerator, 0 );
  EXPECT_EQ( fr_default.denominator, 0 );
  EXPECT_FALSE( fr_default.isValid() );

  ros_camera_server::Framerate fr_explicit( 30, 1 );
  EXPECT_EQ( fr_explicit.numerator, 30 );
  EXPECT_EQ( fr_explicit.denominator, 1 );
  EXPECT_TRUE( fr_explicit.isValid() );
}

TEST( FramerateTest, TestFromString )
{
  ros_camera_server::Framerate fr( "30/1" );
  EXPECT_EQ( fr.numerator, 30 );
  EXPECT_EQ( fr.denominator, 1 );

  ros_camera_server::Framerate fr2( "15/2" );
  EXPECT_EQ( fr2.numerator, 15 );
  EXPECT_EQ( fr2.denominator, 2 );
}

TEST( FramerateTest, TestComparison )
{
  ros_camera_server::Framerate fr1( 30, 1 );
  ros_camera_server::Framerate fr2( 60, 2 );
  ros_camera_server::Framerate fr3( 15, 1 );

  EXPECT_EQ( fr1, fr2 );
  EXPECT_GT( fr1, fr3 );
  EXPECT_LT( fr3, fr1 );
}

TEST( SizeTest, TestComparison )
{
  ros_camera_server::Size s1( 640, 480 );
  ros_camera_server::Size s2( 640, 480 );
  ros_camera_server::Size s3( 1920, 1080 );
  ros_camera_server::Size s4( 320, 240 );

  EXPECT_EQ( s1, s2 );
  EXPECT_GT( s3, s1 );
  EXPECT_LT( s4, s1 );
}

TEST( StreamFormatTest, TestFromCodec )
{
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "h264" ),
             ros_camera_server::StreamFormat::H264 );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "H264" ),
             ros_camera_server::StreamFormat::H264 );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "h265" ),
             ros_camera_server::StreamFormat::H265 );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "H265" ),
             ros_camera_server::StreamFormat::H265 );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "jpeg" ),
             ros_camera_server::StreamFormat::JPEG );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "JPEG" ),
             ros_camera_server::StreamFormat::JPEG );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "png" ),
             ros_camera_server::StreamFormat::PNG );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "PNG" ),
             ros_camera_server::StreamFormat::PNG );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "raw" ),
             ros_camera_server::StreamFormat::RAW );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "RAW" ),
             ros_camera_server::StreamFormat::RAW );
  EXPECT_EQ( ros_camera_server::stream_format_from_codec( "unknown" ),
             ros_camera_server::StreamFormat::INVALID );
}

TEST( Ros2OutputConfigurationTest, TestCameraInfoUrlRoundTrip )
{
  auto config = YAML::Load( R"(
type: ros2
topic: /camera/test
frame_id: test_camera_optical_frame
codec: raw
format: rgb8
camera_info_url: file:///tmp/test_camera.yaml
width: 320
height: 240
framerate: 15/1
)" );

  auto output = ros_camera_server::Ros2OutputConfiguration::from_yaml_shared( config );
  ASSERT_NE( output, nullptr );
  EXPECT_EQ( output->topic, "/camera/test" );
  EXPECT_EQ( output->frame_id, "test_camera_optical_frame" );
  EXPECT_EQ( output->camera_info_url, "file:///tmp/test_camera.yaml" );

  auto roundtrip = output->toYaml();
  EXPECT_EQ( roundtrip["frame_id"].as<std::string>(), "test_camera_optical_frame" );
  EXPECT_EQ( roundtrip["camera_info_url"].as<std::string>(), "file:///tmp/test_camera.yaml" );
  EXPECT_EQ( roundtrip["topic"].as<std::string>(), "/camera/test" );
  EXPECT_EQ( roundtrip["width"].as<int>(), 320 );
  EXPECT_EQ( roundtrip["height"].as<int>(), 240 );
}

TEST( RtpInputConfigurationTest, ParsesH264 )
{
  auto config = YAML::Load( R"(
type: rtp
codec: h264
port: 5004
address: 239.0.0.1
multicast: true
payload_type: 97
latency_ms: 50
drop_on_latency: true
)" );

  auto input = ros_camera_server::RtpInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->type, "rtp" );
  EXPECT_EQ( input->codec, "h264" );
  EXPECT_EQ( input->port, 5004 );
  EXPECT_EQ( input->address, "239.0.0.1" );
  EXPECT_TRUE( input->multicast );
  EXPECT_EQ( input->payload_type, 97 );
  EXPECT_EQ( input->latency_ms, 50 );
  EXPECT_TRUE( input->drop_on_latency );
}

TEST( RtpInputConfigurationTest, LatencyDefaults )
{
  auto config = YAML::Load( R"(
type: rtp
codec: h264
port: 5004
)" );

  auto input = ros_camera_server::RtpInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->latency_ms, 200 );
  EXPECT_FALSE( input->drop_on_latency );
}

TEST( RtpInputConfigurationTest, DecoderDefaultsToAuto )
{
  auto config = YAML::Load( R"(
type: rtp
codec: h264
port: 5004
)" );

  auto input = ros_camera_server::RtpInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->decoder, "auto" );
}

TEST( RtpInputConfigurationTest, ParsesDecoderField )
{
  auto config = YAML::Load( R"(
type: rtp
codec: h265
port: 5004
decoder: nv|sw
)" );

  auto input = ros_camera_server::RtpInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->decoder, "nv|sw" );
}

TEST( RtpInputConfigurationTest, RejectsUnknownCodec )
{
  auto bad = YAML::Load( R"(
type: rtp
codec: vp9
port: 5004
)" );
  EXPECT_THROW( ros_camera_server::RtpInputConfiguration::from_yaml_shared( bad ),
                ros_camera_server::ConfigurationLoadError );
}

TEST( RtpInputConfigurationTest, RejectsRawCodec )
{
  auto bad = YAML::Load( R"(
type: rtp
codec: raw
port: 5004
)" );
  EXPECT_THROW( ros_camera_server::RtpInputConfiguration::from_yaml_shared( bad ),
                ros_camera_server::ConfigurationLoadError );
}

TEST( RtpInputConfigurationTest, RejectsCodecWithoutRtpDepayloader )
{
  auto bad = YAML::Load( R"(
type: rtp
codec: png
port: 5004
)" );
  EXPECT_THROW( ros_camera_server::RtpInputConfiguration::from_yaml_shared( bad ),
                ros_camera_server::ConfigurationLoadError );
}

TEST( VideoTestSrcInputConfigurationTest, ParsesAllFields )
{
  auto config = YAML::Load( R"(
type: videotestsrc
pattern: ball
format: GRAY8
width: 1280
height: 720
framerate: 15/1
)" );

  auto input = ros_camera_server::VideoTestSrcInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->type, "videotestsrc" );
  EXPECT_EQ( input->pattern, "ball" );
  EXPECT_EQ( input->format, "GRAY8" );
  EXPECT_EQ( input->width, 1280 );
  EXPECT_EQ( input->height, 720 );
  EXPECT_EQ( input->framerate, ros_camera_server::Framerate( 15, 1 ) );
}

TEST( VideoTestSrcInputConfigurationTest, Defaults )
{
  auto config = YAML::Load( R"(
type: videotestsrc
)" );

  auto input = ros_camera_server::VideoTestSrcInputConfiguration::from_yaml_shared( config );
  ASSERT_NE( input, nullptr );
  EXPECT_EQ( input->pattern, "smpte" );
  EXPECT_TRUE( input->format.empty() );
  EXPECT_EQ( input->width, 640 );
  EXPECT_EQ( input->height, 480 );
  EXPECT_FALSE( input->framerate.isValid() );
}

// An unknown format must fail loading instead of producing an empty StreamInput in
// createInput, which the rebuild loop would retry forever as a transient fault.
TEST( VideoTestSrcInputConfigurationTest, RejectsUnknownFormat )
{
  auto config = YAML::Load( R"(
type: videotestsrc
format: not_a_format
)" );

  EXPECT_THROW( ros_camera_server::VideoTestSrcInputConfiguration::from_yaml_shared( config ),
                ros_camera_server::ConfigurationLoadError );
}

// Non-positive dimensions produce unsatisfiable caps that never negotiate
// (0 fps restart loop); they must fail loading like an unknown format does.
TEST( VideoTestSrcInputConfigurationTest, RejectsNonPositiveDimensions )
{
  auto zero_width = YAML::Load( R"(
type: videotestsrc
width: 0
)" );
  EXPECT_THROW( ros_camera_server::VideoTestSrcInputConfiguration::from_yaml_shared( zero_width ),
                ros_camera_server::ConfigurationLoadError );

  auto negative_height = YAML::Load( R"(
type: videotestsrc
height: -480
)" );
  EXPECT_THROW( ros_camera_server::VideoTestSrcInputConfiguration::from_yaml_shared( negative_height ),
                ros_camera_server::ConfigurationLoadError );
}

TEST( RtpOutputConfigurationTest, ParsesAndRoundTrips )
{
  auto config = YAML::Load( R"(
type: rtp
codec: h264
host: 192.168.1.50
port: 5004
multicast: true
ttl: 32
bitrate: 2000
width: 1920
height: 1080
framerate: 30/1
)" );

  auto output = ros_camera_server::RtpOutputConfiguration::from_yaml_shared( config );
  ASSERT_NE( output, nullptr );
  EXPECT_EQ( output->codec, "h264" );
  EXPECT_EQ( output->host, "192.168.1.50" );
  EXPECT_EQ( output->port, 5004 );
  EXPECT_TRUE( output->multicast );
  EXPECT_EQ( output->ttl, 32 );
  EXPECT_EQ( output->bitrate, 2000 );
  EXPECT_EQ( output->width, 1920 );
  EXPECT_EQ( output->height, 1080 );
  EXPECT_TRUE( output->framerate.isValid() );

  auto roundtrip = output->toYaml();
  EXPECT_EQ( roundtrip["codec"].as<std::string>(), "h264" );
  EXPECT_EQ( roundtrip["host"].as<std::string>(), "192.168.1.50" );
  EXPECT_EQ( roundtrip["port"].as<int>(), 5004 );
  EXPECT_TRUE( roundtrip["multicast"].as<bool>() );
  EXPECT_EQ( roundtrip["ttl"].as<int>(), 32 );
}

TEST( RtpOutputConfigurationTest, RawCodecAcceptsOnlyRawInput )
{
  auto config = YAML::Load( R"(
type: rtp
codec: raw
host: 127.0.0.1
port: 5004
)" );
  auto output = ros_camera_server::RtpOutputConfiguration::from_yaml_shared( config );
  ASSERT_NE( output, nullptr );
  ASSERT_EQ( output->supported_input_formats.size(), 1u );
  EXPECT_EQ( output->supported_input_formats[0], ros_camera_server::StreamFormat::RAW );
}

TEST( RtpOutputConfigurationTest, EncodedCodecAcceptsOnlyMatchingInput )
{
  auto h264_config = YAML::Load( R"(
type: rtp
codec: h264
host: 127.0.0.1
port: 5004
)" );
  auto h264_output = ros_camera_server::RtpOutputConfiguration::from_yaml_shared( h264_config );
  ASSERT_NE( h264_output, nullptr );
  ASSERT_EQ( h264_output->supported_input_formats.size(), 1u );
  EXPECT_EQ( h264_output->supported_input_formats[0], ros_camera_server::StreamFormat::H264 );

  auto h265_config = YAML::Load( R"(
type: rtp
codec: h265
host: 127.0.0.1
port: 5004
)" );
  auto h265_output = ros_camera_server::RtpOutputConfiguration::from_yaml_shared( h265_config );
  ASSERT_NE( h265_output, nullptr );
  ASSERT_EQ( h265_output->supported_input_formats.size(), 1u );
  EXPECT_EQ( h265_output->supported_input_formats[0], ros_camera_server::StreamFormat::H265 );
}

TEST( RtpOutputConfigurationTest, CreateOutputLinksRtpbinDataPad )
{
  if ( !gst_is_initialized() )
    gst_init( nullptr, nullptr );

  // Fixed test port; createOutput binds the RTCP socket on port+1 during build.
  // If either is already in use, fail clearly rather than silently.
  constexpr int port = 45004;
  auto config = YAML::Load( R"(
type: rtp
codec: h264
host: 127.0.0.1
port: )" + std::to_string( port ) );

  auto output_config = ros_camera_server::RtpOutputConfiguration::from_yaml_shared( config );
  auto output = output_config->createOutput( {}, "camera", 0 );
  ASSERT_NE( output, nullptr ) << "createOutput failed; port " << ( port + 1 )
                               << " (RTCP) likely in use.";
  ASSERT_NE( output->bin, nullptr );

  GstElement *rtpbin = gst_bin_get_by_name( GST_BIN( output->bin ), "rtp_output_bin_0_rtpbin" );
  ASSERT_NE( rtpbin, nullptr );

  GstPad *data_src = gst_element_get_static_pad( rtpbin, "send_rtp_src_0" );
  ASSERT_NE( data_src, nullptr );

  GstPad *peer = gst_pad_get_peer( data_src );
  EXPECT_NE( peer, nullptr );
  if ( peer )
    gst_object_unref( peer );
  gst_object_unref( data_src );
  gst_object_unref( rtpbin );

  gst_element_set_state( GST_ELEMENT( output->bin ), GST_STATE_NULL );
  gst_object_unref( output->bin );
  output->bin = nullptr;
}

TEST( RtpOutputConfigurationTest, RejectsUnknownCodec )
{
  auto bad = YAML::Load( R"(
type: rtp
codec: vp9
host: 127.0.0.1
port: 5004
)" );
  EXPECT_THROW( ros_camera_server::RtpOutputConfiguration::from_yaml_shared( bad ),
                ros_camera_server::ConfigurationLoadError );
}

TEST( RtpOutputConfigurationTest, RequiresHostAndPort )
{
  auto missing_host = YAML::Load( R"(
type: rtp
codec: h264
port: 5004
)" );
  EXPECT_THROW( ros_camera_server::RtpOutputConfiguration::from_yaml_shared( missing_host ),
                ros_camera_server::ConfigurationLoadError );

  auto missing_port = YAML::Load( R"(
type: rtp
codec: h264
host: 127.0.0.1
)" );
  EXPECT_THROW( ros_camera_server::RtpOutputConfiguration::from_yaml_shared( missing_port ),
                ros_camera_server::ConfigurationLoadError );
}

// rbfimagesink publishes compressed images on `<topic>/compressed`
// (image_transport convention). When the user requests format=jpeg|png on the
// ros2 input, createInput must resolve the same suffix so the subscription
// targets the actual topic on the graph; otherwise it sits forever in the
// deferred-detection loop and never subscribes.
TEST( Ros2InputConfigurationTest, CreateInputAddsCompressedSuffixForJpeg )
{
  if ( !gst_is_initialized() )
    gst_init( nullptr, nullptr );

  ros_camera_server::Ros2InputConfiguration config;
  config.type = "ros2";
  config.topic = "/camera_server/example";
  config.format = ros_camera_server::Ros2InputFormat::JPEG;

  auto stream = config.createInput( {}, "camera" );
  ASSERT_NE( stream.bin, nullptr );

  GstElement *src = gst_bin_get_by_name( stream.bin, "input" );
  ASSERT_NE( src, nullptr );

  gchar *topic = nullptr;
  g_object_get( G_OBJECT( src ), "topic", &topic, nullptr );
  ASSERT_NE( topic, nullptr );
  EXPECT_STREQ( topic, "/camera_server/example/compressed" );
  g_free( topic );

  // No jpegparse in the input bin: rbfimagesrc parses the JPEG header itself
  // and emits caps with width/height/sof-marker/colorspace/sampling. A
  // downstream jpegparse would actually break vajpegdec because it strips the
  // sof-marker / colorspace / sampling fields its strict sink template needs.
  GstElement *parser = gst_bin_get_by_name( stream.bin, "input_parser" );
  EXPECT_EQ( parser, nullptr );
  if ( parser )
    gst_object_unref( parser );

  gst_object_unref( src );
  gst_element_set_state( GST_ELEMENT( stream.bin ), GST_STATE_NULL );
  gst_object_unref( stream.bin );
}

TEST( Ros2InputConfigurationTest, CreateInputKeepsTopicForRaw )
{
  if ( !gst_is_initialized() )
    gst_init( nullptr, nullptr );

  ros_camera_server::Ros2InputConfiguration config;
  config.type = "ros2";
  config.topic = "/camera_server/example";
  config.format = ros_camera_server::Ros2InputFormat::RAW;

  auto stream = config.createInput( {}, "camera" );
  ASSERT_NE( stream.bin, nullptr );

  GstElement *src = gst_bin_get_by_name( stream.bin, "input" );
  ASSERT_NE( src, nullptr );

  gchar *topic = nullptr;
  g_object_get( G_OBJECT( src ), "topic", &topic, nullptr );
  ASSERT_NE( topic, nullptr );
  EXPECT_STREQ( topic, "/camera_server/example" );
  g_free( topic );

  gst_object_unref( src );
  gst_element_set_state( GST_ELEMENT( stream.bin ), GST_STATE_NULL );
  gst_object_unref( stream.bin );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
