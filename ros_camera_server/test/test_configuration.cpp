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

#include <gtest/gtest.h>
#include <ros_camera_server/configuration.hpp>
#include <ros_camera_server/outputs/ros2_output.hpp>
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

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
