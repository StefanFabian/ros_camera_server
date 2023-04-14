/*
 *  ros_camera_server - Intelligent camera stream server.
 *  Copyright (C) 2026  Stefan Fabian
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */

#ifndef ROS_CAMERA_SERVER_PIPELINE_TEST_HELPERS_HPP
#define ROS_CAMERA_SERVER_PIPELINE_TEST_HELPERS_HPP

#include "ros_camera_server/configuration.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <vector>

namespace ros_camera_server
{

// -----------------------------------------------------------------------------
// Test Output Configuration for testing
// -----------------------------------------------------------------------------
class TestOutputConfiguration : public OutputConfiguration
{
public:
  TestOutputConfiguration() : OutputConfiguration( "test" ) { }

  YAML::Node toYaml() const override { return YAML::Node(); }

  PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &, const std::string &,
                                    int ) const override
  { return nullptr; }
};

inline std::shared_ptr<TestOutputConfiguration> createOutputConfig( StreamFormat format,
                                                                    int width = 0, int height = 0,
                                                                    Framerate framerate = Framerate() )
{
  auto config = std::make_shared<TestOutputConfiguration>();
  config->supported_input_formats = { format };
  config->width = width > 0 ? width : std::numeric_limits<int>::max();
  config->height = height > 0 ? height : std::numeric_limits<int>::max();
  config->framerate = framerate;

  if ( format == StreamFormat::H264 ) {
    config->codec = "h264";
    config->encoder = "auto";
    config->bitrate = 2000;
  } else if ( format == StreamFormat::H265 ) {
    config->codec = "h265";
    config->encoder = "auto";
    config->bitrate = 2000;
  }

  return config;
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_TEST_HELPERS_HPP
