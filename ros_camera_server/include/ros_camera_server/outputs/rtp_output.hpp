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

#ifndef ROS_CAMERA_SERVER_RTP_OUTPUT_HPP
#define ROS_CAMERA_SERVER_RTP_OUTPUT_HPP

#include "ros_camera_server/configuration.hpp"

namespace ros_camera_server
{

struct RtpOutputConfiguration : public OutputConfiguration {
  static constexpr const char *TYPE = "rtp";

  std::string host = "127.0.0.1";
  int port = 5004;
  bool multicast = false;
  int ttl = 16;
  int payload_type = 96;
  bool embed_capture_timestamp = true;

  RtpOutputConfiguration();

  PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &ptr,
                                    const std::string &camera_id, int index ) const override;

  YAML::Node toYaml() const override;
  static std::shared_ptr<RtpOutputConfiguration> from_yaml_shared( const YAML::Node &config );
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_RTP_OUTPUT_HPP
