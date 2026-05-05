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

#ifndef ROS_CAMERA_SERVER_RTP_INPUT_HPP
#define ROS_CAMERA_SERVER_RTP_INPUT_HPP

#include "ros_camera_server/configuration.hpp"

namespace ros_camera_server
{

class RtpInputConfiguration : public InputConfiguration
{
public:
  std::string codec; // h264 | h265 | jpeg
  int port = 5004;
  std::string address = "0.0.0.0";
  bool multicast = false;
  int payload_type = 96;
  int latency_ms = 200;
  bool drop_on_latency = false;

  [[nodiscard]] StreamInput createInput( const rclcpp::Node::SharedPtr &,
                                         const std::string &camera_id ) const override;

  static std::shared_ptr<RtpInputConfiguration> from_yaml_shared( const YAML::Node &config );
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_RTP_INPUT_HPP
