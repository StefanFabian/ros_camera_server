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

#ifndef ROS_CAMERA_SERVER_ROS2_OUTPUT_HPP
#define ROS_CAMERA_SERVER_ROS2_OUTPUT_HPP

#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/outputs/pipeline_output.hpp"

namespace ros_camera_server
{
struct Ros2OutputConfiguration : public OutputConfiguration {
  static constexpr const char *TYPE = "ros2";
  std::string topic;
  std::string frame_id;
  std::string format; // ROS encoding format (e.g., "rgb8", "bgr8"), empty = no constraint
  std::string camera_info_url;

  Ros2OutputConfiguration();

  [[nodiscard]] PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &node,
                                                  const std::string &camera_id,
                                                  int index ) const override;

  YAML::Node toYaml() const override;

  static std::shared_ptr<Ros2OutputConfiguration> from_yaml_shared( const YAML::Node &config );
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_ROS2_OUTPUT_HPP
