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

#ifndef ROS_CAMERA_SERVER_OPENSEEKTHERMAL_INPUT_HPP
#define ROS_CAMERA_SERVER_OPENSEEKTHERMAL_INPUT_HPP

#include "ros_camera_server/configuration.hpp"

namespace ros_camera_server
{

class OpenSeekThermalInputConfiguration : public InputConfiguration
{
public:
  //! Serial of the device.
  std::string serial;
  //! The port is used if no serial provided. If neither is provided, first device is used.
  std::string port;
  [[nodiscard]] StreamInput createInput( const rclcpp::Node::SharedPtr &,
                                         const std::string &camera_id ) const override;
  static std::shared_ptr<OpenSeekThermalInputConfiguration>
  from_yaml_shared( const YAML::Node &config );
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_OPENSEEKTHERMAL_INPUT_HPP
