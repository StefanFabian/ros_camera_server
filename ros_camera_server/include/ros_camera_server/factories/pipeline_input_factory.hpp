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

#ifndef ROS_CAMERA_SERVER_PIPELINE_INPUT_FACTORY_HPP
#define ROS_CAMERA_SERVER_PIPELINE_INPUT_FACTORY_HPP

#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/exceptions.hpp"

#include <memory>
#include <rclcpp/node.hpp>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace ros_camera_server
{

class PipelineInputFactory
{
public:
  using InputCreator = std::function<std::shared_ptr<InputConfiguration>( const YAML::Node & )>;

  static void registerDefaultInputs();

  /*!
   * @brief Registers a new input type with the factory.
   * @param name The name of the input type. (e.g., "v4l2", "ros2", etc.)
   * @param creator The function to create the input.
   * @throws ros_camera_server::FactoryException if an input type with the same name is already registered.
   */
  static void registerInput( const std::string &name, InputCreator creator );

  //! Creates an input by name. Returns nullptr if not found.
  static std::shared_ptr<InputConfiguration> createInput( const std::string &name,
                                                          const YAML::Node &config );

  //! Returns true if an input type is registered.
  static bool isRegistered( const std::string &name ) { return getRegistry().count( name ) > 0; }

private:
  static std::unordered_map<std::string, InputCreator> &getRegistry();
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_INPUT_FACTORY_HPP
