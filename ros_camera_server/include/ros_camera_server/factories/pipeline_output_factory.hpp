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

#ifndef ROS_CAMERA_SERVER_PIPELINE_OUTPUT_FACTORY_HPP
#define ROS_CAMERA_SERVER_PIPELINE_OUTPUT_FACTORY_HPP

#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/exceptions.hpp"

#include <memory>
#include <rclcpp/node.hpp>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace ros_camera_server
{

class PipelineOutputFactory
{
public:
  using OutputCreator = std::function<std::shared_ptr<OutputConfiguration>( const YAML::Node & )>;

  static void registerDefaultOutputs();

  /*!
   * @brief Registers a new output type with the factory.
   * @param name The name of the output type. (e.g., "srt", "rtmp", etc.)
   * @param creator The function to create the output.
   * @throws ros_camera_server::FactoryException if an output type with the same name is already registered.
   */
  static void registerOutput( const std::string &name, OutputCreator creator );

  //! Creates an output by name. Returns nullptr if not found.
  static std::shared_ptr<OutputConfiguration> createOutput( const std::string &name,
                                                            const YAML::Node &config );

  //! Returns true if an output type is registered.
  static bool isRegistered( const std::string &name ) { return getRegistry().count( name ) > 0; }

private:
  static std::unordered_map<std::string, OutputCreator> &getRegistry()
  {
    static std::unordered_map<std::string, OutputCreator> registry;
    return registry;
  }
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_OUTPUT_FACTORY_HPP
