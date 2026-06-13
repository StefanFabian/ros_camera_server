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

#include "ros_camera_server/factories/pipeline_input_factory.hpp"
#include "../logging.hpp"
#include "ros_camera_server/inputs/openseekthermal_input.hpp"
#include "ros_camera_server/inputs/ros2_input.hpp"
#include "ros_camera_server/inputs/rtp_input.hpp"
#include "ros_camera_server/inputs/v4l2_input.hpp"
#include "ros_camera_server/inputs/videotestsrc_input.hpp"
#include <mutex>

namespace ros_camera_server
{

void PipelineInputFactory::registerDefaultInputs()
{
  static std::once_flag flag;
  std::call_once( flag, []() {
    registerInput( "openseekthermal", OpenSeekThermalInputConfiguration::from_yaml_shared );
    registerInput( "ros2", Ros2InputConfiguration::from_yaml_shared );
    registerInput( "rtp", RtpInputConfiguration::from_yaml_shared );
    registerInput( "v4l2", V4l2InputConfiguration::from_yaml_shared );
    registerInput( "videotestsrc", VideoTestSrcInputConfiguration::from_yaml_shared );
  } );
}

void PipelineInputFactory::registerInput( const std::string &name,
                                          PipelineInputFactory::InputCreator creator )
{
  auto &reg = getRegistry();
  if ( reg.count( name ) > 0 )
    throw ros_camera_server::FactoryException( "Input type already registered: " + name );
  reg[name] = std::move( creator );
  SERVER_LOG_INFO( "Registered input type: %s", name.c_str() );
}

std::shared_ptr<InputConfiguration> PipelineInputFactory::createInput( const std::string &name,
                                                                       const YAML::Node &config )
{
  auto &reg = getRegistry();
  auto it = reg.find( name );
  if ( it != reg.end() )
    return it->second( config );
  std::stringstream types;
  for ( const auto &[type, _] : reg ) { types << type << " "; }
  SERVER_LOG_ERROR( "No input type registered with name: %s. Available input types: %s",
                    name.c_str(), types.str().c_str() );
  return nullptr;
}
std::unordered_map<std::string, PipelineInputFactory::InputCreator> &PipelineInputFactory::getRegistry()
{
  static std::unordered_map<std::string, InputCreator> registry;
  return registry;
}
} // namespace ros_camera_server
