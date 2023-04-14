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

#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include "../logging.hpp"
#include "ros_camera_server/outputs/ros2_output.hpp"
#include "ros_camera_server/outputs/srt_output.hpp"
#include "ros_camera_server/outputs/webrtc_output.hpp"

namespace ros_camera_server
{

void PipelineOutputFactory::registerDefaultOutputs()
{
  static bool registered = false;
  if ( registered )
    return;
  registered = true;

  registerOutput( "ros2", Ros2OutputConfiguration::from_yaml_shared );
  registerOutput( "srt", SrtOutputConfiguration::from_yaml_shared );
  registerOutput( "webrtc", WebrtcOutputConfiguration::from_yaml_shared );
}

void PipelineOutputFactory::registerOutput( const std::string &name,
                                            PipelineOutputFactory::OutputCreator creator )
{
  auto &reg = getRegistry();
  if ( reg.count( name ) > 0 )
    throw ros_camera_server::FactoryException( "Output type already registered: " + name );
  reg[name] = std::move( creator );
}

std::shared_ptr<OutputConfiguration> PipelineOutputFactory::createOutput( const std::string &name,
                                                                          const YAML::Node &config )
{
  auto &reg = getRegistry();
  auto it = reg.find( name );
  if ( it != reg.end() )
    return it->second( config );
  return nullptr;
}
} // namespace ros_camera_server
