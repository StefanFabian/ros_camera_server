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

#include "ros_camera_server/camera_server_node.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  ros_camera_server::PipelineInputFactory::registerDefaultInputs();
  ros_camera_server::PipelineOutputFactory::registerDefaultOutputs();
  auto node = std::make_shared<ros_camera_server::CameraServerNode>( "camera_server" );
  rclcpp::spin( node );
  node.reset();
  rclcpp::shutdown();
  return 0;
}
