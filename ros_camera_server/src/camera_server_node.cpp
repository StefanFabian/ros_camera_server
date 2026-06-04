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
#include "camera_pipeline.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include <fstream>
#include <rclcpp/node.hpp>

namespace ros_camera_server
{

std::string parseYAML( const std::string &path )
{
  std::ifstream file( path );
  if ( !file.is_open() )
    throw std::runtime_error( "Could not open file." );
  std::string yaml( ( std::istreambuf_iterator<char>( file ) ), std::istreambuf_iterator<char>() );
  if ( !file )
    throw std::runtime_error( "Could not read file." );
  return yaml;
}

CameraServerNode::CameraServerNode( const rclcpp::NodeOptions &options )
    : rclcpp::Node( "camera_server", rclcpp::NodeOptions( options ).enable_logger_service( true ) )
{
  // Register the built-in input/output types. When loaded as a composable component our
  // main() never runs, so this must happen here. Both calls are idempotent.
  PipelineInputFactory::registerDefaultInputs();
  PipelineOutputFactory::registerDefaultOutputs();

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.read_only = true;
  declare_parameter<std::string>( "config_path", param_desc );
  declare_parameter<double>( "max_processing_time", 2.0 );
  std::string config_path = get_parameter( "config_path" ).as_string();
  std::string yaml_config = parseYAML( config_path );
  CameraServerConfiguration configuration = CameraServerConfiguration::from_yaml( yaml_config );

  announcement_publisher_ = create_publisher<Announcement>(
      "/camera_server_announcement", rclcpp::QoS( 1 ).transient_local().reliable().keep_last( 1 ) );

  camera_server_ = std::make_unique<CameraServer>(
      std::shared_ptr<rclcpp::Node>( this, []( const rclcpp::Node * ) { /* empty deleter */ } ),
      configuration, [this]() { publishAnnouncement(); } );
}

void CameraServerNode::publishAnnouncement()
{
  Announcement msg;
  std::string server_address = camera_server_->configuration().address;
  msg.robot = camera_server_->configuration().robot;
  msg.server_id = server_address;
  msg.host = server_address;
  msg.signaling_port = camera_server_->configuration().signaling_port;

  const auto &pipelines = camera_server_->pipelines();
  const auto &cameras = camera_server_->configuration().cameras;

  for ( size_t cam_idx = 0; cam_idx < pipelines.size() && cam_idx < cameras.size(); ++cam_idx ) {
    const auto &pipeline = pipelines[cam_idx];
    const auto &camera_config = cameras[cam_idx];

    ros_camera_server_msgs::msg::Camera camera_msg;
    camera_msg.id = camera_config.id;
    camera_msg.name = camera_config.name;

    const auto &outputs = pipeline->outputs();
    for ( const auto &output : outputs ) {
      if ( output ) {
        camera_msg.streams.push_back( output->toCameraStreamMsg( camera_server_->configuration() ) );
      }
    }

    msg.cameras.push_back( camera_msg );
  }

  announcement_publisher_->publish( msg );
}

} // namespace ros_camera_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( ros_camera_server::CameraServerNode )
