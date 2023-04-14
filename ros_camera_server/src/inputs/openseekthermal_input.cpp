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

#include "ros_camera_server/inputs/openseekthermal_input.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

namespace ros_camera_server
{

std::shared_ptr<OpenSeekThermalInputConfiguration>
OpenSeekThermalInputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<OpenSeekThermalInputConfiguration>();
  result->type = "openseekthermal";
  result->port = config["device"].as<std::string>( "" );
  result->serial = config["serial"].as<std::string>( "" );
  return result;
}

StreamInput OpenSeekThermalInputConfiguration::createInput( const rclcpp::Node::SharedPtr &,
                                                            const std::string & /*camera_id*/ ) const
{
  auto *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  GstElement *src = gst_element_factory_make( "openseekthermalsrc", "input" );
  g_object_set( G_OBJECT( src ), "serial", serial.c_str(), "port", port.c_str(), "do-timestamp",
                TRUE, nullptr );

  gst_bin_add( input_bin, src );
  GstPad *input_pad = gst_element_get_static_pad( src, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", input_pad ) );
  gst_object_unref( input_pad );
  return { input_bin, StreamFormat::RAW };
}
} // namespace ros_camera_server
