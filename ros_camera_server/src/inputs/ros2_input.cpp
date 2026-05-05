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

#include "ros_camera_server/inputs/ros2_input.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

namespace ros_camera_server
{

Ros2InputFormat format_from_string( const std::string &format )
{
  if ( format == "raw" ) {
    return Ros2InputFormat::RAW;
  } else if ( format == "jpeg" ) {
    return Ros2InputFormat::JPEG;
  } else if ( format == "png" ) {
    return Ros2InputFormat::PNG;
  } else {
    throw std::invalid_argument( "Unsupported format: " + format );
  }
}

std::shared_ptr<Ros2InputConfiguration>
Ros2InputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<Ros2InputConfiguration>();
  result->type = "ros2";
  result->loadSharedFromYaml( config );
  result->topic = config["topic"].as<std::string>();
  result->format = format_from_string( config["format"].as<std::string>( "raw" ) );
  return result;
}

StreamInput Ros2InputConfiguration::createInput( const rclcpp::Node::SharedPtr &node,
                                                 const std::string & /*camera_id*/ ) const
{
  auto *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  GstElement *src = gst_element_factory_make( "rbfimagesrc", "input" );
  // image_transport convention: compressed encodings are published on
  // `<topic>/compressed`. rbfimagesrc subscribes to the exact topic name, so
  // when the user requests a compressed format we resolve the suffix here
  std::string subscribe_topic = topic;
  if ( format == Ros2InputFormat::JPEG || format == Ros2InputFormat::PNG ) {
    subscribe_topic += "/compressed";
  }
  g_object_set( G_OBJECT( src ), "node", (gpointer)node.get(), "topic", subscribe_topic.c_str(),
                "determine-framerate", TRUE, nullptr );
  if ( framerate.isValid() ) {
    g_object_set( G_OBJECT( src ), "framerate", framerate.toString().c_str(), nullptr );
  }

  gst_bin_add( input_bin, src );

  GstPad *input_pad = gst_element_get_static_pad( src, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", input_pad ) );
  gst_object_unref( input_pad );
  StreamFormat stream_format = StreamFormat::INVALID;
  switch ( format ) {
  case Ros2InputFormat::RAW:
    stream_format = StreamFormat::RAW;
    break;
  case Ros2InputFormat::JPEG:
    stream_format = StreamFormat::JPEG;
    break;
  case Ros2InputFormat::PNG:
    stream_format = StreamFormat::PNG;
    break;
  }
  return { input_bin, stream_format };
}
} // namespace ros_camera_server
