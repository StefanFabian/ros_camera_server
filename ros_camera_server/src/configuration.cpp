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

#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/transcoding/encoder_key.hpp"

#include "logging.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"
#include "ros_camera_server/factories/pipeline_output_factory.hpp"
#include "ros_camera_server/helpers/smart_gst_pointer.hpp"

#include <regex>

namespace ros_camera_server
{
Framerate::Framerate() : numerator( 0 ), denominator( 0 ) { }

Framerate::Framerate( int numerator, int denominator )
    : numerator( numerator ), denominator( denominator )
{
}

Framerate::Framerate( const std::string &value )
{
  if ( value.empty() ) {
    numerator = 0;
    denominator = 0;
    return;
  }
  static std::regex regex( R"((\d+)/(\d+)?)" );
  std::smatch match;
  if ( !std::regex_match( value, match, regex ) ) {
    throw std::runtime_error( "Invalid framerate format: " + value );
  }
  numerator = std::stoi( match[1] );
  denominator = match[2].matched ? std::stoi( match[2] ) : 1;
}

double Framerate::toFps() const { return denominator == 0 ? 0 : double( numerator ) / denominator; }

unsigned long Framerate::toNs() const
{
  // This is inverted since the framerate is in fps, so the time to wait between images is 1 / framerate
  return numerator == 0 ? 0 : denominator * (unsigned long)( 1E9 ) / numerator;
}

std::string Framerate::toString() const
{ return std::to_string( numerator ) + "/" + std::to_string( denominator ); }

bool Framerate::isValid() const { return numerator > 0 && denominator > 0; }

EncoderKey OutputConfiguration::createEncoderKey() const
{
  EncoderKey key;
  key.codec = codec;
  key.encoder = encoder;
  key.width = width;
  key.height = height;
  key.framerate = framerate;
  key.bitrate = bitrate;
  return key;
}

namespace
{
std::shared_ptr<InputConfiguration> load_input_configuration( const YAML::Node &config )
{
  auto type = config["type"].as<std::string>();
  auto input = PipelineInputFactory::createInput( type, config );
  if ( input ) {
    return input;
  }
  throw ConfigurationLoadError( "Unknown input type: " + config["type"].as<std::string>() );
}

std::shared_ptr<OutputConfiguration> load_output_configuration( const YAML::Node &config )
{
  auto type = config["type"].as<std::string>();
  auto output = PipelineOutputFactory::createOutput( type, config );
  if ( output ) {
    return output;
  }
  throw ConfigurationLoadError( "Unknown output type: " + config["type"].as<std::string>() );
}
} // namespace

CameraServerConfiguration CameraServerConfiguration::from_yaml( const std::string &value )
{
  CameraServerConfiguration result;
  YAML::Node config = YAML::Load( value );
  result.robot = config["robot"].as<std::string>( "robot" );
  result.server_id = config["server_id"].as<std::string>( "main" );
  result.clock_port = config["clock_port"].as<unsigned short>( 8554 );
  result.signaling_port = config["signaling_port"].as<int>( 8443 );
  result.address = config["address"].as<std::string>( "localhost" );
  if ( config["cameras"].IsNull() )
    return result;
  for ( const auto &kvp : config["cameras"] ) {
    CameraConfiguration camera;
    camera.id = kvp.first.as<std::string>();
    const YAML::Node &entry = kvp.second;
    camera.name = entry["name"].as<std::string>( camera.id );
    camera.input = load_input_configuration( entry["input"] );
    for ( const auto &output : entry["outputs"] ) {
      camera.outputs.push_back( load_output_configuration( output ) );
    }
    result.cameras.push_back( camera );
  }
  return result;
}
} // namespace ros_camera_server
