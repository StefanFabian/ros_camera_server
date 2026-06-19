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
#include "ros_camera_server/exceptions.hpp"
#include "ros_camera_server/factories/pipeline_input_factory.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros_camera_server
{

namespace
{
// Resolves package://<pkg>/<rel> to an absolute filesystem path. file:// is stripped.
// Other inputs are returned unchanged.
std::string resolvePackageUri( const std::string &uri )
{
  constexpr std::string_view kPackagePrefix = "package://";
  constexpr std::string_view kFilePrefix = "file://";
  if ( uri.rfind( kPackagePrefix, 0 ) != 0 ) {
    if ( uri.rfind( kFilePrefix, 0 ) == 0 )
      return uri.substr( kFilePrefix.size() );
    return uri;
  }
  const std::string remainder = uri.substr( kPackagePrefix.size() );
  const auto slash = remainder.find( '/' );
  if ( slash == std::string::npos || slash == 0 ) {
    throw PipelineBuildError( "Malformed package:// URI '" + uri +
                              "', expected package://<package>/<relative_path>" );
  }
  const std::string package = remainder.substr( 0, slash );
  const std::string relative = remainder.substr( slash + 1 );
  try {
    return ament_index_cpp::get_package_share_directory( package ) + "/" + relative;
  } catch ( const ament_index_cpp::PackageNotFoundError &e ) {
    throw PipelineBuildError( "Could not resolve package:// URI '" + uri + "': " + e.what() );
  }
}
} // namespace

std::shared_ptr<OpenSeekThermalInputConfiguration>
OpenSeekThermalInputConfiguration::from_yaml_shared( const YAML::Node &config )
{
  auto result = std::make_shared<OpenSeekThermalInputConfiguration>();
  result->type = "openseekthermal";
  result->loadSharedFromYaml( config );
  result->port = config["device"].as<std::string>( "" );
  result->serial = config["serial"].as<std::string>( "" );
  result->skip_invalid_frames = config["skip_invalid_frames"].as<bool>( true );
  result->normalize = config["normalize"].as<bool>( false );
  result->normalize_frame_count = config["normalize_frame_count"].as<unsigned int>( 8 );
  result->calibration = config["calibration"].as<std::string>( "" );
  return result;
}

StreamInput OpenSeekThermalInputConfiguration::createInput( const rclcpp::Node::SharedPtr &,
                                                            const std::string & /*camera_id*/ ) const
{
  auto *input_bin = GST_BIN( gst_bin_new( "input_bin" ) );
  GstElement *src = gst_element_factory_make( "openseekthermalsrc", "input" );
  const std::string calibration_path = resolvePackageUri( calibration );
  g_object_set( G_OBJECT( src ), "serial", serial.c_str(), "port", port.c_str(),
                "skip-invalid-frames", skip_invalid_frames ? TRUE : FALSE, "normalize",
                normalize ? TRUE : FALSE, "normalize-frame-count",
                static_cast<guint>( normalize_frame_count ), "calibration",
                calibration_path.c_str(), "do-timestamp", TRUE, nullptr );

  gst_bin_add( input_bin, src );
  GstPad *input_pad = gst_element_get_static_pad( src, "src" );
  gst_element_add_pad( GST_ELEMENT( input_bin ), gst_ghost_pad_new( "src", input_pad ) );
  gst_object_unref( input_pad );
  return { input_bin, StreamFormat::RAW };
}
} // namespace ros_camera_server
