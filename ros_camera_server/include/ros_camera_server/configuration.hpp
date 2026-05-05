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

#ifndef ROS_CAMERA_SERVER_CONFIGURATION_HPP
#define ROS_CAMERA_SERVER_CONFIGURATION_HPP

#include "ros_camera_server/helpers/smart_gst_pointer.hpp"
#include "ros_camera_server/outputs/pipeline_output.hpp"
#include "ros_camera_server/statistics.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ros_camera_server
{

struct EncoderKey;

class ConfigurationLoadError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct Framerate {
  int numerator = 0;
  int denominator = 0;

  Framerate();
  Framerate( int numerator, int denominator );
  explicit Framerate( const std::string &value );

  double toFps() const;

  unsigned long toNs() const;

  std::string toString() const;

  bool isValid() const;

  friend bool operator<( const Framerate &lhs, const Framerate &rhs )
  { return lhs.numerator * rhs.denominator < rhs.numerator * lhs.denominator; }

  friend bool operator>( const Framerate &lhs, const Framerate &rhs )
  { return lhs.numerator * rhs.denominator > rhs.numerator * lhs.denominator; }

  friend bool operator==( const Framerate &lhs, const Framerate &rhs )
  { return lhs.numerator * rhs.denominator == rhs.numerator * lhs.denominator; }

  friend bool operator!=( const Framerate &lhs, const Framerate &rhs ) { return !( lhs == rhs ); }

  friend bool operator<=( const Framerate &lhs, const Framerate &rhs )
  { return lhs < rhs || lhs == rhs; }
  friend bool operator>=( const Framerate &lhs, const Framerate &rhs )
  { return lhs > rhs || lhs == rhs; }
};

struct Size {
  int width = INT16_MAX;
  int height = INT16_MAX;

  Size() = default;
  Size( int width, int height ) : width( width ), height( height ) { }

  friend bool operator<( const Size &lhs, const Size &rhs )
  { return lhs.width < rhs.width || ( lhs.width == rhs.width && lhs.height < rhs.height ); }

  friend bool operator>( const Size &lhs, const Size &rhs )
  { return lhs.width > rhs.width || ( lhs.width == rhs.width && lhs.height > rhs.height ); }

  friend bool operator==( const Size &lhs, const Size &rhs )
  { return lhs.width == rhs.width && lhs.height == rhs.height; }

  friend bool operator!=( const Size &lhs, const Size &rhs ) { return !( lhs == rhs ); }

  friend bool operator<=( const Size &lhs, const Size &rhs ) { return lhs < rhs || lhs == rhs; }

  friend bool operator>=( const Size &lhs, const Size &rhs ) { return lhs > rhs || lhs == rhs; }
};

enum class StreamFormat { INVALID, RAW, JPEG, PNG, H264, H265 };

inline std::string to_string( StreamFormat format )
{
  switch ( format ) {
  case StreamFormat::RAW:
    return "raw";
  case StreamFormat::JPEG:
    return "jpeg";
  case StreamFormat::PNG:
    return "png";
  case StreamFormat::H264:
    return "h264";
  case StreamFormat::H265:
    return "h265";
  default:
    return "invalid";
  }
}

inline StreamFormat stream_format_from_codec( std::string codec )
{
  std::transform( codec.begin(), codec.end(), codec.begin(),
                  []( unsigned char c ) { return std::tolower( c ); } );
  if ( codec == "h264" )
    return StreamFormat::H264;
  if ( codec == "h265" )
    return StreamFormat::H265;
  if ( codec == "jpeg" )
    return StreamFormat::JPEG;
  if ( codec == "png" )
    return StreamFormat::PNG;
  if ( codec == "raw" )
    return StreamFormat::RAW;
  return StreamFormat::INVALID;
}

struct StreamInput {
  GstBin *bin = nullptr;
  StreamFormat format;
  std::shared_ptr<void> state; // Opaque handle for input-specific runtime state (e.g. parameter callbacks)
};

class InputConfiguration
{
public:
  virtual ~InputConfiguration() = default;

  std::string type;
  Framerate framerate;
  /// Decoder backend preference (e.g. "auto", "va|sw"). Used when the input
  /// produces an encoded format and the pipeline requires raw video.
  std::string decoder = "auto";

  /// Load fields shared by every input (framerate, decoder) from yaml.
  /// Derived from_yaml_shared methods call this so a new shared field only
  /// needs to be added in one place.
  void loadSharedFromYaml( const YAML::Node &config );

  [[nodiscard]] virtual StreamInput createInput( const rclcpp::Node::SharedPtr &,
                                                 const std::string &camera_id ) const = 0;
};

class OutputConfiguration
{
public:
  OutputConfiguration( std::string type ) : type( std::move( type ) ) { }
  virtual ~OutputConfiguration() = default;

  std::string type;
  std::string codec;
  std::string encoder = "auto";
  int bitrate = 0;
  Framerate framerate;
  int width = std::numeric_limits<int>::max();
  int height = std::numeric_limits<int>::max();
  std::vector<StreamFormat> supported_input_formats;
  StreamFormat selected_input_format = StreamFormat::INVALID;

  /// Load fields shared by every output (encoder, framerate, width, height,
  /// bitrate) from yaml. Derived from_yaml_shared methods call this so a new
  /// shared field only needs to be added in one place. `codec` is excluded
  /// because defaults and validation are output-specific.
  void loadSharedFromYaml( const YAML::Node &config );

  virtual YAML::Node toYaml() const = 0;

  [[nodiscard]] virtual PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &,
                                                          const std::string &camera_id,
                                                          int index ) const = 0;

  /// Create an EncoderKey for this output configuration
  virtual EncoderKey createEncoderKey() const;

  /// The actual format resolved by the pipeline graph.
  /// Called after the pipeline graph is built to replace "auto" or other placeholder values.
  virtual void onStreamFormatSelected( StreamFormat format ) { selected_input_format = format; }

  /// Check if this output needs a transform (scaling or framerate change)
  virtual bool needsTransform() const
  {
    return ( width != std::numeric_limits<int>::max() && width > 0 ) ||
           ( height != std::numeric_limits<int>::max() && height > 0 ) || framerate.isValid();
  }
};

struct CameraConfiguration {
  std::string id;
  std::string name;
  std::shared_ptr<InputConfiguration> input;
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
};

struct CameraServerConfiguration {
  std::string robot;
  std::string server_id;
  unsigned short clock_port;
  int signaling_port = 8443;
  std::string address;
  std::vector<CameraConfiguration> cameras;

  static CameraServerConfiguration from_yaml( const std::string &value );
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_CONFIGURATION_HPP
