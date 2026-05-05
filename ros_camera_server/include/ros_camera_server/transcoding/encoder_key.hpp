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

#ifndef ROS_CAMERA_SERVER_ENCODER_KEY_HPP
#define ROS_CAMERA_SERVER_ENCODER_KEY_HPP

#include "ros_camera_server/configuration.hpp"

#include <functional>
#include <string>

namespace ros_camera_server
{

/**
 * @brief Key for identifying unique encoder configurations.
 *
 * Two outputs with identical EncoderKey can share the same encoder instance.
 * The encoder field specifies the preferred encoder (e.g., "auto", "va", "nvenc|x264").
 * Outputs only share encoders if their encoder preference strings match exactly.
 */
struct EncoderKey {
  std::string codec;   // "h264", "h265"
  std::string encoder; // Encoder preference (e.g., "auto", "va", "nvenc|x264")
  int width;
  int height;
  Framerate framerate;
  int bitrate;

  bool operator==( const EncoderKey &other ) const
  {
    return codec == other.codec && encoder == other.encoder && width == other.width &&
           height == other.height && framerate == other.framerate && bitrate == other.bitrate;
  }

  bool operator!=( const EncoderKey &other ) const { return !( *this == other ); }
};

/**
 * @brief Key for identifying unique decoder configurations.
 *
 * Two pipelines requiring the same source format and decoder backend preference
 * can share a decoder node. The decoder field accepts the same syntax as the
 * encoder field (e.g. "auto", "va", "nv|sw").
 */
struct DecoderKey {
  StreamFormat format;
  std::string decoder; // Decoder preference (e.g., "auto", "va", "nv|sw")

  bool operator==( const DecoderKey &other ) const
  { return format == other.format && decoder == other.decoder; }

  bool operator!=( const DecoderKey &other ) const { return !( *this == other ); }
};

} // namespace ros_camera_server

// Hash specializations for use in unordered_map
template<>
struct std::hash<ros_camera_server::EncoderKey> {
  std::size_t operator()( const ros_camera_server::EncoderKey &key ) const
  {
    std::size_t h1 = std::hash<std::string>()( key.codec );
    std::size_t h2 = std::hash<std::string>()( key.encoder );
    std::size_t h3 = std::hash<int>()( key.width );
    std::size_t h4 = std::hash<int>()( key.height );
    std::size_t h5 = std::hash<int>()( key.framerate.numerator );
    std::size_t h6 = std::hash<int>()( key.framerate.denominator );
    std::size_t h7 = std::hash<int>()( key.bitrate );
    return h1 ^ ( h2 << 1 ) ^ ( h3 << 2 ) ^ ( h4 << 3 ) ^ ( h5 << 4 ) ^ ( h6 << 5 ) ^ ( h7 << 6 );
  }
};

template<>
struct std::hash<ros_camera_server::DecoderKey> {
  std::size_t operator()( const ros_camera_server::DecoderKey &key ) const
  {
    return std::hash<int>()( static_cast<int>( key.format ) ) ^
           ( std::hash<std::string>()( key.decoder ) << 1 );
  }
};

#endif // ROS_CAMERA_SERVER_ENCODER_KEY_HPP
