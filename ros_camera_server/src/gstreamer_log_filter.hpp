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

#ifndef ROS_CAMERA_SERVER_GSTREAMER_LOG_FILTER_HPP
#define ROS_CAMERA_SERVER_GSTREAMER_LOG_FILTER_HPP

#include <array>
#include <cstdlib>
#include <regex>
#include <string_view>

namespace ros_camera_server
{
namespace detail
{
enum class LogMatchMode {
  Exact,
  Contains,
  Regex,
};
struct IgnoredGStreamerLog {
  std::string_view category;
  std::string_view message;
  LogMatchMode mode;
  const std::regex *regex = nullptr; // Optional regex to match the message, for more complex cases.
};

inline const std::regex GST_LOG_REGEX_WEBRTC_RESOLVE_FAIL =
    std::regex( "(failed to resolve|Could not resolve candidate address): Error resolving .+: "
                "Name or service not known",
                std::regex::optimize );

inline constexpr std::array<IgnoredGStreamerLog, 9> ignored_gstreamer_logs = { {
    { "vadisplay", "vaInitialize: unknown libva error", LogMatchMode::Exact },
    { "vapostproc", "Can't keep DAR!", LogMatchMode::Exact },
    { "v4l2src", "Can't give latency since framerate isn't fixated !", LogMatchMode::Exact },
    { "v4l2", "VIDIOC_S_CROP failed", LogMatchMode::Exact },
    { "v4l2", "Failed to get default compose rectangle with VIDIOC_G_SELECTION: Invalid argument",
      LogMatchMode::Exact },
    { "v4l2bufferpool", "Uncertain or not enough buffers, enabling copy threshold",
      LogMatchMode::Exact },
    { "GST_PADS", "could not send sticky events", LogMatchMode::Exact },
    { "rtpsession", "Can't determine running time for this packet without knowing configured latency",
      LogMatchMode::Exact },
    { "webrtcnice", "", LogMatchMode::Regex, &GST_LOG_REGEX_WEBRTC_RESOLVE_FAIL },
} };

inline bool shouldIgnoreGStreamerLog( std::string_view category, std::string_view message )
{
  for ( const auto &ignored_log : ignored_gstreamer_logs ) {
    if ( ignored_log.category != category )
      continue;

    switch ( ignored_log.mode ) {
    case LogMatchMode::Exact:
      if ( ignored_log.message == message ) {
        return true;
      }
      break;
    case LogMatchMode::Contains:
      if ( message.find( ignored_log.message ) != std::string_view::npos ) {
        return true;
      }
      break;
    case LogMatchMode::Regex:
      if ( std::regex_match( std::string( message ), *ignored_log.regex ) ) {
        return true;
      }
      break;
    }
  }
  return false;
}

inline bool hasPrefix( std::string_view message, std::string_view prefix )
{ return message.size() >= prefix.size() && message.compare( 0, prefix.size(), prefix ) == 0; }

} // namespace detail
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_GSTREAMER_LOG_FILTER_HPP
