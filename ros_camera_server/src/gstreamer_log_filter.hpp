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

inline constexpr std::array<IgnoredGStreamerLog, 10> ignored_gstreamer_logs = { {
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
    { "webrtcbin", "to merge ICE candidate", LogMatchMode::Contains },
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

// Each rule rewrites a cryptic GStreamer error/warning into a friendlier explanation. The matcher
// receives the raw bus/log message text and the GStreamer-supplied debug_info string (file, line,
// function, element path) so it can disambiguate generic messages by their originating function.
struct GStreamerMessageRewrite {
  bool ( *matches )( std::string_view message, std::string_view debug_info );
  std::string_view replacement;
};

inline constexpr std::array<GStreamerMessageRewrite, 1> gstreamer_message_rewrites = { {
    // rtpjpegpay rejects JPEGs whose chroma subsampling isn't YUV 4:2:0 / 4:2:2 (RFC 2435).
    // The error text is just "Invalid component"; the originating function in debug_info is
    // what tells us this is the rtpjpegpay SOF parser.
    { []( std::string_view message, std::string_view debug_info ) {
       return message == "Invalid component" &&
              debug_info.find( "gst_rtp_jpeg_pay_read_sof" ) != std::string_view::npos;
     },
      "rtpjpegpay rejected the JPEG: RFC 2435 only supports YUV 4:2:0 or 4:2:2 chroma "
      "subsampling. Re-encode the source as a baseline JPEG with I420 or Y42B "
      "(e.g. videoconvert ! jpegenc) before payloading." },
} };

// Returns a friendlier replacement for the message, or an empty view if no rule matches.
inline std::string_view rewriteGStreamerMessage( std::string_view message,
                                                 std::string_view debug_info )
{
  for ( const auto &rewrite : gstreamer_message_rewrites ) {
    if ( rewrite.matches( message, debug_info ) )
      return rewrite.replacement;
  }
  return {};
}

} // namespace detail
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_GSTREAMER_LOG_FILTER_HPP
