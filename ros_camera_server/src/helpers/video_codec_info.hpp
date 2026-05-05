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

#ifndef ROS_CAMERA_SERVER_VIDEO_CODEC_INFO_HPP
#define ROS_CAMERA_SERVER_VIDEO_CODEC_INFO_HPP

#include "ros_camera_server/configuration.hpp"

#include <gst/gst.h>
#include <string>

namespace ros_camera_server
{

struct VideoCodecInfo {
  StreamFormat format = StreamFormat::INVALID;
  const char *rtp_encoding_name = nullptr; // upper-case value for application/x-rtp encoding-name
  const char *rtp_depayloader = nullptr;   // factory name (rtph264depay, …)
  const char *parser = nullptr;            // codec parser factory, may be null
};

// Looks up codec metadata for a config codec string ("h264" | "h265" | "jpeg" | "png").
// Returns a value whose `format` is StreamFormat::INVALID if the codec is unknown.
// `parser` is the GStreamer parser factory needed to make caps complete on a raw stream
// of this format (e.g. add width/height for JPEG, alignment/SPS-PPS handling for H.264).
// `rtp_encoding_name` and `rtp_depayloader` are nullptr for formats with no RTP transport.
VideoCodecInfo lookupVideoCodec( const std::string &codec );

// Same as above but keyed by StreamFormat. Returns an INVALID-format value for
// formats without codec metadata (RAW, INVALID).
VideoCodecInfo lookupVideoCodecByFormat( StreamFormat format );

// Pad probe that extracts the RTP-header timestamp extension from incoming buffers
// and attaches it as a GstReferenceTimestampMeta (UNIX clock). Install on the source
// pad of the upstream RTP element (jitterbuffer / rtspsrc) before the depayloader.
GstPadProbeReturn extractRtpTimestampProbe( GstPad *pad, GstPadProbeInfo *info, gpointer user_data );

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_VIDEO_CODEC_INFO_HPP
