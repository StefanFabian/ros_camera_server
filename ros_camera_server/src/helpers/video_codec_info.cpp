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

#include "video_codec_info.hpp"

#include "ros_camera_server/helpers/reference_timestamp_helpers.hpp"
#include "ros_camera_server/helpers/rtp_timestamp_extension.hpp"

namespace ros_camera_server
{

VideoCodecInfo lookupVideoCodec( const std::string &codec )
{
  if ( codec == "h264" )
    return { StreamFormat::H264, "H264", "rtph264depay", "h264parse" };
  if ( codec == "h265" )
    return { StreamFormat::H265, "H265", "rtph265depay", "h265parse" };
  if ( codec == "jpeg" )
    return { StreamFormat::JPEG, "JPEG", "rtpjpegdepay", "jpegparse" };
  if ( codec == "png" )
    return { StreamFormat::PNG, nullptr, nullptr, "pngparse" };
  return {};
}

VideoCodecInfo lookupVideoCodecByFormat( StreamFormat format )
{
  switch ( format ) {
  case StreamFormat::H264:
    return lookupVideoCodec( "h264" );
  case StreamFormat::H265:
    return lookupVideoCodec( "h265" );
  case StreamFormat::JPEG:
    return lookupVideoCodec( "jpeg" );
  case StreamFormat::PNG:
    return lookupVideoCodec( "png" );
  default:
    return {};
  }
}

GstPadProbeReturn extractRtpTimestampProbe( GstPad *, GstPadProbeInfo *info, gpointer )
{
  if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER( info );
    GstClockTime ts = 0;
    if ( rtp_buffer_get_timestamp_extension( buf, ts ) ) {
      buf = gst_buffer_make_writable( buf );
      buffer_add_unix_timestamp_meta( buf, ts );
      GST_PAD_PROBE_INFO_DATA( info ) = buf;
    }
  }
  return GST_PAD_PROBE_OK;
}

} // namespace ros_camera_server
