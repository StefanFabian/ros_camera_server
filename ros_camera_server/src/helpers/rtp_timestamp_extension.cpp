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

#include "ros_camera_server/helpers/rtp_timestamp_extension.hpp"
#include "./endian.hpp"
#include <gst/rtp/gstrtpbuffer.h>

namespace ros_camera_server
{
/**
 * Add timestamp extension to RTP buffer
 *
 * This function adds a RFC 5285 one-byte header extension containing
 * a 64-bit timestamp. The buffer must be writable.
 */
gboolean rtp_buffer_add_timestamp_extension( GstBuffer *buffer, GstClockTime timestamp )
{
  GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
  gboolean success = FALSE;

  // Validate input
  if ( !buffer || !GST_IS_BUFFER( buffer ) ) {
    g_warning( "Invalid buffer provided to rtp_buffer_add_timestamp_extension" );
    return FALSE;
  }

  if ( !GST_CLOCK_TIME_IS_VALID( timestamp ) ) {
    g_warning( "Invalid timestamp provided" );
    return FALSE;
  }

  if ( !gst_rtp_buffer_map( buffer, GST_MAP_READWRITE, &rtp_buffer ) ) {
    g_warning( "Failed to map RTP buffer for writing" );
    return FALSE;
  }

  guint64 timestamp_network = hosttole64( timestamp );
  success = gst_rtp_buffer_add_extension_onebyte_header( &rtp_buffer, RTP_EXTENSION_ID_TIMESTAMP,
                                                         (gconstpointer)&timestamp_network,
                                                         sizeof( guint64 ) );

  if ( !success ) {
    g_warning( "Failed to add RTP header extension. "
               "Possible reasons: incompatible existing extension, "
               "or buffer size limit exceeded" );
  }

  gst_rtp_buffer_unmap( &rtp_buffer );
  return success;
}

/**
 * Extract timestamp extension from RTP buffer
 *
 * This function retrieves the timestamp from a RFC 5285 one-byte
 * header extension with the specified ID.
 */
gboolean rtp_buffer_get_timestamp_extension( GstBuffer *buffer, GstClockTime &timestamp )
{
  GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
  gboolean success = FALSE;

  if ( !buffer || !GST_IS_BUFFER( buffer ) ) {
    g_warning( "Invalid parameters provided to rtp_buffer_get_timestamp_extension" );
    return FALSE;
  }

  if ( !gst_rtp_buffer_map( buffer, GST_MAP_READ, &rtp_buffer ) ) {
    g_warning( "Failed to map RTP buffer for reading" );
    return FALSE;
  }

  if ( !gst_rtp_buffer_get_extension( &rtp_buffer ) ) {
    // No extension header present - this is normal for some packets
    gst_rtp_buffer_unmap( &rtp_buffer );
    return FALSE;
  }

  gpointer extension_data = nullptr;
  guint extension_size = 0;
  success = gst_rtp_buffer_get_extension_onebyte_header( &rtp_buffer, RTP_EXTENSION_ID_TIMESTAMP, 0,
                                                         &extension_data, &extension_size );

  if ( success ) {
    if ( extension_size != sizeof( guint64 ) ) {
      g_warning( "Timestamp extension has unexpected size: %u bytes (expected %zu)", extension_size,
                 sizeof( guint64 ) );
      success = FALSE;
    } else {
      guint64 timestamp_network;
      memcpy( &timestamp_network, extension_data, sizeof( guint64 ) );
      timestamp = le64tohost( timestamp_network );
    }
  }

  gst_rtp_buffer_unmap( &rtp_buffer );
  return success;
}

} // namespace ros_camera_server
