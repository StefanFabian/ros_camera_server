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

#ifndef ROS_CAMERA_SERVER_RTP_TIMESTAMP_EXTENSION_HPP
#define ROS_CAMERA_SERVER_RTP_TIMESTAMP_EXTENSION_HPP

#include <glib.h>
#include <gst/gstbuffer.h>
#include <gst/gstclock.h>

namespace ros_camera_server
{

// Extension ID for our the timestamp
// RTP uses 4 bit for id and 4 bit for length with 0 and 15 reserved. 1 was picked arbitrarily.
constexpr int RTP_EXTENSION_ID_TIMESTAMP = 1;

// Structure to hold timestamp data
typedef struct {
  guint64 capture_timestamp; // Nanoseconds since epoch
} TimestampExtensionData;

/**
 * Add timestamp extension to RTP buffer
 *
 * @param buffer: GstBuffer containing RTP packet (must be writable)
 * @param timestamp: Capture timestamp in nanoseconds
 * @return TRUE if successfully added, FALSE otherwise
 */
gboolean rtp_buffer_add_timestamp_extension( GstBuffer *buffer, GstClockTime timestamp );

/**
 * Extract timestamp extension from RTP buffer
 *
 * @param buffer: GstBuffer containing RTP packet
 * @param timestamp: Output parameter for extracted timestamp
 * @return TRUE if extension found and extracted, FALSE otherwise
 */
gboolean rtp_buffer_get_timestamp_extension( GstBuffer *buffer, GstClockTime &timestamp );

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_RTP_TIMESTAMP_EXTENSION_HPP
