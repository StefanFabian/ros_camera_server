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

#ifndef ROS_CAMERA_SERVER_REFERENCE_TIMESTAMP_HELPERS_HPP
#define ROS_CAMERA_SERVER_REFERENCE_TIMESTAMP_HELPERS_HPP

#include <gst/gst.h>

#include <atomic>
#include <memory>

namespace ros_camera_server
{

/**
 * Shared encoder timing statistics updated by encoder pad probes.
 * Read by pipeline outputs to provide processing time breakdown.
 */
struct EncoderTimingStats {
  std::atomic<int64_t> avg_pre_encoder_us{ -1 }; // ingress → encoder sink
  std::atomic<int64_t> avg_encode_time_us{ -1 }; // encoder sink → encoder src
};

/**
 * Get singleton caps for "timestamp/x-unix" reference.
 * The returned caps are owned by the library and should not be unref'd.
 */
GstCaps *unix_timestamp_reference_caps();

/**
 * Add UNIX timestamp metadata to a buffer
 *
 * @param buffer GstBuffer to add meta to (must be writable)
 * @param timestamp Timestamp in nanoseconds since UNIX epoch
 * @return Pointer to the added meta, or nullptr on failure
 */
GstReferenceTimestampMeta *buffer_add_unix_timestamp_meta( GstBuffer *buffer, GstClockTime timestamp );

/**
 * Get UNIX timestamp metadata from a buffer
 *
 * @param buffer GstBuffer to read meta from
 * @return Pointer to the meta, or nullptr if not present
 */
GstReferenceTimestampMeta *buffer_get_unix_timestamp_meta( GstBuffer *buffer );

/**
 * Get singleton caps for "timestamp/x-server-ingress" reference.
 * The returned caps are owned by the library and should not be unref'd.
 */
GstCaps *server_ingress_reference_caps();

/**
 * Add server ingress timestamp metadata to a buffer.
 * This timestamp marks when the buffer entered this server's pipeline.
 *
 * @param buffer GstBuffer to add meta to (must be writable)
 * @param timestamp Timestamp in nanoseconds since UNIX epoch
 * @return Pointer to the added meta, or nullptr on failure
 */
GstReferenceTimestampMeta *buffer_add_server_ingress_meta( GstBuffer *buffer, GstClockTime timestamp );

/**
 * Get server ingress timestamp metadata from a buffer
 *
 * @param buffer GstBuffer to read meta from
 * @return Pointer to the meta, or nullptr if not present
 */
GstReferenceTimestampMeta *buffer_get_server_ingress_meta( GstBuffer *buffer );

/**
 * Add passthrough probes to an encoder element to preserve timestamp meta.
 *
 * Some hardware encoders strip GstMeta. This function adds probes to:
 * - Sink probe: Read and store the timestamp before encoding
 * - Src probe: Re-attach the timestamp to the output buffer if missing
 *
 * @param encoder_element The encoder element or bin to add probes to.
 *                        If it's a bin, the function looks for a child named "encoder".
 * @return Shared pointer to encoder timing statistics, updated by the probes.
 */
std::shared_ptr<EncoderTimingStats>
add_reference_timestamp_passthrough_probes( GstElement *encoder_element );

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_REFERENCE_TIMESTAMP_HELPERS_HPP
