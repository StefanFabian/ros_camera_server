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

#include "ros_camera_server/outputs/pipeline_output.hpp"
#include "../logging.hpp"
#include "ros_camera_server/helpers/reference_timestamp_helpers.hpp"

#include <gst/gstcaps.h>
#include <gst/gstpad.h>
#include <gst/gststructure.h>
#include <gst/video/video-info.h>

namespace ros_camera_server
{

PipelineOutput::~PipelineOutput()
{
  // If our bin was never adopted by a parent pipeline (e.g. PipelineBuilder
  // failed before gst_bin_add, or this output is being used standalone), drop
  // its (possibly floating) ref so the bin and its children are freed.
  if ( bin == nullptr )
    return;
  if ( GST_OBJECT_PARENT( bin ) != nullptr )
    return;
  if ( g_object_is_floating( G_OBJECT( bin ) ) )
    g_object_ref_sink( G_OBJECT( bin ) );
  gst_object_unref( GST_OBJECT( bin ) );
}

namespace
{
std::string codecFromMediaType( const std::string &media_type )
{
  if ( media_type == "video/x-raw" )
    return "raw";
  if ( media_type == "video/x-h264" )
    return "h264";
  if ( media_type == "video/x-h265" )
    return "h265";
  if ( media_type == "image/jpeg" )
    return "jpeg";
  if ( media_type == "image/png" )
    return "png";
  return media_type;
}
} // namespace

PipelineOutputStatistics PipelineOutput::statistics()
{
  using namespace std::chrono;
  std::lock_guard<std::mutex> lock( stats_mutex_ );
  PipelineOutputStatistics stats;

  auto now = clock::now();
  // Drop old timestamps
  while ( !output_buffer_timestamps_.empty() &&
          duration_cast<milliseconds>( now - output_buffer_timestamps_.front() ).count() > 3000 ) {
    output_buffer_timestamps_.pop_front();
  }
  if ( !output_buffer_timestamps_.empty() ) {
    // Calculate dt from the oldest remaining timestamp (after pruning)
    auto dt = duration_cast<milliseconds>( now - output_buffer_timestamps_.front() ).count();
    if ( dt > 0 )
      stats.fps = float( output_buffer_timestamps_.size() ) / ( float( dt ) / 1000.f );
  }
  if ( !buffer_processing_times_.empty() ) {
    std::chrono::microseconds sum( 0 );
    for ( size_t i = 0; i < buffer_processing_times_.size(); ++i ) {
      sum += buffer_processing_times_[i];
    }
    stats.processing_time = sum / buffer_processing_times_.size();
  }
  if ( !buffer_total_processing_times_.empty() ) {
    std::chrono::microseconds sum( 0 );
    for ( size_t i = 0; i < buffer_total_processing_times_.size(); ++i ) {
      sum += buffer_total_processing_times_[i];
    }
    stats.total_processing_time = sum / buffer_total_processing_times_.size();
  }

  // Query actual format from negotiated caps
  if ( bin != nullptr ) {
    GstPad *sink_pad = gst_element_get_static_pad( GST_ELEMENT( bin ), "sink" );
    if ( sink_pad != nullptr ) {
      GstCaps *caps = gst_pad_get_current_caps( sink_pad );
      if ( caps != nullptr && gst_caps_get_size( caps ) > 0 ) {
        GstStructure *structure = gst_caps_get_structure( caps, 0 );
        const gchar *media_type = gst_structure_get_name( structure );
        if ( media_type != nullptr ) {
          stats.codec = codecFromMediaType( media_type );
        }
        gst_structure_get_int( structure, "width", &stats.width );
        gst_structure_get_int( structure, "height", &stats.height );
        gst_caps_unref( caps );
      }
      gst_object_unref( sink_pad );
    }
  }

  // Fall back to configured codec when caps aren't negotiated (e.g., valve closed, no clients)
  if ( stats.codec.empty() && !configured_codec.empty() ) {
    stats.codec = configured_codec;
  }

  // Read encoder timing breakdown from shared probes
  if ( encoder_timing_stats_ ) {
    int64_t pre_enc = encoder_timing_stats_->avg_pre_encoder_us.load( std::memory_order_relaxed );
    int64_t enc = encoder_timing_stats_->avg_encode_time_us.load( std::memory_order_relaxed );
    if ( pre_enc >= 0 )
      stats.pre_encoder_time = microseconds( pre_enc );
    if ( enc >= 0 )
      stats.encoder_latency = microseconds( enc );
  }

  stats.encoder_name = encoder_name;
  stats.client_count = getClientCount();
  return stats;
}

guint64 PipelineOutput::reportBufferSent( const GstBuffer *buffer )
{
  using namespace std::chrono;
  // Get the current time as soon as the buffer arrives
  auto end_time = clock::now();
  guint64 ros_now = node_->now().nanoseconds();

  std::lock_guard<std::mutex> lock( stats_mutex_ );
  // Use PTS to detect new frames (multiple buffers can come from same frame)
  GstClockTime pts = GST_BUFFER_PTS( buffer );

  // Check if new frame
  bool new_frame = pts == GST_CLOCK_TIME_NONE || pts != last_buffer_timestamp_;
  if ( new_frame ) {
    last_buffer_timestamp_ = pts;
    buffer_processing_times_.pop_front();
    buffer_total_processing_times_.pop_front();
    is_new_frame_ = true;

    // Read capture time from GstMeta (attached at pipeline input or upstream)
    GstReferenceTimestampMeta *capture_meta =
        buffer_get_unix_timestamp_meta( const_cast<GstBuffer *>( buffer ) );
    if ( capture_meta ) {
      current_frame_capture_time_ns_ = capture_meta->timestamp;
    } else {
      // Warn once per output if meta is missing (considered a bug)
      SERVER_LOG_WARN_THROTTLE(
          *node_->get_clock(),
          15000, "[%s/%d] Capture time meta missing from buffer - encoder may have stripped it. This message is throttled.",
          camera_id_.c_str(), output_index_ );
      current_frame_capture_time_ns_ = 0;
    }

    // Read server ingress time for processing time measurement (this server only)
    GstReferenceTimestampMeta *ingress_meta =
        buffer_get_server_ingress_meta( const_cast<GstBuffer *>( buffer ) );
    if ( ingress_meta ) {
      current_frame_ingress_time_ns_ = ingress_meta->timestamp;
    } else {
      current_frame_ingress_time_ns_ = 0;
    }
  }

  guint64 capture_time = current_frame_capture_time_ns_;
  guint64 ingress_time = current_frame_ingress_time_ns_;
  if ( ingress_time != 0 ) {
    auto server_duration = duration_cast<microseconds>( nanoseconds( ros_now - ingress_time ) );
    auto total_duration = capture_time != 0
                              ? duration_cast<microseconds>( nanoseconds( ros_now - capture_time ) )
                              : server_duration;
    if ( is_new_frame_ ) {
      buffer_processing_times_.push( server_duration );
      buffer_total_processing_times_.push( total_duration );
      is_new_frame_ = false;
    } else {
      // Update the last processing time for this frame
      buffer_processing_times_[buffer_processing_times_.size() - 1] = server_duration;
      buffer_total_processing_times_[buffer_total_processing_times_.size() - 1] = total_duration;
    }
  }

  if ( new_frame || output_buffer_timestamps_.empty() ) {
    output_buffer_timestamps_.push( end_time );
    return capture_time;
  }
  // This buffer is still from the same frame as the last one, update the timestamp to now
  output_buffer_timestamps_[output_buffer_timestamps_.size() - 1] = end_time;
  return capture_time;
}

} // namespace ros_camera_server
