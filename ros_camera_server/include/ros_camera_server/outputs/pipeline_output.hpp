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

#ifndef ROS_CAMERA_SERVER_PIPELINE_OUTPUT_HPP
#define ROS_CAMERA_SERVER_PIPELINE_OUTPUT_HPP

#include "ros_camera_server/helpers/ring_buffer.hpp"
#include "ros_camera_server/statistics.hpp"
#include <chrono>
#include <gst/gstbin.h>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/node.hpp>
#include <ros_camera_server_msgs/msg/camera_stream.hpp>

namespace ros_camera_server
{

struct CameraServerConfiguration;

class PipelineOutput
{
public:
  using Ptr = std::unique_ptr<PipelineOutput>;

  PipelineOutput( rclcpp::Node::SharedPtr node, std::string camera_id, int output_index )
      : node_( std::move( node ) ), camera_id_( std::move( camera_id ) ),
        output_index_( output_index )
  {
  }

  virtual ~PipelineOutput();

  virtual PipelineOutputStatistics statistics();

  /// Create a CameraStream message with runtime values
  virtual ros_camera_server_msgs::msg::CameraStream
  toCameraStreamMsg( const CameraServerConfiguration &config ) const = 0;

  /// Get the number of connected clients (-1 if not supported, 0 if no clients, >0 for client count)
  virtual int getClientCount() const { return -1; }

  GstBin *bin = nullptr;

  // Encoder name for this output path (e.g., "nvh264enc", "x264enc")
  // Set by PipelineBuilder during pipeline construction
  std::string encoder_name;

  // Configured codec for this output path (e.g., "h264", "h265")
  // Used as fallback when caps are not yet negotiated (e.g., valve closed)
  std::string configured_codec;

  /// Set shared encoder timing statistics (called by PipelineBuilder)
  void setEncoderTimingStats( std::shared_ptr<struct EncoderTimingStats> stats )
  { encoder_timing_stats_ = std::move( stats ); }

protected:
  //! If your pipeline is simple, you can use this to create the statistics instead of overriding.
  //! @returns The capture timestamp for the given buffer, or 0 if no timestamp could be found.
  guint64 reportBufferSent( const GstBuffer *buffer );

  const rclcpp::Node::SharedPtr &node() const { return node_; }
  const std::string &cameraId() const { return camera_id_; }
  int outputIndex() const { return output_index_; }

private:
  using clock = std::chrono::steady_clock;
  mutable std::mutex stats_mutex_;
  mutable RingBuffer<clock::time_point, 30> output_buffer_timestamps_;
  mutable RingBuffer<std::chrono::microseconds, 30> buffer_processing_times_;
  mutable RingBuffer<std::chrono::microseconds, 30> buffer_total_processing_times_;
  mutable GstClockTime last_buffer_timestamp_ = 0;
  // Capture time of the current output frame (may span multiple buffers)
  guint64 current_frame_capture_time_ns_ = 0;
  // Server ingress time of the current output frame (when buffer entered this server's pipeline)
  guint64 current_frame_ingress_time_ns_ = 0;

  rclcpp::Node::SharedPtr node_;
  std::string camera_id_;
  int output_index_;
  bool is_new_frame_ = false;
  std::shared_ptr<struct EncoderTimingStats> encoder_timing_stats_;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_OUTPUT_HPP
