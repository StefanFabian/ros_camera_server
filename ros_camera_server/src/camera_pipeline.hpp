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

#ifndef ROS_CAMERA_SERVER_CAMERA_PIPELINE_HPP
#define ROS_CAMERA_SERVER_CAMERA_PIPELINE_HPP

#include "pipeline/flow_controller.hpp"
#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/helpers/ring_buffer.hpp"
#include "ros_camera_server/helpers/smart_gst_pointer.hpp"
#include "ros_camera_server/statistics.hpp"
#include <atomic>
#include <mutex>
#include <optional>
#include <rclcpp/node.hpp>
#include <vector>

namespace ros_camera_server
{

class PipelineMonitorInterface;

class CameraPipeline
{
public:
  CameraPipeline( rclcpp::Node::SharedPtr node, CameraConfiguration configuration,
                  std::shared_ptr<PipelineMonitorInterface> monitor );
  ~CameraPipeline();

  void buildPipeline();

  const CameraConfiguration &configuration() const { return configuration_; }

  std::string id() const;

  std::string name() const;

  GstState getState() const;

  bool isBuilt() const;

  void start();

  void stop();

  void restart();

  PipelineStatistics statistics() const;

  const std::vector<PipelineOutput::Ptr> &outputs() const { return outputs_; }

  GstBin *gstPipeline() { return pipeline_.get(); }
  const GstBin *gstPipeline() const { return pipeline_.get(); }

  void useClock( GstClock *clock );

  /// Update flow control valves based on current client counts.
  void updateFlowControl();

  /// Returns true if data is flowing to the given output index.
  bool isOutputFlowing( size_t output_index ) const;

  std::chrono::milliseconds uptime() const;

private:
  static gboolean onBusMessage( GstBus *bus, GstMessage *msg, gpointer user_data );

  static GstPadProbeReturn inputBufferCallback( GstPad *pad, GstPadProbeInfo *info,
                                                gpointer user_data );

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PipelineMonitorInterface> monitor_;
  CameraConfiguration configuration_;
  SmartGstPointer<GstBin> pipeline_;
  StreamInput input_;
  std::vector<PipelineOutput::Ptr> outputs_;
  FlowController flow_controller_;
  // Tee request pads issued by PipelineBuilder. Released in the destructor
  // before pipeline_ is unreffed.
  std::vector<std::pair<GstElement *, GstPad *>> tee_request_pads_;

  using clock = std::chrono::steady_clock;
  clock::time_point start_time_;
  mutable std::mutex input_timestamps_mutex_;
  mutable RingBuffer<clock::time_point, 30> input_buffer_timestamps_;
  // Guards current_segment_: written/read on the streaming thread (SEGMENT event, buffer probe)
  // and cleared on the executor thread (restart()).
  mutable std::mutex segment_mutex_;
  std::optional<GstSegment> current_segment_ = std::nullopt;
  guint input_probe_ = 0;

  // The clock we forced onto the pipeline via useClock(), held with our own ref. Read on the
  // streaming thread to derive capture timestamps without taking the per-buffer GST_OBJECT_LOCK.
  SmartGstPointer<GstClock> pipeline_clock_;
  // Pipeline base_time, snapshotted on each PLAYING transition (bus/main-loop thread) and read on
  // the streaming thread. GST_CLOCK_TIME_NONE while not playing -> capture timestamps fall back to
  // server ingress time.
  std::atomic<GstClockTime> pipeline_base_time_ = GST_CLOCK_TIME_NONE;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_CAMERA_PIPELINE_HPP
