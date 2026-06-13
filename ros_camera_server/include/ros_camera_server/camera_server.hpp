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

#ifndef ROS_CAMERA_SERVER_CAMERA_SERVER_HPP
#define ROS_CAMERA_SERVER_CAMERA_SERVER_HPP

#include "ros_camera_server/configuration.hpp"

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <unordered_map>

namespace ros_camera_server
{
class CameraPipeline;
class PipelineMonitor;
class SignalingServer;

class CameraServer
{
public:
  explicit CameraServer( const rclcpp::Node::SharedPtr &node, CameraServerConfiguration configuration,
                         std::function<void()> on_initialized = {} );
  ~CameraServer();

  const CameraServerConfiguration &configuration() const { return configuration_; }

  const std::vector<std::unique_ptr<CameraPipeline>> &pipelines() const { return pipelines_; }

  bool isInitialized() const { return initialized_.load(); }

  //! Port the signaling server is listening on, or 0 if it isn't running. Only meaningful after
  //! isInitialized() returns true.
  int signalingPort() const;

private:
  static void onGStreamerLog( GstDebugCategory *category, GstDebugLevel level, const gchar *file,
                              const gchar *function, gint line, GObject *object,
                              GstDebugMessage *message, gpointer user_data ) G_GNUC_NO_INSTRUMENT;

  void handleGStreamerLog( GstDebugCategory *category, GstDebugLevel level, GObject *object,
                           GstDebugMessage *message );

  void updateStatus();

  void checkAndRepair();

  /// Check if any camera has WebRTC outputs and start signaling server if needed.
  void initializeWebRTC();

  /// Build the given (not yet built) pipeline, wire up its WebRTC outputs and start it.
  /// Must run on the GStreamer thread: buildPipeline() installs a bus watch and other GSources
  /// that bind to the thread-default GMainContext, and the signaling server is only safe to touch
  /// from that context. checkAndRepair() dispatches this onto the GStreamer main loop.
  void rebuildPipeline( CameraPipeline *pipeline );

  /// Run fn on the GStreamer thread's main loop.
  void invokeOnGstThread( std::function<void()> fn );

  /// Dispatch a pipeline restart to the GStreamer thread and reset the pipeline's
  /// stuck-output counter.
  void restartPipeline( CameraPipeline *pipeline );

  /// Update flow control valves based on client counts.
  void updateFlowControl();

  rclcpp::Node::SharedPtr node_;
  CameraServerConfiguration configuration_;
  std::vector<std::unique_ptr<CameraPipeline>> pipelines_;
  std::shared_ptr<PipelineMonitor> pipeline_monitor_;
  std::unique_ptr<SignalingServer> signaling_server_;
  std::thread gstreamer_thread_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr check_and_repair_timer_;
  rclcpp::TimerBase::SharedPtr flow_control_timer_;
  SmartGstPointer<GstObject> clock_provider_;
  std::atomic<bool> initialized_{ false };
  GMainLoop *g_main_loop_ = nullptr;
  /// Consecutive checkAndRepair ticks an output of the pipeline was stuck (clients connected and
  /// data flow enabled, but 0 fps while the input produces frames). Only touched on the ROS
  /// timer thread.
  std::unordered_map<CameraPipeline *, int> stuck_output_ticks_;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_CAMERA_SERVER_HPP
