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

#ifndef ROS_CAMERA_SERVER_PIPELINE_MONITOR_INTERFACE_HPP
#define ROS_CAMERA_SERVER_PIPELINE_MONITOR_INTERFACE_HPP

#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <memory>
#include <string>

namespace ros_camera_server
{

// Receives diagnostic events from the GStreamer log handler and pipeline buses. Implementations
// may aggregate, filter, log, or trigger recovery. Methods may be invoked from arbitrary
// GStreamer threads; implementations are responsible for their own synchronization.
class PipelineMonitorInterface
{
public:
  using SharedPtr = std::shared_ptr<PipelineMonitorInterface>;

  virtual ~PipelineMonitorInterface() = default;

  virtual void observeGStreamerLog( GstDebugCategory *category, GstDebugLevel level,
                                    GObject *object, GstDebugMessage *message ) = 0;
  virtual void observeBusWarning( const std::string &camera_id, GstObject *source,
                                  const GError *error, const char *debug_info ) = 0;
  virtual void observeBusError( const std::string &camera_id, GstObject *source,
                                const GError *error, const char *debug_info ) = 0;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_MONITOR_INTERFACE_HPP
