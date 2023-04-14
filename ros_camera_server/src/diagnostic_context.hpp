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

#ifndef ROS_CAMERA_SERVER_DIAGNOSTIC_CONTEXT_HPP
#define ROS_CAMERA_SERVER_DIAGNOSTIC_CONTEXT_HPP

#include "pipeline/pipeline_graph.hpp"

#include <gst/gst.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ros_camera_server
{

inline constexpr const char *DIAGNOSTIC_CONTEXT_KEY = "ros-camera-server-diagnostic-context";

struct DiagnosticContext {
  std::string camera_id;
  std::optional<NodeId> graph_node_id;
  std::vector<size_t> affected_outputs;
};

using DiagnosticContextPtr = std::shared_ptr<const DiagnosticContext>;

void attachDiagnosticContext( GObject *object, DiagnosticContextPtr context );
DiagnosticContextPtr getDiagnosticContext( GObject *object );
std::string formatDiagnosticContext( const DiagnosticContextPtr &context );

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_DIAGNOSTIC_CONTEXT_HPP
