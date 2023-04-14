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

#ifndef ROS_CAMERA_SERVER_STATISTICS_HPP
#define ROS_CAMERA_SERVER_STATISTICS_HPP

#include <chrono>
#include <gst/gstelement.h>
#include <string>
#include <vector>

namespace ros_camera_server
{

struct PipelineOutputStatistics {
  float fps = 0.0f;
  std::chrono::microseconds processing_time = std::chrono::microseconds{ -1 };
  // Total processing time including upstream processing (only set when different from processing_time)
  std::chrono::microseconds total_processing_time = std::chrono::microseconds{ -1 };

  // Encoder timing breakdown (from shared encoder probes)
  std::chrono::microseconds pre_encoder_time = std::chrono::microseconds{ -1 };
  std::chrono::microseconds encoder_latency = std::chrono::microseconds{ -1 };

  // Actual format information from negotiated caps
  std::string codec;
  int width = 0;
  int height = 0;

  // Encoder element name (e.g., "nvh264enc", "x264enc", "vah264enc")
  std::string encoder_name;

  int client_count = -1; // -1 = unknown / not supported, 0 = no clients, >0 = client count
};

struct PipelineStatistics {
  GstState state;
  float input_fps = 0.0f;
  std::vector<PipelineOutputStatistics> output_statistics;
};
} // namespace ros_camera_server
#endif // ROS_CAMERA_SERVER_STATISTICS_HPP
