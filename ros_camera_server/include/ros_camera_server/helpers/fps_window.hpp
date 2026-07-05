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

#ifndef ROS_CAMERA_SERVER_HELPERS_FPS_WINDOW_HPP
#define ROS_CAMERA_SERVER_HELPERS_FPS_WINDOW_HPP

#include "ros_camera_server/helpers/ring_buffer.hpp"

#include <chrono>

namespace ros_camera_server
{

//! Drops timestamps older than the window from the buffer, then returns the average frame rate
//! over the remaining timestamps, or 0 if none remain.
template<size_t Size>
float pruneAndComputeFps( RingBuffer<std::chrono::steady_clock::time_point, Size> &timestamps,
                          std::chrono::steady_clock::time_point now,
                          std::chrono::milliseconds window = std::chrono::milliseconds( 10000 ),
                          int min_timestamps = 3 )
{
  using namespace std::chrono;
  while ( !timestamps.empty() && now - timestamps.front() > window ) { timestamps.pop_front(); }
  if ( timestamps.size() < min_timestamps )
    return 0.0f;
  const auto dt = duration_cast<milliseconds>( now - timestamps.front() ).count();
  if ( dt <= 0 )
    return 0.0f;
  return float( timestamps.size() ) / ( float( dt ) / 1000.0f );
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_HELPERS_FPS_WINDOW_HPP
