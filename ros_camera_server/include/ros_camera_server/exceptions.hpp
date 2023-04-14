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

#ifndef ROS_CAMERA_SERVER_EXCEPTIONS_HPP
#define ROS_CAMERA_SERVER_EXCEPTIONS_HPP

#include <stdexcept>

namespace ros_camera_server
{

class FactoryException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class PipelineBuildError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_EXCEPTIONS_HPP
