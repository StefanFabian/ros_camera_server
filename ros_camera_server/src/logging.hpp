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

#ifndef ROS_CAMERA_SERVER_LOGGING_HPP
#define ROS_CAMERA_SERVER_LOGGING_HPP

#include <rclcpp/logging.hpp>

#define SERVER_LOG_NAME "ros_camera_server"

#define SERVER_LOG_DEBUG( ... ) RCLCPP_DEBUG( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_DEBUG_STREAM( ... )                                                             \
  RCLCPP_DEBUG_STREAM( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_DEBUG_ONCE( ... )                                                               \
  RCLCPP_DEBUG_ONCE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_DEBUG_THROTTLE( ... )                                                           \
  RCLCPP_DEBUG_THROTTLE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_INFO( ... ) RCLCPP_INFO( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_INFO_STREAM( ... )                                                              \
  RCLCPP_INFO_STREAM( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_INFO_ONCE( ... )                                                                \
  RCLCPP_INFO_ONCE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_INFO_THROTTLE( ... )                                                            \
  RCLCPP_INFO_THROTTLE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_WARN( ... ) RCLCPP_WARN( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_WARN_STREAM( ... )                                                              \
  RCLCPP_WARN_STREAM( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_WARN_ONCE( ... )                                                                \
  RCLCPP_WARN_ONCE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_WARN_THROTTLE( ... )                                                            \
  RCLCPP_WARN_THROTTLE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_ERROR( ... ) RCLCPP_ERROR( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_ERROR_STREAM( ... )                                                             \
  RCLCPP_ERROR_STREAM( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_ERROR_ONCE( ... )                                                               \
  RCLCPP_ERROR_ONCE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )
#define SERVER_LOG_ERROR_THROTTLE( ... )                                                           \
  RCLCPP_ERROR_THROTTLE( rclcpp::get_logger( SERVER_LOG_NAME ), __VA_ARGS__ )

#endif // ROS_CAMERA_SERVER_LOGGING_HPP
