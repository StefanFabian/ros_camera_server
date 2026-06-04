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

#ifndef ROS_CAMERA_SERVER_CAMERA_SERVER_NODE_HPP
#define ROS_CAMERA_SERVER_CAMERA_SERVER_NODE_HPP

#include "ros_camera_server/camera_server.hpp"
#include <rclcpp/node.hpp>
#include <ros_camera_server_msgs/msg/announcement.hpp>

namespace ros_camera_server
{
class CameraServerNode : public rclcpp::Node
{
public:
  using Announcement = ros_camera_server_msgs::msg::Announcement;

  explicit CameraServerNode( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );
  ~CameraServerNode() override = default;

private:
  void publishAnnouncement();

  rclcpp::Publisher<Announcement>::SharedPtr announcement_publisher_;
  std::unique_ptr<CameraServer> camera_server_;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_CAMERA_SERVER_NODE_HPP
