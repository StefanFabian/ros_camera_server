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

#ifndef ROS_CAMERA_SERVER_SMART_GST_POINTER_HPP
#define ROS_CAMERA_SERVER_SMART_GST_POINTER_HPP

#include <gst/gst.h>
#include <string>

namespace ros_camera_server
{

template<typename T>
struct SmartGstPointer {
  SmartGstPointer( T *obj ) noexcept : obj_( obj ) { } // NOLINT(google-explicit-constructor)

  SmartGstPointer() noexcept : obj_( nullptr ) { }
  SmartGstPointer( SmartGstPointer &&other ) noexcept : obj_( other.obj_ ) { other.obj_ = nullptr; }
  SmartGstPointer( const SmartGstPointer &other ) : obj_( other.obj_ )
  {
    if ( obj_ != nullptr )
      gst_object_ref( obj_ );
  }

  ~SmartGstPointer()
  {
    if ( obj_ == nullptr )
      return;
    gst_object_unref( obj_ );
  }

  bool operator!() noexcept { return !obj_; }

  operator T *() noexcept // NOLINT(google-explicit-constructor)
  { return obj_; }

  T &operator*() noexcept { return *obj_; }

  T *operator->() noexcept { return obj_; }

  T *&get() noexcept { return obj_; }

  const T *get() const noexcept { return obj_; }

  /// Release ownership of the pointer without unreffing.
  /// Returns the raw pointer and sets the internal pointer to nullptr.
  /// Use when transferring ownership (e.g., to gst_bin_add).
  T *release() noexcept
  {
    T *tmp = obj_;
    obj_ = nullptr;
    return tmp;
  }

  SmartGstPointer &operator=( SmartGstPointer &&other ) noexcept
  {
    if ( obj_ != nullptr )
      gst_object_unref( obj_ );
    obj_ = other.obj_;
    other.obj_ = nullptr;
    return *this;
  }

  SmartGstPointer &operator=( const SmartGstPointer &other )
  {
    if ( &other == this )
      return *this;
    if ( obj_ != nullptr )
      gst_object_unref( obj_ );
    obj_ = other.obj_;
    if ( obj_ != nullptr )
      gst_object_ref( obj_ );
    return *this;
  }

private:
  T *obj_;
};
} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_SMART_GST_POINTER_HPP
