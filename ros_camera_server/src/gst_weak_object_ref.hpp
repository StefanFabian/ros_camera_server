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

#ifndef ROS_CAMERA_SERVER_GST_WEAK_OBJECT_REF_HPP
#define ROS_CAMERA_SERVER_GST_WEAK_OBJECT_REF_HPP

#include <glib-object.h>

namespace ros_camera_server
{

// RAII wrapper around GWeakRef. Move-only. Safe to init/set/get from any thread.
class WeakObjectRef
{
public:
  WeakObjectRef() { g_weak_ref_init( &ref_, nullptr ); }
  ~WeakObjectRef() { g_weak_ref_clear( &ref_ ); }

  WeakObjectRef( const WeakObjectRef & ) = delete;
  WeakObjectRef &operator=( const WeakObjectRef & ) = delete;

  WeakObjectRef( WeakObjectRef &&other ) noexcept
  {
    g_weak_ref_init( &ref_, nullptr );
    auto *obj = static_cast<GObject *>( g_weak_ref_get( &other.ref_ ) );
    g_weak_ref_set( &ref_, obj );
    if ( obj != nullptr ) {
      g_object_unref( obj );
    }
    g_weak_ref_set( &other.ref_, nullptr );
  }

  WeakObjectRef &operator=( WeakObjectRef &&other ) noexcept
  {
    if ( this != &other ) {
      auto *obj = static_cast<GObject *>( g_weak_ref_get( &other.ref_ ) );
      g_weak_ref_set( &ref_, obj );
      if ( obj != nullptr ) {
        g_object_unref( obj );
      }
      g_weak_ref_set( &other.ref_, nullptr );
    }
    return *this;
  }

  void set( GObject *object ) { g_weak_ref_set( &ref_, object ); }
  // Returns a strong ref (caller must g_object_unref) or nullptr if the object is gone.
  GObject *acquire() { return static_cast<GObject *>( g_weak_ref_get( &ref_ ) ); }

private:
  GWeakRef ref_;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_GST_WEAK_OBJECT_REF_HPP
