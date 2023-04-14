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

#ifndef ROS_CAMERA_SERVER_RING_BUFFER_HPP
#define ROS_CAMERA_SERVER_RING_BUFFER_HPP

#include <array>
#include <cstddef>

template<typename T, size_t Size>
class RingBuffer
{
private:
  std::array<T, Size> buffer;
  size_t head = 0;
  size_t tail = 0;
  bool full = false;

public:
  void push( const T &value )
  {
    buffer[head] = value;
    if ( full ) {
      tail = ( tail + 1 ) % Size;
    }
    head = ( head + 1 ) % Size;
    full = ( head == tail );
  }

  //! The oldest element in the buffer.
  const T &front() const { return buffer[tail]; }

  //! The newest element in the buffer.
  const T &back()
  {
    int index = int( head ) - 1;
    if ( index < 0 ) {
      index += Size;
    }
    return buffer[index];
  }

  T &operator[]( size_t index )
  {
    index = tail + index;
    if ( index >= Size ) {
      index -= Size;
    }
    return buffer[index];
  }

  const T &operator[]( size_t index ) const
  {
    index = tail + index;
    if ( index >= Size ) {
      index -= Size;
    }
    return buffer[index];
  }

  bool empty() const { return ( !full && ( head == tail ) ); }

  size_t size() const
  {
    if ( full ) {
      return Size;
    }
    return ( head >= tail ) ? ( head - tail ) : ( Size + head - tail );
  }

  void clear()
  {
    full = false;
    head = 0;
    tail = 0;
  }

  void pop_front()
  {
    if ( empty() ) {
      return; // Nothing to pop
    }
    full = false;
    tail = ( tail + 1 ) % Size;
  }
};
#endif // ROS_CAMERA_SERVER_RING_BUFFER_HPP
