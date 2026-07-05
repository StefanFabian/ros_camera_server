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

#include <gtest/gtest.h>
#include <ros_camera_server/helpers/fps_window.hpp>
#include <ros_camera_server/helpers/ring_buffer.hpp>

TEST( RingBufferTest, TestPushAndSize )
{
  RingBuffer<int, 3> buffer;
  EXPECT_TRUE( buffer.empty() );
  EXPECT_EQ( buffer.size(), 0 );

  buffer.push( 1 );
  EXPECT_FALSE( buffer.empty() );
  EXPECT_EQ( buffer.size(), 1 );
  EXPECT_EQ( buffer.front(), 1 );
  EXPECT_EQ( buffer.back(), 1 );

  buffer.push( 2 );
  EXPECT_EQ( buffer.size(), 2 );
  EXPECT_EQ( buffer.front(), 1 ); // Oldest
  EXPECT_EQ( buffer.back(), 2 );  // Newest

  buffer.push( 3 );
  EXPECT_EQ( buffer.size(), 3 );
  EXPECT_EQ( buffer.front(), 1 );
  EXPECT_EQ( buffer.back(), 3 );

  // Overwrite
  buffer.push( 4 );
  EXPECT_EQ( buffer.size(), 3 );
  EXPECT_EQ( buffer.front(), 2 ); // 1 was overwritten
  EXPECT_EQ( buffer.back(), 4 );
}

TEST( RingBufferTest, TestAccess )
{
  RingBuffer<int, 5> buffer;
  buffer.push( 10 );
  buffer.push( 20 );
  buffer.push( 30 );

  EXPECT_EQ( buffer[0], 10 );
  EXPECT_EQ( buffer[1], 20 );
  EXPECT_EQ( buffer[2], 30 );

  // Wrap around scenario
  // Fill buffer: [10, 20, 30, 40, 50]
  buffer.push( 40 );
  buffer.push( 50 );

  // Overwrite: [60, 20, 30, 40, 50] (head moves)
  buffer.push( 60 );
  // tail should be at index 1 (value 20)
  // logical index 0 -> buffer[1] = 20

  EXPECT_EQ( buffer[0], 20 );
  EXPECT_EQ( buffer[1], 30 );
  EXPECT_EQ( buffer[2], 40 );
  EXPECT_EQ( buffer[3], 50 );
  EXPECT_EQ( buffer[4], 60 );

  EXPECT_EQ( buffer.front(), 20 );
  EXPECT_EQ( buffer.back(), 60 );
}

TEST( RingBufferTest, TestPopFront )
{
  RingBuffer<int, 3> buffer;
  buffer.push( 1 );
  buffer.push( 2 );

  EXPECT_EQ( buffer.size(), 2 );
  buffer.pop_front();
  EXPECT_EQ( buffer.size(), 1 );
  EXPECT_EQ( buffer.front(), 2 );

  buffer.pop_front();
  EXPECT_TRUE( buffer.empty() );

  // Pop empty
  buffer.pop_front();
  EXPECT_TRUE( buffer.empty() );
}

TEST( RingBufferTest, TestClear )
{
  RingBuffer<int, 3> buffer;
  buffer.push( 1 );
  buffer.push( 2 );
  buffer.clear();
  EXPECT_TRUE( buffer.empty() );
  EXPECT_EQ( buffer.size(), 0 );

  buffer.push( 3 );
  EXPECT_EQ( buffer.size(), 1 );
  EXPECT_EQ( buffer.front(), 3 );
}

namespace
{
using time_point = std::chrono::steady_clock::time_point;
using namespace std::chrono_literals;

//! Fill the buffer with timestamps at a fixed interval, ending one interval before now.
template<size_t Size>
void fillAtInterval( RingBuffer<time_point, Size> &buffer, time_point now,
                     std::chrono::milliseconds interval, size_t count )
{
  for ( size_t i = count; i > 0; --i ) { buffer.push( now - i * interval ); }
}
} // namespace

TEST( FpsWindowTest, EmptyBufferReturnsZero )
{
  RingBuffer<time_point, 30> buffer;
  EXPECT_EQ( ros_camera_server::pruneAndComputeFps( buffer, time_point( 10000s ) ), 0.0f );
}

TEST( FpsWindowTest, SteadyRateFullBuffer )
{
  // 25 fps: 30 timestamps spanning 1.2s, all inside the 3s window.
  RingBuffer<time_point, 30> buffer;
  const time_point now( 10000s );
  fillAtInterval( buffer, now, 40ms, 30 );
  EXPECT_NEAR( ros_camera_server::pruneAndComputeFps( buffer, now, 3s ), 25.0f, 0.1f );
  EXPECT_EQ( buffer.size(), 30u );
}

TEST( FpsWindowTest, LowRatePrunesBeforeComputingDt )
{
  // 5 fps: a full 30-slot buffer spans 6s, so half the timestamps are outside the 3s window.
  // dt must come from the oldest timestamp remaining after pruning, not from before.
  RingBuffer<time_point, 30> buffer;
  const time_point now( 10000s );
  fillAtInterval( buffer, now, 200ms, 30 );
  EXPECT_NEAR( ros_camera_server::pruneAndComputeFps( buffer, now, 3s ), 5.0f, 0.4f );
  EXPECT_LT( buffer.size(), 30u );
}

TEST( FpsWindowTest, MinTimestamps )
{
  RingBuffer<time_point, 30> buffer;
  const time_point now( 10000s );
  const auto interval_ms = 40ms;
  fillAtInterval( buffer, now, interval_ms, 30 );
  EXPECT_EQ( ros_camera_server::pruneAndComputeFps( buffer, now + 3s, 3s, 1 ), 0.0f );
  EXPECT_TRUE( buffer.empty() );

  buffer.clear();
  fillAtInterval( buffer, now, interval_ms, 30 );
  EXPECT_GT( ros_camera_server::pruneAndComputeFps( buffer, now + 3s - interval_ms, 3s, 1 ), 0.0f );
  EXPECT_EQ( buffer.size(), 1 );

  buffer.clear();
  fillAtInterval( buffer, now, interval_ms, 30 );
  EXPECT_EQ( ros_camera_server::pruneAndComputeFps( buffer, now + 3s - interval_ms, 3s, 2 ), 0.0f );
  EXPECT_EQ( buffer.size(), 1 );
}

TEST( FpsWindowTest, SingleTimestampAtNowReturnsZero )
{
  // dt of 0 must not divide by zero.
  RingBuffer<time_point, 30> buffer;
  const time_point now( 10000s );
  buffer.push( now );
  EXPECT_EQ( ros_camera_server::pruneAndComputeFps( buffer, now ), 0.0f );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
