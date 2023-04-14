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

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
