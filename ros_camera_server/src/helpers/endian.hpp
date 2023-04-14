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

#ifndef ROS_CAMERA_SERVER_ENDIAN_HPP
#define ROS_CAMERA_SERVER_ENDIAN_HPP

#include <cstdint>
#if defined( __cpp_lib_endian ) && __cpp_lib_endian >= 201907L
#include <bit>
#endif

namespace ros_camera_server
{
#if defined( __BYTE_ORDER__ ) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ ||                         \
    defined( __BIG_ENDIAN__ ) || defined( __ARMEB__ ) || defined( __THUMBEB__ ) ||                 \
    defined( __AARCH64EB__ ) || defined( _MIBSEB ) || defined( __MIBSEB ) || defined( __MIBSEB__ )
constexpr bool is_little_endian = false; // Big-endian
#define ENDIAN_ATTRIBUTES constexpr
#elif defined( __BYTE_ORDER__ ) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ ||                    \
    defined( __LITTLE_ENDIAN__ ) || defined( __ARMEL__ ) || defined( __THUMBEL__ ) ||              \
    defined( __AARCH64EL__ ) || defined( _MIPSEL ) || defined( __MIPSEL ) || defined( __MIPSEL__ )
constexpr bool is_little_endian = true; // Little-endian
#define ENDIAN_ATTRIBUTES constexpr
#elif defined( __cpp_lib_endian ) && __cpp_lib_endian >= 201907L
constexpr bool is_little_endian = std::endian::native == std::endian::little;
#define ENDIAN_ATTRIBUTES constexpr
#else
inline const bool is_little_endian = []() { return ( *(const uint16_t *)"\x01\x02" == 0x0201 ); }();
#define ENDIAN_ATTRIBUTES
#endif

#if defined( __cpp_lib_byteswap ) && __cpp_lib_byteswap >= 202110L
constexpr uint16_t byteswap( uint16_t value ) { return std::byteswap( value ); }

constexpr uint32_t byteswap( uint32_t value ) { return std::byteswap( value ); }

constexpr uint64_t byteswap( uint64_t value ) { return std::byteswap( value ); }
#else
constexpr uint16_t byteswap( uint16_t value )
{ return static_cast<uint16_t>( value << 8 ) | static_cast<uint16_t>( value >> 8 ); }

constexpr uint32_t byteswap( uint32_t value )
{
  return ( ( value & 0x000000FF ) << 24 ) | ( ( value & 0x0000FF00 ) << 8 ) |
         ( ( value & 0x00FF0000 ) >> 8 ) | ( ( value & 0xFF000000 ) >> 24 );
}

constexpr uint64_t byteswap( uint64_t value )
{
  return ( ( value & 0x00000000000000FFuLL ) << 56 ) | ( ( value & 0x000000000000FF00uLL ) << 40 ) |
         ( ( value & 0x0000000000FF0000uLL ) << 24 ) | ( ( value & 0x00000000FF000000uLL ) << 8 ) |
         ( ( value & 0x000000FF00000000uLL ) >> 8 ) | ( ( value & 0x0000FF0000000000uLL ) >> 24 ) |
         ( ( value & 0x00FF000000000000uLL ) >> 40 ) | ( ( value & 0xFF00000000000000uLL ) >> 56 );
}
#endif

ENDIAN_ATTRIBUTES uint16_t hosttole16( uint16_t value )
{ return is_little_endian ? value : byteswap( value ); }
ENDIAN_ATTRIBUTES uint16_t hosttobe16( uint16_t value )
{ return !is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint32_t hosttole32( uint32_t value )
{ return is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint32_t hosttobe32( uint32_t value )
{ return !is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint64_t hosttole64( uint64_t value )
{ return is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint64_t hosttobe64( uint64_t value )
{ return !is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint16_t le16tohost( uint16_t value )
{ return is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint16_t be16tohost( uint16_t value )
{ return !is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint32_t le32tohost( uint32_t value )
{ return is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint32_t be32tohost( uint32_t value )
{ return !is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint64_t le64tohost( uint64_t value )
{ return is_little_endian ? value : byteswap( value ); }

ENDIAN_ATTRIBUTES uint64_t be64tohost( uint64_t value )
{ return !is_little_endian ? value : byteswap( value ); }

#undef ENDIAN_ATTRIBUTES

} // namespace ros_camera_server

#endif
