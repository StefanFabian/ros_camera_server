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

#include <gst/gst.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

// ── ANSI color helpers ──────────────────────────────────────────────────────
namespace color
{
const char *reset = "\033[0m";
const char *bold = "\033[1m";
const char *dim = "\033[2m";
const char *red = "\033[31m";
const char *green = "\033[32m";
const char *yellow = "\033[33m";
const char *blue = "\033[34m";
const char *magenta = "\033[35m";
const char *cyan = "\033[36m";
const char *white = "\033[37m";

bool enabled = true;

const char *c( const char *code ) { return enabled ? code : ""; }
} // namespace color

// ── V4L2 device discovery ───────────────────────────────────────────────────
static std::vector<std::string> discoverV4l2Devices()
{
  std::vector<std::string> devices;
  const std::string dev_path = "/dev";
  for ( const auto &entry : std::filesystem::directory_iterator( dev_path ) ) {
    const std::string name = entry.path().filename().string();
    if ( name.rfind( "video", 0 ) == 0 ) {
      devices.push_back( entry.path().string() );
    }
  }
  std::sort( devices.begin(), devices.end() );
  return devices;
}

// ── Caps query for a single device ──────────────────────────────────────────
struct CapsEntry {
  std::string media_type;
  std::string format; // pixel format or codec name from the structure
  int width = 0;
  int height = 0;
  std::string framerate; // human-readable
};

static std::string framerateValueToString( const GValue *value )
{
  if ( GST_VALUE_HOLDS_FRACTION( value ) ) {
    int num = gst_value_get_fraction_numerator( value );
    int den = gst_value_get_fraction_denominator( value );
    return std::to_string( num ) + "/" + std::to_string( den );
  }
  if ( GST_VALUE_HOLDS_FRACTION_RANGE( value ) ) {
    const GValue *min_v = gst_value_get_fraction_range_min( value );
    const GValue *max_v = gst_value_get_fraction_range_max( value );
    return framerateValueToString( min_v ) + " - " + framerateValueToString( max_v );
  }
  if ( GST_VALUE_HOLDS_LIST( value ) ) {
    std::string result;
    guint n = gst_value_list_get_size( value );
    for ( guint i = 0; i < n; ++i ) {
      if ( i > 0 )
        result += ", ";
      result += framerateValueToString( gst_value_list_get_value( value, i ) );
    }
    return result;
  }
  return "?";
}

static std::string dimensionValueToString( const GValue *value )
{
  if ( G_VALUE_HOLDS_INT( value ) ) {
    return std::to_string( g_value_get_int( value ) );
  }
  if ( GST_VALUE_HOLDS_INT_RANGE( value ) ) {
    int min_v = gst_value_get_int_range_min( value );
    int max_v = gst_value_get_int_range_max( value );
    int step = gst_value_get_int_range_step( value );
    std::string result = std::to_string( min_v ) + " - " + std::to_string( max_v );
    if ( step > 1 )
      result += " (step " + std::to_string( step ) + ")";
    return result;
  }
  return "?";
}

namespace
{
struct Resolution {
  std::string width_str;
  std::string height_str;
  bool operator<( const Resolution &o ) const
  {
    if ( width_str != o.width_str )
      return width_str < o.width_str;
    return height_str < o.height_str;
  }
};

struct FormatInfo {
  // resolution string → list of framerate strings
  std::map<std::string, std::vector<std::string>> resolutions;
};
} // namespace

static void printDeviceCaps( const std::string &device_path )
{
  GstElement *src = gst_element_factory_make( "v4l2src", nullptr );
  if ( !src ) {
    std::cerr << color::c( color::red ) << "  Failed to create v4l2src element."
              << color::c( color::reset ) << "\n";
    return;
  }
  g_object_set( G_OBJECT( src ), "device", device_path.c_str(), nullptr );

  if ( gst_element_set_state( src, GST_STATE_READY ) != GST_STATE_CHANGE_SUCCESS ) {
    std::cerr << color::c( color::red ) << "  Could not open device " << device_path
              << color::c( color::reset ) << "\n";
    gst_object_unref( src );
    return;
  }

  // Try to read device name from v4l2src properties
  gchar *device_name = nullptr;
  g_object_get( G_OBJECT( src ), "device-name", &device_name, nullptr );

  GstPad *pad = gst_element_get_static_pad( src, "src" );
  GstCaps *caps = gst_pad_query_caps( pad, nullptr );
  gst_object_unref( pad );

  if ( device_name && std::strlen( device_name ) > 0 ) {
    std::cout << color::c( color::dim ) << "  Name: " << device_name << color::c( color::reset )
              << "\n";
    g_free( device_name );
  }

  if ( !caps || gst_caps_is_empty( caps ) ) {
    std::cout << color::c( color::yellow ) << "  No caps available." << color::c( color::reset )
              << "\n";
    if ( caps )
      gst_caps_unref( caps );
    gst_element_set_state( src, GST_STATE_NULL );
    gst_object_unref( src );
    return;
  }

  // Organize: media_type → format → resolution → framerates
  // Using ordered maps so output is sorted.
  struct FormatData {
    // "WxH" → framerates
    std::map<std::string, std::set<std::string>> resolutions;
  };
  std::map<std::string, std::map<std::string, FormatData>> organized;

  for ( guint i = 0; i < gst_caps_get_size( caps ); ++i ) {
    const GstStructure *s = gst_caps_get_structure( caps, i );
    std::string mt = gst_structure_get_name( s );

    std::string format_name;
    if ( gst_structure_has_field( s, "format" ) ) {
      const GValue *fmt_val = gst_structure_get_value( s, "format" );
      if ( G_VALUE_HOLDS_STRING( fmt_val ) ) {
        format_name = g_value_get_string( fmt_val );
      } else if ( GST_VALUE_HOLDS_LIST( fmt_val ) ) {
        guint n = gst_value_list_get_size( fmt_val );
        for ( guint j = 0; j < n; ++j ) {
          const GValue *v = gst_value_list_get_value( fmt_val, j );
          if ( j > 0 )
            format_name += ", ";
          if ( G_VALUE_HOLDS_STRING( v ) )
            format_name += g_value_get_string( v );
        }
      }
    }
    if ( format_name.empty() )
      format_name = "(default)";

    const GValue *w_val = gst_structure_get_value( s, "width" );
    const GValue *h_val = gst_structure_get_value( s, "height" );
    std::string res_str;
    if ( w_val && h_val ) {
      res_str = dimensionValueToString( w_val ) + " x " + dimensionValueToString( h_val );
    } else {
      res_str = "(any)";
    }

    std::string fps;
    if ( gst_structure_has_field( s, "framerate" ) ) {
      fps = framerateValueToString( gst_structure_get_value( s, "framerate" ) );
    } else {
      fps = "(any)";
    }

    organized[mt][format_name].resolutions[res_str].insert( fps );
  }

  // Print structured output
  for ( const auto &[mt, formats] : organized ) {
    std::cout << "  " << color::c( color::bold ) << color::c( color::cyan ) << mt
              << color::c( color::reset ) << "\n";
    for ( const auto &[fmt, data] : formats ) {
      std::cout << "    " << color::c( color::green ) << "format: " << fmt
                << color::c( color::reset ) << "\n";
      for ( const auto &[res, fps_set] : data.resolutions ) {
        std::cout << "      " << color::c( color::yellow ) << res << color::c( color::reset );
        std::cout << "  " << color::c( color::dim ) << "@  ";
        bool first = true;
        for ( const auto &fps : fps_set ) {
          if ( !first )
            std::cout << ", ";
          std::cout << fps;
          first = false;
        }
        std::cout << color::c( color::reset ) << "\n";
      }
    }
  }

  gst_caps_unref( caps );
  gst_element_set_state( src, GST_STATE_NULL );
  gst_object_unref( src );
}

// ── Configuration hint ──────────────────────────────────────────────────────
static void printConfigHint()
{
  std::cout << "\n"
            << color::c( color::dim )
            << "Use these values in your camera configuration:\n"
               "  input:\n"
               "    type: v4l2\n"
               "    device: \"/dev/videoX\"\n"
               "    width: <width>\n"
               "    height: <height>\n"
               "    framerate: \"<num>/<den>\"        # e.g. \"30/1\"\n"
               "    media_type: \"<media type>\"      # e.g. \"image/jpeg\" (optional)\n"
            << color::c( color::reset ) << std::endl;
}

// ── main ────────────────────────────────────────────────────────────────────
int main( int argc, char *argv[] )
{
  gst_init( &argc, &argv );

  // Check if stdout is a terminal for color support
  color::enabled = isatty( fileno( stdout ) );

  std::string target_device;
  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( arg == "--no-color" ) {
      color::enabled = false;
    } else if ( arg == "--help" || arg == "-h" ) {
      std::cout << "Usage: v4l2_caps_investigator [OPTIONS] [DEVICE]\n\n"
                << "Show available V4L2 camera capabilities.\n\n"
                << "Arguments:\n"
                << "  DEVICE       V4L2 device path (e.g. /dev/video0).\n"
                << "               If omitted, all V4L2 devices are listed.\n\n"
                << "Options:\n"
                << "  --no-color   Disable colored output.\n"
                << "  -h, --help   Show this help message.\n";
      return 0;
    } else {
      target_device = arg;
    }
  }

  if ( !target_device.empty() ) {
    std::cout << color::c( color::bold ) << target_device << color::c( color::reset ) << "\n";
    printDeviceCaps( target_device );
    printConfigHint();
    return 0;
  }

  // Enumerate all devices
  auto devices = discoverV4l2Devices();
  if ( devices.empty() ) {
    std::cout << color::c( color::red ) << "No V4L2 devices found in /dev/."
              << color::c( color::reset ) << "\n";
    return 1;
  }

  for ( const auto &dev : devices ) {
    std::cout << color::c( color::bold ) << dev << color::c( color::reset ) << "\n";
    printDeviceCaps( dev );
    std::cout << "\n";
  }
  printConfigHint();
  return 0;
}
