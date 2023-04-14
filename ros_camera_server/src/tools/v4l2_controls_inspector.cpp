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

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
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
  for ( const auto &entry : std::filesystem::directory_iterator( "/dev" ) ) {
    const std::string name = entry.path().filename().string();
    if ( name.rfind( "video", 0 ) == 0 ) {
      devices.push_back( entry.path().string() );
    }
  }
  std::sort( devices.begin(), devices.end() );
  return devices;
}

// ── Control type name mapping ───────────────────────────────────────────────
static std::string controlTypeName( __u32 type )
{
  switch ( type ) {
  case V4L2_CTRL_TYPE_INTEGER:
    return "int";
  case V4L2_CTRL_TYPE_BOOLEAN:
    return "bool";
  case V4L2_CTRL_TYPE_MENU:
    return "menu";
  case V4L2_CTRL_TYPE_INTEGER_MENU:
    return "int_menu";
  case V4L2_CTRL_TYPE_BITMASK:
    return "bitmask";
  case V4L2_CTRL_TYPE_BUTTON:
    return "button";
  case V4L2_CTRL_TYPE_INTEGER64:
    return "int64";
  case V4L2_CTRL_TYPE_STRING:
    return "string";
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    return "class";
  default:
    return "unknown(" + std::to_string( type ) + ")";
  }
}

// ── Control class name mapping ──────────────────────────────────────────────
static std::string controlClassName( __u32 id )
{
  __u32 cls = V4L2_CTRL_ID2CLASS( id );
  switch ( cls ) {
  case V4L2_CTRL_CLASS_USER:
    return "User Controls";
  case V4L2_CTRL_CLASS_CAMERA:
    return "Camera Controls";
  case V4L2_CTRL_CLASS_CODEC:
    return "Codec Controls";
  case V4L2_CTRL_CLASS_FLASH:
    return "Flash Controls";
  case V4L2_CTRL_CLASS_IMAGE_SOURCE:
    return "Image Source Controls";
  case V4L2_CTRL_CLASS_IMAGE_PROC:
    return "Image Processing Controls";
  default:
    return "Other Controls (0x" + ( [&]() {
             std::ostringstream oss;
             oss << std::hex << std::setfill( '0' ) << std::setw( 8 ) << cls;
             return oss.str();
           } )() +
           ")";
  }
}

// ── Normalize control name to GStreamer extra-controls key ───────────────────
// GStreamer v4l2src lowercases the name and replaces spaces/non-alnum with '_'
static std::string toGstControlKey( const std::string &name )
{
  std::string result;
  result.reserve( name.size() );
  for ( char ch : name ) {
    if ( std::isalnum( static_cast<unsigned char>( ch ) ) ) {
      result += static_cast<char>( std::tolower( static_cast<unsigned char>( ch ) ) );
    } else {
      result += '_';
    }
  }
  // Trim trailing underscores
  while ( !result.empty() && result.back() == '_' ) { result.pop_back(); }
  return result;
}

struct ControlInfo {
  __u32 id;
  std::string name;
  std::string gst_key;
  __u32 type;
  __s32 minimum;
  __s32 maximum;
  __s32 step;
  __s32 default_value;
  __u32 flags;
  std::vector<std::pair<__s32, std::string>> menu_items;
};

// ── Enumerate controls for a single device ──────────────────────────────────
static std::vector<ControlInfo> enumerateControls( const std::string &device_path )
{
  std::vector<ControlInfo> controls;

  int fd = open( device_path.c_str(), O_RDWR );
  if ( fd < 0 ) {
    std::cerr << color::c( color::red ) << "  Failed to open " << device_path
              << color::c( color::reset ) << "\n";
    return controls;
  }

  // Use V4L2_CTRL_FLAG_NEXT_CTRL to enumerate all controls
  struct v4l2_queryctrl qctrl{};
  qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while ( ioctl( fd, VIDIOC_QUERYCTRL, &qctrl ) == 0 ) {
    if ( qctrl.flags & V4L2_CTRL_FLAG_DISABLED ) {
      qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
      continue;
    }

    ControlInfo info;
    info.id = qctrl.id;
    info.name = reinterpret_cast<const char *>( qctrl.name );
    info.gst_key = toGstControlKey( info.name );
    info.type = qctrl.type;
    info.minimum = qctrl.minimum;
    info.maximum = qctrl.maximum;
    info.step = qctrl.step;
    info.default_value = qctrl.default_value;
    info.flags = qctrl.flags;

    // Enumerate menu items if applicable
    if ( qctrl.type == V4L2_CTRL_TYPE_MENU || qctrl.type == V4L2_CTRL_TYPE_INTEGER_MENU ) {
      struct v4l2_querymenu qmenu{};
      qmenu.id = qctrl.id;
      for ( qmenu.index = static_cast<__u32>( qctrl.minimum );
            qmenu.index <= static_cast<__u32>( qctrl.maximum ); ++qmenu.index ) {
        if ( ioctl( fd, VIDIOC_QUERYMENU, &qmenu ) == 0 ) {
          if ( qctrl.type == V4L2_CTRL_TYPE_MENU ) {
            info.menu_items.emplace_back( static_cast<__s32>( qmenu.index ),
                                          reinterpret_cast<const char *>( qmenu.name ) );
          } else {
            info.menu_items.emplace_back( static_cast<__s32>( qmenu.index ),
                                          std::to_string( qmenu.value ) );
          }
        }
      }
    }

    // Read current value
    struct v4l2_control ctrl{};
    ctrl.id = qctrl.id;
    if ( ioctl( fd, VIDIOC_G_CTRL, &ctrl ) == 0 ) {
      // Store current value in step field (reuse for display - we'll output it separately)
    }

    controls.push_back( info );

    qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }

  close( fd );
  return controls;
}

// ── Read current values ─────────────────────────────────────────────────────
static std::map<__u32, __s32> readCurrentValues( const std::string &device_path,
                                                 const std::vector<ControlInfo> &controls )
{
  std::map<__u32, __s32> values;
  int fd = open( device_path.c_str(), O_RDWR );
  if ( fd < 0 )
    return values;

  for ( const auto &ci : controls ) {
    if ( ci.type == V4L2_CTRL_TYPE_CTRL_CLASS || ci.type == V4L2_CTRL_TYPE_BUTTON )
      continue;

    struct v4l2_control ctrl{};
    ctrl.id = ci.id;
    if ( ioctl( fd, VIDIOC_G_CTRL, &ctrl ) == 0 ) {
      values[ci.id] = ctrl.value;
    }
  }

  close( fd );
  return values;
}

// ── Print controls for a device ─────────────────────────────────────────────
static void printDeviceControls( const std::string &device_path )
{
  auto controls = enumerateControls( device_path );
  if ( controls.empty() ) {
    std::cout << color::c( color::dim ) << "  No controls found.\n" << color::c( color::reset );
    return;
  }

  auto current_values = readCurrentValues( device_path, controls );

  // Group controls by class
  std::string current_class;
  for ( const auto &ci : controls ) {
    // Skip ctrl class headers
    if ( ci.type == V4L2_CTRL_TYPE_CTRL_CLASS ) {
      std::cout << "\n  " << color::c( color::bold ) << color::c( color::cyan ) << ci.name
                << color::c( color::reset ) << "\n";
      current_class = ci.name;
      continue;
    }

    // Print class header if first control without preceding class entry
    std::string cls = controlClassName( ci.id );
    if ( cls != current_class ) {
      current_class = cls;
      std::cout << "\n  " << color::c( color::bold ) << color::c( color::cyan ) << cls
                << color::c( color::reset ) << "\n";
    }

    // Control name and type
    std::cout << "    " << color::c( color::green ) << std::left << std::setw( 35 ) << ci.name
              << color::c( color::reset );
    std::cout << color::c( color::dim ) << "(" << controlTypeName( ci.type ) << ")"
              << color::c( color::reset );

    // Range and value
    if ( ci.type == V4L2_CTRL_TYPE_INTEGER || ci.type == V4L2_CTRL_TYPE_INTEGER64 ) {
      std::cout << "  range=[" << ci.minimum << ".." << ci.maximum << "]"
                << " step=" << ci.step << " default=" << ci.default_value;
    } else if ( ci.type == V4L2_CTRL_TYPE_BOOLEAN ) {
      std::cout << "  default=" << ( ci.default_value ? "true" : "false" );
    } else if ( ci.type == V4L2_CTRL_TYPE_BITMASK ) {
      std::cout << "  max=0x" << std::hex << ci.maximum << std::dec
                << " default=" << ci.default_value;
    }

    // Current value
    auto it = current_values.find( ci.id );
    if ( it != current_values.end() ) {
      std::cout << "  " << color::c( color::yellow ) << "value=" << it->second;
      // For menu type, also show menu item name
      if ( ( ci.type == V4L2_CTRL_TYPE_MENU || ci.type == V4L2_CTRL_TYPE_INTEGER_MENU ) ) {
        for ( const auto &[idx, name] : ci.menu_items ) {
          if ( idx == it->second ) {
            std::cout << " (" << name << ")";
            break;
          }
        }
      }
      std::cout << color::c( color::reset );
    }

    // Flags
    if ( ci.flags & V4L2_CTRL_FLAG_INACTIVE ) {
      std::cout << " " << color::c( color::dim ) << "[inactive]" << color::c( color::reset );
    }
    if ( ci.flags & V4L2_CTRL_FLAG_READ_ONLY ) {
      std::cout << " " << color::c( color::dim ) << "[read-only]" << color::c( color::reset );
    }
    if ( ci.flags & V4L2_CTRL_FLAG_VOLATILE ) {
      std::cout << " " << color::c( color::dim ) << "[volatile]" << color::c( color::reset );
    }
    if ( ci.flags & V4L2_CTRL_FLAG_WRITE_ONLY ) {
      std::cout << " " << color::c( color::dim ) << "[write-only]" << color::c( color::reset );
    }

    std::cout << "\n";

    // Print menu items
    if ( !ci.menu_items.empty() ) {
      for ( const auto &[idx, name] : ci.menu_items ) {
        std::cout << "      " << color::c( color::dim ) << idx << ": " << name
                  << color::c( color::reset ) << "\n";
      }
    }

    // Show GStreamer extra-controls key
    std::cout << "      " << color::c( color::magenta ) << "gst key: " << ci.gst_key
              << color::c( color::reset ) << "\n";
  }
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
      std::cout << "Usage: v4l2_controls_inspector [OPTIONS] [DEVICE]\n\n"
                << "Enumerate available V4L2 camera controls and show how to use them\n"
                << "with GStreamer's v4l2src extra-controls property.\n\n"
                << "Arguments:\n"
                << "  DEVICE       V4L2 device path (e.g. /dev/video0).\n"
                << "               If omitted, all V4L2 devices are listed.\n\n"
                << "Options:\n"
                << "  --no-color   Disable colored output.\n"
                << "  -h, --help   Show this help message.\n\n"
                << "This tool shows:\n"
                << "  1. All V4L2 controls with their type, range, current value, and flags\n"
                << "  2. The GStreamer extra-controls key name for each control\n"
                << "  3. Example gst-launch and C++ code for setting controls\n";
      return 0;
    } else {
      target_device = arg;
    }
  }

  std::cout << color::c( color::bold ) << "V4L2 Controls Inspector" << color::c( color::reset )
            << "\n";
  std::cout << "Enumerates available V4L2 controls and their GStreamer mappings.\n\n";

  if ( !target_device.empty() ) {
    std::cout << color::c( color::bold ) << target_device << color::c( color::reset ) << "\n";
    printDeviceControls( target_device );
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
    // Try to get device name via GStreamer
    GstElement *src = gst_element_factory_make( "v4l2src", nullptr );
    std::string device_name;
    if ( src ) {
      g_object_set( G_OBJECT( src ), "device", dev.c_str(), nullptr );
      if ( gst_element_set_state( src, GST_STATE_READY ) == GST_STATE_CHANGE_SUCCESS ) {
        gchar *name = nullptr;
        g_object_get( G_OBJECT( src ), "device-name", &name, nullptr );
        if ( name ) {
          device_name = name;
          g_free( name );
        }
        gst_element_set_state( src, GST_STATE_NULL );
      }
      gst_object_unref( src );
    }

    std::cout << color::c( color::bold ) << dev << color::c( color::reset );
    if ( !device_name.empty() ) {
      std::cout << " " << color::c( color::dim ) << "(" << device_name << ")"
                << color::c( color::reset );
    }
    std::cout << "\n";

    printDeviceControls( dev );
    std::cout << std::string( 70, '-' ) << "\n\n";
  }

  return 0;
}
