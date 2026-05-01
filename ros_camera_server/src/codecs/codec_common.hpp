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

#ifndef ROS_CAMERA_SERVER_CODEC_COMMON_HPP
#define ROS_CAMERA_SERVER_CODEC_COMMON_HPP

#include <algorithm>
#include <atomic>
#include <functional>
#include <gst/gstbin.h>
#include <initializer_list>
#include <sstream>
#include <string>
#include <string_view>

#include "../logging.hpp"
#include "ros_camera_server/exceptions.hpp"
#include "ros_camera_server/helpers/smart_gst_pointer.hpp"

namespace ros_camera_server
{
constexpr uint32_t CODEC_FLAG_AUTO = 0xffff;

/// Check if a GStreamer element factory with the given name exists.
inline bool check_encoder( const std::string &name )
{
  SmartGstPointer<GstElementFactory> factory = gst_element_factory_find( name.c_str() );
  return !!factory;
}

// ============================================================================
// Options
// ============================================================================

/// Options passed to encoder/decoder creation functions.
/// Extensible for future codec configuration needs.
struct CodecOptions {
  int bitrate = 0; // Target bitrate in kbps (0 = codec default)
};

// ============================================================================
// Backend descriptors for data-driven codec registration
// ============================================================================

/// Describes a single encoder/decoder backend (e.g., "vah264enc").
struct CodecBackend {
  uint32_t flag;       // Enum bitflag value for this backend
  const char *name;    // Human-readable name (e.g., "VA_LP")
  const char *factory; // GStreamer element factory name (e.g., "vah264lpenc")
};

/// Maps an alias string to one or more backend flags.
/// Example: "SOFTWARE" -> OPENH264 | X264
struct CodecAlias {
  const char *name; // Alias string (uppercase)
  uint32_t flags;   // OR of backend flags this alias expands to
};

/// Holds the full backend/alias tables for a codec direction (encoder or decoder).
struct CodecDescriptor {
  const char *codec_name;       // e.g. "H264"
  const char *direction;        // "encoder" or "decoder"
  const CodecBackend *backends; // Priority-ordered backend list
  size_t backend_count;
  const CodecAlias *aliases; // Alias mappings (SOFTWARE, SW, etc.)
  size_t alias_count;
};

// ============================================================================
// Bitwise operator macro for enum class flags
// ============================================================================

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CODEC_BITFLAG_OPS( EnumType )                                                              \
  inline constexpr EnumType operator|( EnumType a, EnumType b )                                    \
  { return static_cast<EnumType>( static_cast<uint32_t>( a ) | static_cast<uint32_t>( b ) ); }     \
  inline constexpr EnumType operator&( EnumType a, EnumType b )                                    \
  { return static_cast<EnumType>( static_cast<uint32_t>( a ) & static_cast<uint32_t>( b ) ); }     \
  inline constexpr EnumType operator~( EnumType a )                                                \
  { return static_cast<EnumType>( ~static_cast<uint32_t>( a ) ); }                                 \
  inline constexpr bool intersects( EnumType a, EnumType b )                                       \
  { return ( static_cast<uint32_t>( a ) & static_cast<uint32_t>( b ) ) != 0; }

// ============================================================================
// Generic codec functions (data-driven)
// ============================================================================

/// Format a bitflag enum value as a pipe-separated string using the backend table.
inline std::string codecToString( const CodecDescriptor &desc, uint32_t value )
{
  if ( value == CODEC_FLAG_AUTO )
    return "AUTO";
  if ( value == 0 )
    return "NONE";
  std::stringstream ss;
  for ( size_t i = 0; i < desc.backend_count; ++i ) {
    if ( value & desc.backends[i].flag )
      ss << desc.backends[i].name << "|";
  }
  std::string str = ss.str();
  if ( !str.empty() && str.back() == '|' )
    str.pop_back();
  return str;
}

/// Parse a pipe-delimited string into bitflags using the backend table and aliases.
inline uint32_t codecFromString( const CodecDescriptor &desc, std::string_view str )
{
  if ( str.empty() )
    return CODEC_FLAG_AUTO;
  uint32_t result = 0;
  std::string normalized( str );
  std::transform( str.begin(), str.end(), normalized.begin(),
                  []( unsigned char c ) { return std::toupper( c ); } );

  size_t start = 0;
  size_t end = 0;
  do {
    end = normalized.find( '|', start );
    std::string token = end == std::string::npos ? normalized.substr( start )
                                                 : normalized.substr( start, end - start );
    if ( token == "AUTO" ) {
      return CODEC_FLAG_AUTO;
    }

    // Check aliases first
    bool found = false;
    for ( size_t i = 0; i < desc.alias_count; ++i ) {
      if ( token == desc.aliases[i].name ) {
        result |= desc.aliases[i].flags;
        found = true;
        break;
      }
    }
    if ( !found ) {
      // Check backends
      for ( size_t i = 0; i < desc.backend_count; ++i ) {
        if ( token == desc.backends[i].name ) {
          result |= desc.backends[i].flag;
          found = true;
          break;
        }
      }
    }
    if ( !found ) {
      SERVER_LOG_ERROR_STREAM( "Unknown " << desc.codec_name << " " << desc.direction << " token: '"
                                          << token << "'. Ignoring." );
    }
    start = end + 1;
  } while ( end != std::string::npos );
  return result;
}

/// Select the best available backend by checking GStreamer factory availability.
/// Backends are checked in priority order (the order they appear in the descriptor).
inline uint32_t codecSelect( const CodecDescriptor &desc, uint32_t desired, uint32_t excluded = 0 )
{
  for ( size_t i = 0; i < desc.backend_count; ++i ) {
    uint32_t flag = desc.backends[i].flag;
    if ( ( desired & flag ) && !( excluded & flag ) &&
         gst_element_factory_find( desc.backends[i].factory ) ) {
      return flag;
    }
  }
  if ( ( desired | excluded ) == CODEC_FLAG_AUTO ) { // AUTO already tried everything
    return 0;                                        // NONE
  }
  SERVER_LOG_WARN_STREAM( "Unsupported " << desc.codec_name << " " << desc.direction << " '"
                                         << codecToString( desc, desired )
                                         << "'. Falling back to auto." );
  return codecSelect( desc, CODEC_FLAG_AUTO, excluded );
}

/// Generic create loop: tries to create an element using the createElement callback,
/// falling back to the next available backend on failure.
///
/// @param desc The codec descriptor
/// @param desired Bitflags of desired backends
/// @param createElement Callback that creates a GstElement* for a given backend flag.
///                       Returns nullptr on failure.
/// @returns The created GstElement*
/// @throws PipelineBuildError if no backend can produce an element
inline GstElement *codecCreate( const CodecDescriptor &desc, uint32_t desired,
                                const std::function<GstElement *( uint32_t )> &createElement,
                                uint32_t excluded = 0 )
{
  while ( true ) {
    uint32_t selected = codecSelect( desc, desired, excluded );
    if ( selected == 0 ) {
      throw PipelineBuildError( std::string( "No suitable " ) + desc.codec_name + " " +
                                desc.direction +
                                " found. Please install an appropriate GStreamer plugin." );
    }

    GstElement *element = createElement( selected );
    if ( element )
      return element;

    SERVER_LOG_WARN_STREAM( "Failed to create " << desc.codec_name << " " << desc.direction << " "
                                                << codecToString( desc, selected )
                                                << ", trying fallback" );
    excluded |= selected;
  }
}

// ============================================================================
// Memory feature flags for HW scaler / encoder compatibility
// ============================================================================

/// Memory types that GStreamer elements can produce or consume.
/// Used to determine compatibility between HW scalers and encoder backends.
enum class MemoryFeature : uint32_t {
  NONE = 0x00,
  VA = 0x01,         // memory:VAMemory
  VA_SURFACE = 0x02, // memory:VASurface
  DMA_BUF = 0x04,    // memory:DMABuf
  CUDA = 0x08,       // memory:CUDAMemory
  NVMM = 0x10,       // memory:NVMM
};

CODEC_BITFLAG_OPS( MemoryFeature )

/// Common bitflags for encoder/decoder backends across different codecs.
enum class CodecBackendFlag : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VA_LP = 0x02,
  VAAPI = 0x04,
  NV = 0x08,
  NVV4L2 = 0x10,
  MPP = 0x20,
};

/// Returns the HW memory types an encoder backend accepts as input.
/// Returns MemoryFeature::NONE for software-only backends (MPP, x264, etc.).
inline MemoryFeature encoderInputMemoryFlags( CodecBackendFlag backend_flag )
{
  switch ( backend_flag ) {
  case CodecBackendFlag::VA:
  case CodecBackendFlag::VA_LP:
    return MemoryFeature::VA | MemoryFeature::DMA_BUF;
  case CodecBackendFlag::VAAPI:
    return MemoryFeature::VA_SURFACE | MemoryFeature::DMA_BUF;
  case CodecBackendFlag::NV:
    return MemoryFeature::CUDA;
  case CodecBackendFlag::NVV4L2:
    return MemoryFeature::NVMM;
  default:
    return MemoryFeature::NONE;
  }
}

/// Maps a single MemoryFeature bit to its GStreamer caps feature string.
inline const char *capsFeatureForMemoryFlag( MemoryFeature flag )
{
  switch ( flag ) {
  case MemoryFeature::VA:
    return "memory:VAMemory";
  case MemoryFeature::VA_SURFACE:
    return "memory:VASurface";
  case MemoryFeature::DMA_BUF:
    return "memory:DMABuf";
  case MemoryFeature::CUDA:
    return "memory:CUDAMemory";
  case MemoryFeature::NVMM:
    return "memory:NVMM";
  default:
    return nullptr;
  }
}

/// Describes a HW scaler candidate for the selection algorithm.
struct HwScalerCandidate {
  const char *factory;         // Primary scaler element factory name
  const char *upload_factory;  // Optional upload element (nullptr if not needed)
  MemoryFeature output_memory; // Memory flags this scaler produces
};

/// Priority-ordered list of HW scaler candidates.
inline constexpr std::array<HwScalerCandidate, 5> HW_SCALER_CANDIDATES = { {
    { "vapostproc", nullptr, MemoryFeature::VA },
    { "vaapipostproc", nullptr, MemoryFeature::VA_SURFACE },
    { "nvvideoconvert", nullptr, MemoryFeature::CUDA | MemoryFeature::NVMM },
    { "nvvidconv", nullptr, MemoryFeature::CUDA | MemoryFeature::NVMM },
    { "cudascale", "cudaupload", MemoryFeature::CUDA },
} };

/// Returns the highest-priority overlapping MemoryFeature bit between two flag sets.
inline MemoryFeature highestPriorityOverlap( MemoryFeature a, MemoryFeature b )
{
  uint32_t overlap = static_cast<uint32_t>( a ) & static_cast<uint32_t>( b );
  if ( overlap == 0 )
    return MemoryFeature::NONE;
  // Return lowest set bit (highest priority since flags are ordered by bit position)
  return static_cast<MemoryFeature>( overlap & ( -overlap ) );
}

// ============================================================================
// Encoder configuration constants
// ============================================================================

namespace encoder_defaults
{
constexpr gint64 MIN_KEYFRAME_INTERVAL_NS = 1'000'000'000L; // 1s
constexpr int DEFAULT_BITRATE_KBPS = 1000;                  // 1 Mbps
constexpr int DEFAULT_KEY_INT_MAX = 10;
constexpr int DEFAULT_GOP_SIZE = 30;
// 1 slice = no slicing, >1 can reduce latency at the cost of compression efficiency
// Some encoders require the sliced image to have dimensions that are multiples of e.g. 16
// and will fail otherwise. This is why we default to 1.
constexpr int DEFAULT_NUM_SLICES = 1;
constexpr int DEFAULT_REF_FRAMES = 1;
} // namespace encoder_defaults

// ============================================================================
// Encoder bin creation helpers
// ============================================================================

/**
 * @brief Create a standard encoder bin with videoconvert, encoder, and parser.
 *
 * The bin converts input to I420 format (required by most H.264/H.265 encoders
 * to produce standard profiles), then encodes and parses the stream.
 *
 * @param encoder The encoder element (already configured)
 * @param parser_factory The parser element factory name (e.g., "h264parse", "h265parse")
 * @param bin_name Name for the created bin
 * @param config_interval Parser config interval (-1 for every frame)
 * @return GstElement* The encoder bin
 * @throws PipelineBuildError on failure
 */
inline GstElement *createEncoderBin( GstElement *encoder, const char *parser_factory,
                                     const std::string &bin_name, int config_interval = -1 )
{
  GstElement *encoding_bin = gst_bin_new( bin_name.c_str() );
  GstElement *convert = gst_element_factory_make( "videoconvert", "convert" );
  GstElement *parser = gst_element_factory_make( parser_factory, "parser" );

  if ( !convert || !parser ) {
    if ( convert )
      gst_object_unref( convert );
    if ( parser )
      gst_object_unref( parser );
    gst_object_unref( encoding_bin );
    gst_object_unref( encoder );
    throw PipelineBuildError( "Failed to create encoder bin elements" );
  }

  g_object_set( G_OBJECT( parser ), "config-interval", config_interval, nullptr );

  GstPad *encoder_input_pad = gst_element_get_static_pad( convert, "sink" );
  GstPad *encoder_output_pad = gst_element_get_static_pad( parser, "src" );

  gst_bin_add_many( GST_BIN( encoding_bin ), convert, encoder, parser, nullptr );
  gst_element_link_many( convert, encoder, parser, nullptr );
  gst_element_add_pad( GST_ELEMENT( encoding_bin ), gst_ghost_pad_new( "sink", encoder_input_pad ) );
  gst_element_add_pad( GST_ELEMENT( encoding_bin ), gst_ghost_pad_new( "src", encoder_output_pad ) );
  gst_object_unref( encoder_input_pad );
  gst_object_unref( encoder_output_pad );

  return encoding_bin;
}

/**
 * @brief Create an NV4L2 encoder bin with nvvidconv preprocessing.
 *
 * @param encoder_factory The encoder element factory name
 * @param parser_factory The parser element factory name
 * @param bin_name Name for the created bin
 * @param config_func Function to configure the encoder element
 * @return GstElement* The encoder bin
 * @throws PipelineBuildError on failure
 */
template<typename ConfigFunc>
inline GstElement *createNvv4l2EncoderBin( const char *encoder_factory, const char *parser_factory,
                                           const std::string &bin_name, ConfigFunc config_func )
{
  GstElement *encoder_bin = gst_bin_new( bin_name.c_str() );
  GstElement *nvvidconv = gst_element_factory_make( "nvvidconv", "nvvidconv" );
  GstElement *encoder = gst_element_factory_make( encoder_factory, "encoder" );
  GstElement *parser = gst_element_factory_make( parser_factory, "parser" );

  if ( !nvvidconv || !encoder || !parser ) {
    if ( nvvidconv )
      gst_object_unref( nvvidconv );
    if ( encoder )
      gst_object_unref( encoder );
    if ( parser )
      gst_object_unref( parser );
    gst_object_unref( encoder_bin );
    throw PipelineBuildError( std::string( "Failed to create NVV4L2 encoder elements for " ) +
                              encoder_factory );
  }

  // Configure the encoder
  config_func( encoder );
  g_object_set( G_OBJECT( parser ), "config-interval", -1, nullptr );

  gst_bin_add_many( GST_BIN( encoder_bin ), nvvidconv, encoder, parser, nullptr );
  gst_element_link_many( nvvidconv, encoder, parser, nullptr );

  GstPad *sink_pad = gst_element_get_static_pad( nvvidconv, "sink" );
  GstPad *src_pad = gst_element_get_static_pad( parser, "src" );
  gst_element_add_pad( GST_ELEMENT( encoder_bin ), gst_ghost_pad_new( "sink", sink_pad ) );
  gst_element_add_pad( GST_ELEMENT( encoder_bin ), gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( sink_pad );
  gst_object_unref( src_pad );

  return encoder_bin;
}

/**
 * @brief Wrap an image (JPEG/PNG) encoder in a bin with a videoconvert prepended.
 *
 * Some HW image encoders (vajpegenc, vaapijpegenc, ...) accept narrower input
 * caps than the SW path (e.g. no packed RGB/BGR). The videoconvert is a
 * passthrough when caps already match and otherwise converts to a format the
 * encoder accepts.
 *
 * @param encoder The encoder element (already configured, ownership transferred)
 * @param bin_name Name for the bin
 * @return GstElement* The encoder bin
 * @throws PipelineBuildError on failure
 */
inline GstElement *createImageEncoderBin( GstElement *encoder, const std::string &bin_name )
{
  GstElement *encoder_bin = gst_bin_new( bin_name.c_str() );
  GstElement *convert = gst_element_factory_make( "videoconvert", "convert" );

  if ( !convert ) {
    gst_object_unref( encoder_bin );
    gst_object_unref( encoder );
    throw PipelineBuildError( "Failed to create videoconvert for image encoder bin" );
  }

  GstPad *sink_pad = gst_element_get_static_pad( convert, "sink" );
  GstPad *src_pad = gst_element_get_static_pad( encoder, "src" );

  gst_bin_add_many( GST_BIN( encoder_bin ), convert, encoder, nullptr );
  gst_element_link( convert, encoder );
  gst_element_add_pad( GST_ELEMENT( encoder_bin ), gst_ghost_pad_new( "sink", sink_pad ) );
  gst_element_add_pad( GST_ELEMENT( encoder_bin ), gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( sink_pad );
  gst_object_unref( src_pad );

  return encoder_bin;
}

// ============================================================================
// Decoder bin creation helper
// ============================================================================

/**
 * @brief Wrap a decoder element in a bin with a videoconvert appended.
 *
 * Hardware decoders (VA, VAAPI, NV) often output frames in GPU memory
 * (e.g. video/x-raw(memory:VAMemory)) which downstream elements like
 * rbfimagesink cannot consume. The videoconvert ensures the output is
 * always in system-memory video/x-raw.
 *
 * @param decoder The decoder element (already configured, ownership transferred)
 * @param bin_name Name for the bin
 * @return GstElement* The decoder bin
 * @throws PipelineBuildError on failure
 */
inline GstElement *createDecoderBin( GstElement *decoder, const char *bin_name )
{
  GstElement *decoder_bin = gst_bin_new( bin_name );
  GstElement *convert = gst_element_factory_make( "videoconvert", "convert" );

  if ( !convert ) {
    gst_object_unref( decoder_bin );
    gst_object_unref( decoder );
    throw PipelineBuildError( "Failed to create videoconvert for decoder bin" );
  }

  GstPad *sink_pad = gst_element_get_static_pad( decoder, "sink" );
  GstPad *src_pad = gst_element_get_static_pad( convert, "src" );

  gst_bin_add_many( GST_BIN( decoder_bin ), decoder, convert, nullptr );
  gst_element_link( decoder, convert );
  gst_element_add_pad( GST_ELEMENT( decoder_bin ), gst_ghost_pad_new( "sink", sink_pad ) );
  gst_element_add_pad( GST_ELEMENT( decoder_bin ), gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( sink_pad );
  gst_object_unref( src_pad );

  return decoder_bin;
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_CODEC_COMMON_HPP
