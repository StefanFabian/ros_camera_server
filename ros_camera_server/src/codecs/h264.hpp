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

#ifndef ROS_CAMERA_SERVER_H264_HPP
#define ROS_CAMERA_SERVER_H264_HPP

#include "codec_common.hpp"

#include <gst/gstbin.h>
#include <sstream>
#include <string>

namespace ros_camera_server
{

// ============================================================================
// H264 Encoder
// ============================================================================

enum class H264Encoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VA_LP = 0x02,
  VAAPI = 0x04,
  NV = 0x08,
  NVV4L2 = 0x10,
  MPP = 0x20,
  AV = 0x2000,
  OPENH264 = 0x4000,
  X264 = 0x8000,
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( H264Encoder )

// Priority-ordered encoder backends (VA_LP > VA > NV > NVV4L2 > VAAPI > OPENH264 > X264)
constexpr CodecBackend h264_encoder_backends[] = {
    { 0x02, "VA_LP", "vah264lpenc" },      { 0x01, "VA", "vah264enc" },
    { 0x08, "NV", "nvh264enc" },           { 0x10, "NVV4L2", "nvv4l2h264enc" },
    { 0x20, "MPP", "mpph264enc" },         { 0x04, "VAAPI", "vaapih264enc" },
    { 0x4000, "OPENH264", "openh264enc" }, { 0x8000, "X264", "x264enc" },
};

constexpr CodecAlias h264_encoder_aliases[] = { { "SOFTWARE", 0x4000 | 0x8000 },
                                                { "SW", 0x4000 | 0x8000 },
                                                { "HW", 0xff },
                                                { "AUTO", CODEC_FLAG_AUTO } };

inline const CodecDescriptor &h264EncoderDescriptor()
{
  static const CodecDescriptor desc = {
      "H264",
      "encoder",
      h264_encoder_backends,
      std::size( h264_encoder_backends ),
      h264_encoder_aliases,
      std::size( h264_encoder_aliases ),
  };
  return desc;
}

inline std::ostream &operator<<( std::ostream &stream, H264Encoder encoder )
{
  stream << codecToString( h264EncoderDescriptor(), static_cast<uint32_t>( encoder ) );
  return stream;
}

// Forward declaration
inline GstElement *createH264EncoderElement( H264Encoder encoder, const std::string &element_name,
                                             const CodecOptions &options );

/**
 * @brief Create an H.264 encoder bin including videoconvert and codec parser with automatic fallback on failure.
 *
 * @param encoder Encoder preference string (e.g., "auto", "va", "nv|x264")
 * @param bin_name Name for the created bin
 * @param options Codec options (bitrate, etc.)
 * @return GstElement* The encoder bin
 * @throws PipelineBuildError if no suitable encoder can be created
 */
inline GstElement *createH264Encoder( const std::string &encoder, const std::string &bin_name,
                                      const CodecOptions &options = {}, uint32_t excluded = 0 )
{
  using namespace encoder_defaults;

  const auto &desc = h264EncoderDescriptor();
  uint32_t desired = codecFromString( desc, encoder );

  return codecCreate(
      desc, desired,
      [&bin_name, &options]( uint32_t selected ) -> GstElement * {
        GstElement *element =
            createH264EncoderElement( static_cast<H264Encoder>( selected ), "encoder", options );
        if ( !element )
          return nullptr;
        if ( GST_IS_BIN( element ) ) {
          // NVV4L2 already returns a complete bin with nvvidconv
          return element;
        }
        // Wrap encoder with videoconvert and parser
        return createEncoderBin( element, "h264parse", bin_name );
      },
      excluded );
}

/**
 * @brief Create a raw H.264 encoder element (without bin wrapper).
 *
 * @param encoder The specific encoder to create
 * @param element_name Name for the created element
 * @param options Codec options (bitrate, etc.)
 * @return GstElement* The encoder element, or nullptr on failure
 */
inline GstElement *createH264EncoderElement( H264Encoder encoder, const std::string &element_name,
                                             const CodecOptions &options )
{
  using namespace encoder_defaults;

  GstElement *h264_encoder = nullptr;

  if ( encoder == H264Encoder::VA_LP ) {
    SERVER_LOG_INFO_ONCE( "Using vah264lpenc encoder" );
    h264_encoder = gst_element_factory_make( "vah264lpenc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h264_encoder ), "bitrate", options.bitrate, "key-int-max",
                  DEFAULT_KEY_INT_MAX, "aud", TRUE, "min-force-key-unit-interval",
                  MIN_KEYFRAME_INTERVAL_NS, "num-slices", DEFAULT_NUM_SLICES, "cabac", TRUE, "dct8x8",
                  TRUE, "rate-control", 4 /* vbr */, "ref-frames", DEFAULT_REF_FRAMES, nullptr );
  } else if ( encoder == H264Encoder::VA ) {
    SERVER_LOG_INFO_ONCE( "Using vah264enc encoder" );
    h264_encoder = gst_element_factory_make( "vah264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h264_encoder ), "bitrate", options.bitrate, "key-int-max",
                  DEFAULT_KEY_INT_MAX, "aud", TRUE, "min-force-key-unit-interval",
                  MIN_KEYFRAME_INTERVAL_NS, "num-slices", DEFAULT_NUM_SLICES, "cabac", TRUE, "dct8x8",
                  TRUE, "rate-control", 4 /* vbr */, "ref-frames", DEFAULT_REF_FRAMES, nullptr );
  } else if ( encoder == H264Encoder::VAAPI ) {
    SERVER_LOG_INFO_ONCE( "Using vaapih264enc encoder" );
    h264_encoder = gst_element_factory_make( "vaapih264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h264_encoder ), "bitrate", options.bitrate, "keyframe-period", 20,
                  "prediction-type", 1 /* hierarchical-p */, "tune", 1 /* high-compression */,
                  "aud", TRUE, "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS,
                  "num-slices", DEFAULT_NUM_SLICES, "cabac", TRUE, "dct8x8", TRUE, "rate-control",
                  4 /* vbr */, nullptr );
  } else if ( encoder == H264Encoder::NV ) {
    SERVER_LOG_INFO_ONCE( "Using nvh264enc encoder" );
    h264_encoder = gst_element_factory_make( "nvh264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h264_encoder ), "preset", 4 /* low-latency-hq */, "bitrate",
                  options.bitrate, "gop-size", DEFAULT_GOP_SIZE, "strict-gop", TRUE, "rc-mode",
                  3 /* vbr */, "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS,
                  "spatial-aq", TRUE, "zerolatency", TRUE, "aud", TRUE, nullptr );
  } else if ( encoder == H264Encoder::NVV4L2 ) {
    SERVER_LOG_INFO_ONCE( "Using nvv4l2h264enc encoder" );
    // NVV4L2 returns its own bin with nvvidconv
    int bitrate = options.bitrate;
    return createNvv4l2EncoderBin(
        "nvv4l2h264enc", "h264parse", element_name, [bitrate]( GstElement *enc ) {
          g_object_set( G_OBJECT( enc ), "profile", 0 /* Baseline */, "bitrate", bitrate * 1000,
                        "preset-level", 1 /* UltraFastPreset */, "maxperf-enable", TRUE,
                        "copy-timestamp", TRUE, "min-force-key-unit-interval",
                        encoder_defaults::MIN_KEYFRAME_INTERVAL_NS, "insert-aud", TRUE,
                        "insert-sps-pps", TRUE, nullptr );
        } );
  } else if ( encoder == H264Encoder::MPP ) {
    SERVER_LOG_INFO_ONCE( "Using mpph264enc encoder" );
    h264_encoder = gst_element_factory_make( "mpph264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    int bps = ( options.bitrate == 0 ? DEFAULT_BITRATE_KBPS : options.bitrate ) * 1000;
    g_object_set( G_OBJECT( h264_encoder ), "bps", static_cast<guint>( bps ), "gop",
                  DEFAULT_GOP_SIZE, "header-mode", 1 /* each-idr */, "profile", 66 /* baseline */,
                  "rc-mode", 0 /* vbr */, "max-pending", 1u, "min-force-key-unit-interval",
                  MIN_KEYFRAME_INTERVAL_NS, "zero-copy-pkt", TRUE, nullptr );
  } else if ( encoder == H264Encoder::OPENH264 ) {
    SERVER_LOG_INFO_ONCE( "Using openh264enc encoder" );
    h264_encoder = gst_element_factory_make( "openh264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    int bitrate_bps = ( options.bitrate == 0 ? DEFAULT_BITRATE_KBPS : options.bitrate ) * 1000;
    g_object_set( G_OBJECT( h264_encoder ), "bitrate", bitrate_bps, "max-bitrate", bitrate_bps,
                  "rate-control", 1 /* bitrate mode */, "gop-size", DEFAULT_GOP_SIZE,
                  "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS, nullptr );
  } else if ( encoder == H264Encoder::X264 ) {
    SERVER_LOG_INFO_ONCE( "Using x264enc encoder" );
    h264_encoder = gst_element_factory_make( "x264enc", element_name.c_str() );
    if ( !h264_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h264_encoder ), "bitrate",
                  options.bitrate == 0 ? DEFAULT_BITRATE_KBPS : options.bitrate, "tune",
                  4 /* zerolatency */, "speed-preset", 1 /* superfast */, "key-int-max",
                  DEFAULT_KEY_INT_MAX, "intra-refresh", TRUE, "aud", TRUE, "b-adapt", FALSE,
                  nullptr );
  }

  return h264_encoder;
}

// ============================================================================
// H264 Decoder
// ============================================================================

enum class H264Decoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,         // vah264dec (newer VA-API)
  VA_LP = 0x02,      // vah264lpdec (low-power VA-API)
  VAAPI = 0x04,      // vaapih264dec (legacy VA-API)
  NV = 0x08,         // nvh264dec (NVIDIA)
  NVV4L2 = 0x10,     // nvv4l2decoder (NVIDIA Jetson)
  AV = 0x2000,       // avdec_h264
  OPENH264 = 0x4000, // openh264dec
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( H264Decoder )

// Priority-ordered decoder backends (VA_LP > VA > NV > NVV4L2 > VAAPI > OPENH264 > AV)
constexpr CodecBackend h264_decoder_backends[] = {
    { 0x02, "VA_LP", "vah264lpdec" },  { 0x01, "VA", "vah264dec" },
    { 0x08, "NV", "nvh264dec" },       { 0x10, "NVV4L2", "nvv4l2decoder" },
    { 0x04, "VAAPI", "vaapih264dec" }, { 0x4000, "OPENH264", "openh264dec" },
    { 0x2000, "AV", "avdec_h264" },
};

constexpr CodecAlias h264_decoder_aliases[] = {
    { "SOFTWARE", 0x4000 | 0x2000 },
    { "SW", 0x4000 | 0x2000 },
    { "NVIDIA", 0x08 },
};

inline const CodecDescriptor &h264DecoderDescriptor()
{
  static const CodecDescriptor desc = {
      "H264",
      "decoder",
      h264_decoder_backends,
      std::size( h264_decoder_backends ),
      h264_decoder_aliases,
      std::size( h264_decoder_aliases ),
  };
  return desc;
}

inline std::ostream &operator<<( std::ostream &stream, H264Decoder decoder )
{
  stream << codecToString( h264DecoderDescriptor(), static_cast<uint32_t>( decoder ) );
  return stream;
}

/**
 * @brief Create an H.264 decoder with automatic hardware fallback.
 *
 * Tries hardware decoders first (VA, NV, NVV4L2, VAAPI), falling back to software.
 * If a decoder fails to create, it automatically tries the next available option.
 *
 * @param name Name for the created element
 * @param decoder Decoder preference string (e.g., "auto", "va", "nv|AV")
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no decoder is available
 */
inline GstElement *createH264Decoder( const std::string &name, const std::string &decoder = "auto" )
{
  const auto &desc = h264DecoderDescriptor();
  uint32_t desired = codecFromString( desc, decoder );

  return codecCreate( desc, desired, [&desc, &name]( uint32_t selected ) -> GstElement * {
    // Find the factory name for this backend
    for ( size_t i = 0; i < desc.backend_count; ++i ) {
      if ( desc.backends[i].flag == selected ) {
        return gst_element_factory_make( desc.backends[i].factory, name.c_str() );
      }
    }
    return nullptr;
  } );
}

/**
 * @brief Create an H.264 decoder with automatic fallback on failure.
 *
 * @param name Name for the created element
 * @param decoders Bitflags of acceptable decoders
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no suitable decoder can be created
 */
inline GstElement *createH264Decoder( const std::string &name, H264Decoder decoders )
{
  const auto &desc = h264DecoderDescriptor();

  return codecCreate(
      desc, static_cast<uint32_t>( decoders ), [&desc, &name]( uint32_t selected ) -> GstElement * {
        for ( size_t i = 0; i < desc.backend_count; ++i ) {
          if ( desc.backends[i].flag == selected ) {
            return gst_element_factory_make( desc.backends[i].factory, name.c_str() );
          }
        }
        return nullptr;
      } );
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_H264_HPP
