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

#ifndef ROS_CAMERA_SERVER_H265_HPP
#define ROS_CAMERA_SERVER_H265_HPP

#include "codec_common.hpp"

namespace ros_camera_server
{

// ============================================================================
// H265 Encoder
// ============================================================================

enum class H265Encoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VA_LP = 0x02,
  VAAPI = 0x04,
  NV = 0x08,
  NVV4L2 = 0x10,
  MPP = 0x20,
  X265 = 0x8000,
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( H265Encoder )

// Priority order: VA_LP > VA > NV > NVV4L2 > MPP > VAAPI > X265
static constexpr CodecBackend h265_encoder_backends[] = {
    { 0x02, "VA_LP", "vah265lpenc" }, { 0x01, "VA", "vah265enc" },
    { 0x08, "NV", "nvh265enc" },      { 0x10, "NVV4L2", "nvv4l2h265enc" },
    { 0x20, "MPP", "mpph265enc" },    { 0x04, "VAAPI", "vaapih265enc" },
    { 0x8000, "X265", "x265enc" },
};

static constexpr CodecAlias h265_encoder_aliases[] = {
    { "SOFTWARE", 0x8000 }, { "SW", 0x8000 }, { "HW", 0xff }, { "AUTO", CODEC_FLAG_AUTO } };

static constexpr CodecDescriptor h265_encoder_desc = {
    "H265",
    "encoder",
    h265_encoder_backends,
    std::size( h265_encoder_backends ),
    h265_encoder_aliases,
    std::size( h265_encoder_aliases ),
};

// ============================================================================
// H265 Decoder
// ============================================================================

enum class H265Decoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VA_LP = 0x02,
  VAAPI = 0x04,
  NV = 0x08,
  NVV4L2 = 0x10,
  AV = 0x2000,
  X265 = 0x8000,
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( H265Decoder )

// Priority order: VA_LP > VA > NV > NVV4L2 > VAAPI > X265 > AV
static constexpr CodecBackend h265_decoder_backends[] = {
    { 0x02, "VA_LP", "vah265lpdec" },  { 0x01, "VA", "vah265dec" },
    { 0x08, "NV", "nvh265dec" },       { 0x10, "NVV4L2", "nvv4l2decoder" },
    { 0x04, "VAAPI", "vaapih265dec" }, { 0x8000, "X265", "x265dec" },
    { 0x2000, "AV", "avdec_h265" },
};

static constexpr CodecAlias h265_decoder_aliases[] = {
    { "SW", 0x8000 | 0x2000 },
    { "NVIDIA", 0x08 | 0x10 },
};

static constexpr CodecDescriptor h265_decoder_desc = {
    "H265",
    "decoder",
    h265_decoder_backends,
    std::size( h265_decoder_backends ),
    h265_decoder_aliases,
    std::size( h265_decoder_aliases ),
};

// ============================================================================
// H265 Encoder creation
// ============================================================================

/**
 * @brief Create a raw H.265 encoder element (without bin wrapper).
 *
 * @param backend The specific backend flag to create
 * @param element_name Name for the created element
 * @param options Codec options (bitrate, etc.)
 * @return GstElement* The encoder element or bin, or nullptr on failure
 */
inline GstElement *createH265EncoderElement( uint32_t backend, const std::string &element_name,
                                             const CodecOptions &options )
{
  using namespace encoder_defaults;

  GstElement *h265_encoder = nullptr;
  int bitrate = options.bitrate;

  if ( backend == static_cast<uint32_t>( H265Encoder::VA_LP ) ) {
    SERVER_LOG_INFO_ONCE( "Using vah265lpenc encoder" );
    h265_encoder = gst_element_factory_make( "vah265lpenc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h265_encoder ), "bitrate", bitrate, "key-int-max", DEFAULT_KEY_INT_MAX,
                  "aud", TRUE, "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS,
                  "num-slices", DEFAULT_NUM_SLICES, "rate-control", 4 /* vbr */, "ref-frames",
                  DEFAULT_REF_FRAMES, nullptr );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::VA ) ) {
    SERVER_LOG_INFO_ONCE( "Using vah265enc encoder" );
    h265_encoder = gst_element_factory_make( "vah265enc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h265_encoder ), "bitrate", bitrate, "key-int-max", DEFAULT_KEY_INT_MAX,
                  "aud", TRUE, "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS,
                  "num-slices", DEFAULT_NUM_SLICES, "rate-control", 4 /* vbr */, "ref-frames",
                  DEFAULT_REF_FRAMES, nullptr );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::VAAPI ) ) {
    SERVER_LOG_INFO_ONCE( "Using vaapih265enc encoder" );
    h265_encoder = gst_element_factory_make( "vaapih265enc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h265_encoder ), "bitrate", bitrate, "keyframe-period", 20,
                  "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS, "num-slices",
                  DEFAULT_NUM_SLICES, "rate-control", 4 /* vbr */, nullptr );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::NV ) ) {
    SERVER_LOG_INFO_ONCE( "Using nvh265enc encoder" );
    h265_encoder = gst_element_factory_make( "nvh265enc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h265_encoder ), "preset", 4 /* low-latency-hq */, "bitrate", bitrate,
                  "gop-size", DEFAULT_GOP_SIZE, "strict-gop", TRUE, "rc-mode", 3 /* vbr */,
                  "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS, "spatial-aq", TRUE,
                  "zerolatency", TRUE, "aud", TRUE, nullptr );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::NVV4L2 ) ) {
    SERVER_LOG_INFO_ONCE( "Using nvv4l2h265enc encoder" );
    // NVV4L2 returns its own bin with nvvidconv
    return createNvv4l2EncoderBin(
        "nvv4l2h265enc", "h265parse", element_name, [bitrate]( GstElement *enc ) {
          g_object_set( G_OBJECT( enc ), "bitrate", bitrate * 1000, "preset-level",
                        1 /* UltraFastPreset */, "maxperf-enable", TRUE, "copy-timestamp", TRUE,
                        "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS, "insert-aud", TRUE,
                        "insert-sps-pps", TRUE, nullptr );
        } );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::MPP ) ) {
    SERVER_LOG_INFO_ONCE( "Using mpph265enc encoder" );
    h265_encoder = gst_element_factory_make( "mpph265enc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    int bps = ( bitrate == 0 ? DEFAULT_BITRATE_KBPS : bitrate ) * 1000;
    g_object_set( G_OBJECT( h265_encoder ), "bps", static_cast<guint>( bps ), "gop",
                  DEFAULT_GOP_SIZE, "header-mode", 1 /* each-idr */, "rc-mode", 0 /* vbr */,
                  "max-pending", 1u, "min-force-key-unit-interval", MIN_KEYFRAME_INTERVAL_NS,
                  "zero-copy-pkt", TRUE, nullptr );
  } else if ( backend == static_cast<uint32_t>( H265Encoder::X265 ) ) {
    SERVER_LOG_INFO_ONCE( "Using x265enc encoder" );
    h265_encoder = gst_element_factory_make( "x265enc", element_name.c_str() );
    if ( !h265_encoder )
      return nullptr;
    g_object_set( G_OBJECT( h265_encoder ), "bitrate", bitrate == 0 ? DEFAULT_BITRATE_KBPS : bitrate,
                  "tune", 4 /* zerolatency */, "speed-preset", 1 /* ultrafast */, "key-int-max",
                  DEFAULT_KEY_INT_MAX, "aud", TRUE, nullptr );
  }

  return h265_encoder;
}

/**
 * @brief Create an H.265 encoder bin with automatic fallback on failure.
 *
 * @param encoder Encoder preference string (e.g., "auto", "va", "nv|x265")
 * @param bin_name Name for the created bin
 * @param options Codec options (bitrate, etc.)
 * @return GstElement* The encoder bin
 * @throws PipelineBuildError if no suitable encoder can be created
 */
inline GstElement *createH265Encoder( const std::string &encoder, const std::string &bin_name,
                                      const CodecOptions &options = {}, uint32_t excluded = 0 )
{
  uint32_t desired = codecFromString( h265_encoder_desc, encoder );

  return codecCreate(
      h265_encoder_desc, desired,
      [&bin_name, &options]( uint32_t backend ) -> GstElement * {
        GstElement *element = createH265EncoderElement( backend, "encoder", options );
        if ( !element )
          return nullptr;
        if ( GST_IS_BIN( element ) ) {
          // NVV4L2 already returns a complete bin
          return element;
        }
        return createEncoderBin( element, "h265parse", bin_name );
      },
      excluded );
}

// ============================================================================
// H265 Decoder creation
// ============================================================================

/**
 * @brief Create an H.265 decoder with automatic hardware fallback.
 *
 * Tries hardware decoders first (VA_LP, VA, NV, NVV4L2, VAAPI), falling back
 * to software (X265, AV). If a decoder fails to create, it automatically tries
 * the next available option.
 *
 * @param name Name for the created element
 * @param decoder Decoder preference string (e.g., "auto", "va", "nv|sw")
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no decoder is available
 */
inline GstElement *createH265Decoder( const std::string &name, const std::string &decoder = "auto" )
{
  uint32_t desired = codecFromString( h265_decoder_desc, decoder );

  return codecCreate( h265_decoder_desc, desired, [&name]( uint32_t backend ) -> GstElement * {
    for ( const auto &b : h265_decoder_backends ) {
      if ( b.flag == backend ) {
        return gst_element_factory_make( b.factory, name.c_str() );
      }
    }
    return nullptr;
  } );
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_H265_HPP
