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

#ifndef ROS_CAMERA_SERVER_JPEG_HPP
#define ROS_CAMERA_SERVER_JPEG_HPP

#include "codec_common.hpp"

namespace ros_camera_server
{

// ============================================================================
// JPEG Decoder
// ============================================================================

enum class JpegDecoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VAAPI = 0x02,
  NVJPEG = 0x04,
  MPP = 0x08,
  SOFTWARE = 0x8000,
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( JpegDecoder )

// Priority order: VA > NVJPEG > VAAPI > MPP >  SOFTWARE
constexpr CodecBackend jpeg_decoder_backends[] = {
    { 0x01, "VA", "vajpegdec" },       { 0x04, "NVJPEG", "nvjpegdec" },
    { 0x08, "MPP", "mppjpegdec" },     { 0x02, "VAAPI", "vaapijpegdec" },
    { 0x8000, "SOFTWARE", "jpegdec" },
};

constexpr CodecAlias jpeg_decoder_aliases[] = {
    { "NVIDIA", static_cast<uint32_t>( JpegDecoder::NVJPEG ) },
    { "SW", static_cast<uint32_t>( JpegDecoder::SOFTWARE ) },
};

inline const CodecDescriptor &jpegDecoderDescriptor()
{
  static const CodecDescriptor desc{
      "JPEG",
      "decoder",
      jpeg_decoder_backends,
      std::size( jpeg_decoder_backends ),
      jpeg_decoder_aliases,
      std::size( jpeg_decoder_aliases ),
  };
  return desc;
}

/**
 * @brief Create a JPEG decoder with automatic hardware fallback.
 *
 * Tries hardware decoders first (VA > NVJPEG > VAAPI), falling back to software.
 * If a decoder fails to create, it automatically tries the next available option.
 *
 * @param decoder Decoder preference string (e.g., "auto", "va", "nvjpeg|software")
 * @param options Codec options (currently unused for JPEG)
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no decoder is available
 */

inline GstElement *createJpegDecoder( const std::string &name, uint32_t desired = CODEC_FLAG_AUTO,
                                      const CodecOptions &options = {} )
{
  (void)options;
  const auto &desc = jpegDecoderDescriptor();

  return codecCreate( desc, desired, [&desc, &name]( uint32_t selected ) -> GstElement * {
    for ( size_t i = 0; i < desc.backend_count; ++i ) {
      if ( desc.backends[i].flag == selected ) {
        GstElement *decoder = gst_element_factory_make( desc.backends[i].factory, name.c_str() );
        if ( !decoder )
          return nullptr;
        SERVER_LOG_INFO( "Created JPEG decoder: %s", desc.backends[i].name );
        return createDecoderBin( decoder, ( name + "_bin" ).c_str() );
      }
    }
    return nullptr;
  } );
}

/**
 * @brief Create a JPEG decoder with automatic hardware fallback.
 *
 * Tries hardware decoders first (VA > NVJPEG > VAAPI), falling back to software.
 * If a decoder fails to create, it automatically tries the next available option.
 *
 * @param decoder Decoder preference string (e.g., "auto", "va", "nvjpeg|software")
 * @param options Codec options (currently unused for JPEG)
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no decoder is available
 */
inline GstElement *createJpegDecoder( const std::string &name, const std::string &decoder,
                                      const CodecOptions &options = {} )
{
  const auto &desc = jpegDecoderDescriptor();
  uint32_t desired = codecFromString( desc, decoder );
  return createJpegDecoder( name, desired, options );
}

// ============================================================================
// JPEG Encoder
// ============================================================================

enum class JpegEncoder : uint32_t {
  NONE = 0x00,
  VA = 0x01,
  VAAPI = 0x02,
  NVJPEG = 0x04,
  MPP = 0x08,
  SOFTWARE = 0x8000,
  AUTO = CODEC_FLAG_AUTO
};

CODEC_BITFLAG_OPS( JpegEncoder )

// Priority order: VA > NVJPEG > VAAPI > SOFTWARE
constexpr CodecBackend jpeg_encoder_backends[] = {
    { 0x01, "VA", "vajpegenc" },       { 0x04, "NVJPEG", "nvjpegenc" },
    { 0x08, "MPP", "mppjpegenc" },     { 0x02, "VAAPI", "vaapijpegenc" },
    { 0x8000, "SOFTWARE", "jpegenc" },
};

constexpr CodecAlias jpeg_encoder_aliases[] = {
    { "NVIDIA", static_cast<uint32_t>( JpegEncoder::NVJPEG ) },
    { "SW", static_cast<uint32_t>( JpegEncoder::SOFTWARE ) },
};

inline const CodecDescriptor &jpegEncoderDescriptor()
{
  static const CodecDescriptor desc{
      "JPEG",
      "encoder",
      jpeg_encoder_backends,
      std::size( jpeg_encoder_backends ),
      jpeg_encoder_aliases,
      std::size( jpeg_encoder_aliases ),
  };
  return desc;
}

/**
 * @brief Create a JPEG encoder with automatic hardware fallback.
 *
 * Tries hardware encoders first (VA > NVJPEG > VAAPI), falling back to software.
 * If an encoder fails to create, it automatically tries the next available option.
 *
 * @param encoder Encoder preference string (e.g., "auto", "va", "nvjpeg|software")
 * @param options Codec options (currently unused for JPEG)
 * @return GstElement* The encoder element
 * @throws PipelineBuildError if no encoder is available
 */
inline GstElement *createJpegEncoder( const std::string &encoder, const std::string &name,
                                      const CodecOptions &options = {} )
{
  (void)options;
  const auto &desc = jpegEncoderDescriptor();
  uint32_t desired = codecFromString( desc, encoder );

  return codecCreate( desc, desired, [&desc, &name]( uint32_t selected ) -> GstElement * {
    for ( size_t i = 0; i < desc.backend_count; ++i ) {
      if ( desc.backends[i].flag == selected ) {
        return gst_element_factory_make( desc.backends[i].factory, name.c_str() );
      }
    }
    return nullptr;
  } );
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_JPEG_HPP
