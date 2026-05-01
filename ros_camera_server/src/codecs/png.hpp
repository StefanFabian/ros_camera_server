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

#ifndef ROS_CAMERA_SERVER_PNG_HPP
#define ROS_CAMERA_SERVER_PNG_HPP

#include "codec_common.hpp"

namespace ros_camera_server
{

// ============================================================================
// PNG Decoder
// ============================================================================

enum class PngDecoder : uint32_t { NONE = 0x00, SOFTWARE = 0x8000, AUTO = CODEC_FLAG_AUTO };

CODEC_BITFLAG_OPS( PngDecoder )

constexpr CodecBackend png_decoder_backends[] = {
    { 0x8000, "SOFTWARE", "pngdec" },
};

constexpr CodecAlias png_decoder_aliases[] = {
    { "SW", 0x8000 },
};

inline const CodecDescriptor &pngDecoderDescriptor()
{
  static const CodecDescriptor desc = {
      "PNG",
      "decoder",
      png_decoder_backends,
      std::size( png_decoder_backends ),
      png_decoder_aliases,
      std::size( png_decoder_aliases ),
  };
  return desc;
}

/**
 * @brief Create a PNG decoder with automatic fallback.
 *
 * PNG decoding is software-only (pngdec).
 *
 * @param decoder Decoder preference string (e.g., "auto", "software", "sw")
 * @param options Codec options for future extensibility
 * @return GstElement* The decoder element
 * @throws PipelineBuildError if no decoder is available
 */
inline GstElement *createPngDecoder( const std::string &name, const std::string &decoder = "auto",
                                     const CodecOptions &options = {} )
{
  (void)options;
  const auto &desc = pngDecoderDescriptor();
  uint32_t desired = codecFromString( desc, decoder );
  return codecCreate( desc, desired, [&desc, &name]( uint32_t selected ) -> GstElement * {
    for ( size_t i = 0; i < desc.backend_count; ++i ) {
      if ( desc.backends[i].flag == selected ) {
        GstElement *decoder = gst_element_factory_make( desc.backends[i].factory, name.c_str() );
        if ( !decoder )
          return nullptr;
        return createDecoderBin( decoder, ( name + "_bin" ).c_str() );
      }
    }
    return nullptr;
  } );
}

// ============================================================================
// PNG Encoder
// ============================================================================

enum class PngEncoder : uint32_t { NONE = 0x00, SOFTWARE = 0x8000, AUTO = CODEC_FLAG_AUTO };

CODEC_BITFLAG_OPS( PngEncoder )

constexpr CodecBackend png_encoder_backends[] = {
    { 0x8000, "SOFTWARE", "pngenc" },
};

constexpr CodecAlias png_encoder_aliases[] = {
    { "SW", 0x8000 },
};

inline const CodecDescriptor &pngEncoderDescriptor()
{
  static const CodecDescriptor desc = {
      "PNG",
      "encoder",
      png_encoder_backends,
      std::size( png_encoder_backends ),
      png_encoder_aliases,
      std::size( png_encoder_aliases ),
  };
  return desc;
}

/**
 * @brief Create a PNG encoder with automatic fallback.
 *
 * PNG encoding is software-only (pngenc).
 *
 * @param encoder Encoder preference string (e.g., "auto", "software", "sw")
 * @param options Codec options for future extensibility
 * @return GstElement* The encoder element
 * @throws PipelineBuildError if no encoder is available
 */
inline GstElement *createPngEncoder( const std::string &encoder, const std::string &name,
                                     const CodecOptions &options = {} )
{
  (void)options;
  const auto &desc = pngEncoderDescriptor();
  uint32_t desired = codecFromString( desc, encoder );
  return codecCreate( desc, desired, [&desc, &name]( uint32_t selected ) -> GstElement * {
    for ( size_t i = 0; i < desc.backend_count; ++i ) {
      if ( desc.backends[i].flag == selected ) {
        GstElement *encoder = gst_element_factory_make( desc.backends[i].factory, "encoder" );
        if ( !encoder )
          return nullptr;
        return createImageEncoderBin( encoder, name );
      }
    }
    return nullptr;
  } );
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PNG_HPP
