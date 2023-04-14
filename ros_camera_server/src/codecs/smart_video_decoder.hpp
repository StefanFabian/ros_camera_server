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

#ifndef ROS_CAMERA_SERVER_SMART_VIDEO_DECODER_HPP
#define ROS_CAMERA_SERVER_SMART_VIDEO_DECODER_HPP

#include "codec_common.hpp"

#include <atomic>
#include <gst/gst.h>
#include <mutex>

namespace ros_camera_server
{

struct SmartDecoderData {
  const CodecDescriptor *desc;
  uint32_t desired_flags;
  uint32_t excluded_flags = 0;

  GstElement *bin = nullptr;
  GstElement *queue = nullptr;
  GstElement *decoder = nullptr;
  GstElement *convert = nullptr;
  GstElement *pending_decoder = nullptr;

  std::atomic<uint64_t> input_count{ 0 };
  std::atomic<uint64_t> output_count{ 0 };
  std::atomic<bool> swap_pending{ false };
  bool exhausted = false;

  std::mutex swap_mutex;

  static constexpr uint64_t CHECK_THRESHOLD = 20;
};

namespace detail
{

inline const CodecBackend *findBackend( const CodecDescriptor &desc, uint32_t flag )
{
  for ( size_t i = 0; i < desc.backend_count; ++i ) {
    if ( desc.backends[i].flag == flag )
      return &desc.backends[i];
  }
  return nullptr;
}

inline uint32_t identifyCurrentDecoder( SmartDecoderData *data )
{
  GstElementFactory *factory = gst_element_get_factory( data->decoder );
  if ( !factory )
    return 0;
  const char *name = gst_plugin_feature_get_name( GST_PLUGIN_FEATURE( factory ) );
  for ( size_t i = 0; i < data->desc->backend_count; ++i ) {
    if ( g_strcmp0( data->desc->backends[i].factory, name ) == 0 )
      return data->desc->backends[i].flag;
  }
  return 0;
}

inline void smartDecoderSwap( SmartDecoderData *data )
{
  std::lock_guard<std::mutex> lock( data->swap_mutex );
  if ( data->exhausted )
    return;

  // Exclude the current (failing) decoder
  uint32_t current = identifyCurrentDecoder( data );
  if ( current != 0 )
    data->excluded_flags |= current;

  // Find next working backend
  GstElement *new_decoder = nullptr;
  const CodecBackend *backend = nullptr;
  while ( true ) {
    uint32_t next = codecSelect( *data->desc, data->desired_flags, data->excluded_flags );
    if ( next == 0 ) {
      SERVER_LOG_ERROR( "SmartVideoDecoder: All decoder backends exhausted." );
      data->exhausted = true;
      data->swap_pending = false;
      GstElement *bin = data->bin;
      std::string msg = std::string( "All " ) + data->desc->codec_name +
                        " decoder backends exhausted. No decoder produced output.";
      gst_element_message_full( bin, GST_MESSAGE_ERROR, GST_STREAM_ERROR, GST_STREAM_ERROR_DECODE,
                                g_strdup( msg.c_str() ), nullptr, __FILE__, GST_FUNCTION, __LINE__ );
      return;
    }
    backend = findBackend( *data->desc, next );
    new_decoder = gst_element_factory_make( backend->factory, "smart_internal_decoder" );
    if ( new_decoder )
      break;
    data->excluded_flags |= next;
  }

  SERVER_LOG_WARN( "SmartVideoDecoder: Decoder not producing output, switching to %s", backend->name );

  data->pending_decoder = new_decoder;

  // Block the queue's src pad to safely swap the decoder element
  GstPad *queue_src = gst_element_get_static_pad( data->queue, "src" );
  gst_pad_add_probe(
      queue_src, GST_PAD_PROBE_TYPE_BLOCK_DOWNSTREAM,
      []( GstPad *, GstPadProbeInfo *, gpointer user_data ) -> GstPadProbeReturn {
        auto *d = static_cast<SmartDecoderData *>( user_data );

        // Remove old decoder
        gst_element_unlink( d->queue, d->decoder );
        gst_element_unlink( d->decoder, d->convert );
        gst_element_set_state( d->decoder, GST_STATE_NULL );
        gst_bin_remove( GST_BIN( d->bin ), d->decoder );

        // Install new decoder
        d->decoder = d->pending_decoder;
        d->pending_decoder = nullptr;
        gst_bin_add( GST_BIN( d->bin ), d->decoder );
        gst_element_link_many( d->queue, d->decoder, d->convert, nullptr );
        gst_element_sync_state_with_parent( d->decoder );

        // Reset monitoring
        d->input_count = 0;
        d->output_count = 0;
        d->swap_pending = false;

        return GST_PAD_PROBE_REMOVE;
      },
      data, nullptr );
  gst_object_unref( queue_src );
}

} // namespace detail

/**
 * @brief Create a decoder bin that monitors output and falls back to alternative backends.
 *
 * The bin structure is: [ghost sink] -> queue -> decoder -> videoconvert -> [ghost src]
 *
 * Pad probes monitor input/output frame counts. If the decoder produces no output after
 * CHECK_THRESHOLD input frames, the decoder element is hot-swapped for the next available
 * backend via a blocking pad probe.
 *
 * @param desc The codec descriptor with backend priority list
 * @param desired Bitflags of desired backends (default: AUTO tries all in priority order)
 * @return GstElement* A bin with static "sink" and "src" ghost pads
 * @throws PipelineBuildError if no decoder backend is available
 */
inline GstElement *createSmartVideoDecoder( const std::string &bin_name, const CodecDescriptor &desc,
                                            uint32_t desired = CODEC_FLAG_AUTO )
{
  auto *data = new SmartDecoderData();
  data->desc = &desc;
  data->desired_flags = desired;

  // Find initial decoder
  GstElement *initial = nullptr;
  const char *initial_name = nullptr;
  while ( true ) {
    uint32_t selected = codecSelect( desc, desired, data->excluded_flags );
    if ( selected == 0 ) {
      delete data;
      throw PipelineBuildError( std::string( "No suitable " ) + desc.codec_name + " decoder found." );
    }
    const CodecBackend *backend = detail::findBackend( desc, selected );
    initial = gst_element_factory_make( backend->factory, "smart_internal_decoder" );
    if ( initial ) {
      initial_name = backend->name;
      break;
    }
    data->excluded_flags |= selected;
  }

  SERVER_LOG_INFO( "SmartVideoDecoder: Initialized with %s", initial_name );

  GstElement *bin = gst_bin_new( bin_name.c_str() );
  GstElement *queue = gst_element_factory_make( "queue", "smart_decoder_queue" );
  g_object_set( queue, "max-size-buffers", 1, "leaky", 2 /* downstream */,
                nullptr ); // Keep only 1 buffer to minimize memory usage and latency
  GstElement *convert = gst_element_factory_make( "videoconvert", "smart_decoder_convert" );

  if ( !bin || !queue || !convert ) {
    if ( bin )
      gst_object_unref( bin );
    if ( queue )
      gst_object_unref( queue );
    if ( convert )
      gst_object_unref( convert );
    gst_object_unref( initial );
    delete data;
    throw PipelineBuildError( "Failed to create smart decoder bin elements" );
  }

  data->bin = bin;
  data->queue = queue;
  data->decoder = initial;
  data->convert = convert;

  // Attach data to bin for lifetime management
  g_object_set_data_full( G_OBJECT( bin ), "smart-decoder-data", data,
                          []( gpointer p ) { delete static_cast<SmartDecoderData *>( p ); } );

  gst_bin_add_many( GST_BIN( bin ), queue, initial, convert, nullptr );
  gst_element_link_many( queue, initial, convert, nullptr );

  // Ghost pads
  GstPad *q_sink = gst_element_get_static_pad( queue, "sink" );
  GstPad *c_src = gst_element_get_static_pad( convert, "src" );
  gst_element_add_pad( bin, gst_ghost_pad_new( "sink", q_sink ) );
  gst_element_add_pad( bin, gst_ghost_pad_new( "src", c_src ) );
  gst_object_unref( q_sink );
  gst_object_unref( c_src );

  // Monitoring probes
  GstPad *ghost_sink = gst_element_get_static_pad( bin, "sink" );
  gst_pad_add_probe(
      ghost_sink, GST_PAD_PROBE_TYPE_BUFFER,
      []( GstPad *, GstPadProbeInfo *, gpointer user_data ) -> GstPadProbeReturn {
        auto *d = static_cast<SmartDecoderData *>( user_data );
        uint64_t in = ++d->input_count;
        if ( in >= SmartDecoderData::CHECK_THRESHOLD && d->output_count == 0 && !d->exhausted &&
             !d->swap_pending.exchange( true ) ) {
          detail::smartDecoderSwap( d );
        }
        return GST_PAD_PROBE_OK;
      },
      data, nullptr );
  gst_object_unref( ghost_sink );

  GstPad *ghost_src = gst_element_get_static_pad( bin, "src" );
  gst_pad_add_probe(
      ghost_src, GST_PAD_PROBE_TYPE_BUFFER,
      []( GstPad *, GstPadProbeInfo *, gpointer user_data ) -> GstPadProbeReturn {
        static_cast<SmartDecoderData *>( user_data )->output_count++;
        return GST_PAD_PROBE_OK;
      },
      data, nullptr );
  gst_object_unref( ghost_src );

  return bin;
}

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_SMART_VIDEO_DECODER_HPP
