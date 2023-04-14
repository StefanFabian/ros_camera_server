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

#include "ros_camera_server/helpers/reference_timestamp_helpers.hpp"
#include "ros_camera_server/helpers/ring_buffer.hpp"

#include <chrono>

namespace ros_camera_server
{

GstCaps *unix_timestamp_reference_caps()
{
  static GstCaps *caps = nullptr;
  if ( g_once_init_enter( &caps ) ) {
    GstCaps *new_caps = gst_caps_new_empty_simple( "timestamp/x-unix" );
    g_once_init_leave( &caps, new_caps );
  }
  return caps;
}

GstReferenceTimestampMeta *buffer_add_unix_timestamp_meta( GstBuffer *buffer, GstClockTime timestamp )
{
  g_return_val_if_fail( GST_IS_BUFFER( buffer ), nullptr );
  g_return_val_if_fail( gst_buffer_is_writable( buffer ), nullptr );

  return gst_buffer_add_reference_timestamp_meta( buffer, unix_timestamp_reference_caps(),
                                                  timestamp, GST_CLOCK_TIME_NONE );
}

GstReferenceTimestampMeta *buffer_get_unix_timestamp_meta( GstBuffer *buffer )
{
  g_return_val_if_fail( GST_IS_BUFFER( buffer ), nullptr );

  return gst_buffer_get_reference_timestamp_meta( buffer, unix_timestamp_reference_caps() );
}

GstCaps *server_ingress_reference_caps()
{
  static GstCaps *caps = nullptr;
  if ( g_once_init_enter( &caps ) ) {
    GstCaps *new_caps = gst_caps_new_empty_simple( "timestamp/x-server-ingress" );
    g_once_init_leave( &caps, new_caps );
  }
  return caps;
}

GstReferenceTimestampMeta *buffer_add_server_ingress_meta( GstBuffer *buffer, GstClockTime timestamp )
{
  g_return_val_if_fail( GST_IS_BUFFER( buffer ), nullptr );
  g_return_val_if_fail( gst_buffer_is_writable( buffer ), nullptr );

  return gst_buffer_add_reference_timestamp_meta( buffer, server_ingress_reference_caps(),
                                                  timestamp, GST_CLOCK_TIME_NONE );
}

GstReferenceTimestampMeta *buffer_get_server_ingress_meta( GstBuffer *buffer )
{
  g_return_val_if_fail( GST_IS_BUFFER( buffer ), nullptr );

  return gst_buffer_get_reference_timestamp_meta( buffer, server_ingress_reference_caps() );
}

// Context for encoder passthrough probes - stores pending timestamps in a ring buffer
// This handles cases where hardware encoders strip meta and buffer multiple frames
struct EncoderPassthroughContext {
  RingBuffer<GstClockTime, 6> unix_timestamps;
  RingBuffer<GstClockTime, 6> ingress_timestamps;

  // Timing measurement
  std::shared_ptr<EncoderTimingStats> timing_stats;
  RingBuffer<std::chrono::steady_clock::time_point, 6> sink_times;

  void push( GstClockTime unix_ts, GstClockTime ingress_ts )
  {
    unix_timestamps.push( unix_ts );
    ingress_timestamps.push( ingress_ts );
  }

  bool empty() const { return unix_timestamps.empty(); }
};

static void destroy_passthrough_context( gpointer data )
{ delete static_cast<EncoderPassthroughContext *>( data ); }

static GstPadProbeReturn encoder_sink_probe( GstPad * /*pad*/, GstPadProbeInfo *info,
                                             gpointer user_data )
{
  GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
  if ( !buffer ) {
    return GST_PAD_PROBE_OK;
  }

  auto *ctx = static_cast<EncoderPassthroughContext *>( user_data );
  GstReferenceTimestampMeta *unix_meta = buffer_get_unix_timestamp_meta( buffer );
  GstReferenceTimestampMeta *ingress_meta = buffer_get_server_ingress_meta( buffer );
  ctx->push( unix_meta ? unix_meta->timestamp : GST_CLOCK_TIME_NONE,
             ingress_meta ? ingress_meta->timestamp : GST_CLOCK_TIME_NONE );

  // Measure pre-encoder time (ingress → encoder sink)
  ctx->sink_times.push( std::chrono::steady_clock::now() );
  if ( ingress_meta && ctx->timing_stats ) {
    int64_t now_ns = static_cast<int64_t>( g_get_real_time() ) * 1000;
    int64_t pre_encoder_us = ( now_ns - static_cast<int64_t>( ingress_meta->timestamp ) ) / 1000;
    if ( pre_encoder_us >= 0 ) {
      int64_t prev = ctx->timing_stats->avg_pre_encoder_us.load( std::memory_order_relaxed );
      int64_t avg = ( prev < 0 ) ? pre_encoder_us : ( prev * 7 + pre_encoder_us ) / 8;
      ctx->timing_stats->avg_pre_encoder_us.store( avg, std::memory_order_relaxed );
    }
  }
  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn encoder_src_probe( GstPad * /*pad*/, GstPadProbeInfo *info,
                                            gpointer user_data )
{
  auto *ctx = static_cast<EncoderPassthroughContext *>( user_data );
  if ( ctx->empty() ) {
    return GST_PAD_PROBE_OK;
  }

  GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
  if ( !buffer ) {
    return GST_PAD_PROBE_OK;
  }

  bool has_unix = buffer_get_unix_timestamp_meta( buffer ) != nullptr;
  bool has_ingress = buffer_get_server_ingress_meta( buffer ) != nullptr;

  GstClockTime unix_ts = ctx->unix_timestamps.front();
  ctx->unix_timestamps.pop_front();
  GstClockTime ingress_ts = ctx->ingress_timestamps.front();
  ctx->ingress_timestamps.pop_front();

  // Measure encoder time (encoder sink → encoder src)
  if ( ctx->timing_stats && !ctx->sink_times.empty() ) {
    using namespace std::chrono;
    auto sink_time = ctx->sink_times.front();
    ctx->sink_times.pop_front();
    auto encode_time = duration_cast<microseconds>( steady_clock::now() - sink_time );
    int64_t encode_us = encode_time.count();
    int64_t prev = ctx->timing_stats->avg_encode_time_us.load( std::memory_order_relaxed );
    int64_t avg = ( prev < 0 ) ? encode_us : ( prev * 7 + encode_us ) / 8;
    ctx->timing_stats->avg_encode_time_us.store( avg, std::memory_order_relaxed );
  }

  // Re-attach any metas that were stripped by the encoder
  bool need_unix = !has_unix && GST_CLOCK_TIME_IS_VALID( unix_ts );
  bool need_ingress = !has_ingress && GST_CLOCK_TIME_IS_VALID( ingress_ts );
  if ( need_unix || need_ingress ) {
    buffer = gst_buffer_make_writable( buffer );
    if ( need_unix ) {
      buffer_add_unix_timestamp_meta( buffer, unix_ts );
    }
    if ( need_ingress ) {
      buffer_add_server_ingress_meta( buffer, ingress_ts );
    }
    GST_PAD_PROBE_INFO_DATA( info ) = buffer;
  }

  return GST_PAD_PROBE_OK;
}

std::shared_ptr<EncoderTimingStats>
add_reference_timestamp_passthrough_probes( GstElement *encoder_element )
{
  if ( !encoder_element ) {
    return {};
  }

  // Find the actual encoder element (it may be wrapped in a bin)
  GstElement *encoder = encoder_element;
  if ( GST_IS_BIN( encoder_element ) ) {
    GstElement *inner_encoder = gst_bin_get_by_name( GST_BIN( encoder_element ), "encoder" );
    if ( inner_encoder ) {
      encoder = inner_encoder;
    }
  }

  // Create shared context for both probes
  auto *ctx = new EncoderPassthroughContext();
  ctx->timing_stats = std::make_shared<EncoderTimingStats>();

  // Add sink probe (owns the context, will destroy on pad removal)
  GstPad *sink_pad = gst_element_get_static_pad( encoder, "sink" );
  if ( sink_pad ) {
    gst_pad_add_probe( sink_pad, GST_PAD_PROBE_TYPE_BUFFER, encoder_sink_probe, ctx,
                       destroy_passthrough_context );
    gst_object_unref( sink_pad );
  }

  // Add src probe (shares context, no destroy callback)
  GstPad *src_pad = gst_element_get_static_pad( encoder, "src" );
  if ( src_pad ) {
    gst_pad_add_probe( src_pad, GST_PAD_PROBE_TYPE_BUFFER, encoder_src_probe, ctx, nullptr );
    gst_object_unref( src_pad );
  }

  // Unref the encoder if we got it from the bin
  if ( encoder != encoder_element && GST_IS_BIN( encoder_element ) ) {
    gst_object_unref( encoder );
  }

  return ctx->timing_stats;
}

} // namespace ros_camera_server
