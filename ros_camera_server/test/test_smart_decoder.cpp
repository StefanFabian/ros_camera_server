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

#include <gtest/gtest.h>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include "codecs/smart_video_decoder.hpp"

#include <chrono>
#include <thread>

using namespace ros_camera_server;

// Mock backends: "BROKEN" drops all buffers, "WORKING" passes them through.
// Both use the identity element; the test configures drop-probability on the first one.
constexpr CodecBackend mock_backends[] = {
    { 0x01, "BROKEN", "identity" },
    { 0x02, "WORKING", "identity" },
};

static const CodecDescriptor &mockDescriptor()
{
  static const CodecDescriptor desc{
      "MOCK", "decoder", mock_backends, std::size( mock_backends ), nullptr, 0,
  };
  return desc;
}

class SmartDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override { gst_init( nullptr, nullptr ); }
};

TEST_F( SmartDecoderTest, FallsBackWhenDecoderProducesNoOutput )
{
  GstElement *decoder_bin =
      createSmartVideoDecoder( "smart_decoder", mockDescriptor(), CODEC_FLAG_AUTO );
  ASSERT_NE( decoder_bin, nullptr );

  GstElement *pipeline = gst_pipeline_new( "test" );
  GstElement *src = gst_element_factory_make( "appsrc", "src" );
  GstElement *sink = gst_element_factory_make( "appsink", "sink" );

  GstCaps *caps = gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, "I420", "width",
                                       G_TYPE_INT, 320, "height", G_TYPE_INT, 240, "framerate",
                                       GST_TYPE_FRACTION, 30, 1, nullptr );
  g_object_set( src, "caps", caps, "format", GST_FORMAT_TIME, nullptr );
  gst_caps_unref( caps );

  // Configure the initial (BROKEN) decoder to drop all buffers
  GstElement *broken = gst_bin_get_by_name( GST_BIN( decoder_bin ), "smart_internal_decoder" );
  ASSERT_NE( broken, nullptr );
  g_object_set( broken, "drop-probability", 1.0f, nullptr );
  gst_object_unref( broken );

  gst_bin_add_many( GST_BIN( pipeline ), src, decoder_bin, sink, nullptr );
  gst_element_link_many( src, decoder_bin, sink, nullptr );
  gst_element_set_state( pipeline, GST_STATE_PLAYING );

  // Push enough buffers to trigger fallback (CHECK_THRESHOLD = 30)
  const size_t frame_size = 320 * 240 * 3 / 2; // I420
  for ( int i = 0; i < 40; ++i ) {
    GstBuffer *buf = gst_buffer_new_allocate( nullptr, frame_size, nullptr );
    GST_BUFFER_PTS( buf ) = i * 33 * GST_MSECOND;
    GST_BUFFER_DURATION( buf ) = 33 * GST_MSECOND;
    ASSERT_EQ( gst_app_src_push_buffer( GST_APP_SRC( src ), buf ), GST_FLOW_OK );
    std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
  }

  // Wait for the blocking probe to fire and complete the swap
  std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );

  // Push a buffer that should flow through the new (working) decoder
  GstBuffer *buf = gst_buffer_new_allocate( nullptr, frame_size, nullptr );
  GST_BUFFER_PTS( buf ) = 2000 * GST_MSECOND;
  GST_BUFFER_DURATION( buf ) = 33 * GST_MSECOND;
  ASSERT_EQ( gst_app_src_push_buffer( GST_APP_SRC( src ), buf ), GST_FLOW_OK );

  GstSample *sample = gst_app_sink_try_pull_sample( GST_APP_SINK( sink ), 2 * GST_SECOND );
  EXPECT_NE( sample, nullptr ) << "Expected sample after decoder fallback";
  if ( sample )
    gst_sample_unref( sample );

  // Verify internal decoder was swapped (new identity has default drop-probability=0)
  GstElement *current = gst_bin_get_by_name( GST_BIN( decoder_bin ), "smart_internal_decoder" );
  ASSERT_NE( current, nullptr );
  gfloat drop_prob = 1.0f;
  g_object_get( current, "drop-probability", &drop_prob, nullptr );
  EXPECT_FLOAT_EQ( drop_prob, 0.0f );
  gst_object_unref( current );

  gst_element_set_state( pipeline, GST_STATE_NULL );
  gst_object_unref( pipeline );
}

TEST_F( SmartDecoderTest, PassesThroughWhenDecoderWorks )
{
  GstElement *decoder_bin =
      createSmartVideoDecoder( "smart_decoder", mockDescriptor(), CODEC_FLAG_AUTO );
  ASSERT_NE( decoder_bin, nullptr );

  GstElement *pipeline = gst_pipeline_new( "test" );
  GstElement *src = gst_element_factory_make( "appsrc", "src" );
  GstElement *sink = gst_element_factory_make( "appsink", "sink" );

  GstCaps *caps = gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, "I420", "width",
                                       G_TYPE_INT, 320, "height", G_TYPE_INT, 240, "framerate",
                                       GST_TYPE_FRACTION, 30, 1, nullptr );
  g_object_set( src, "caps", caps, "format", GST_FORMAT_TIME, nullptr );
  gst_caps_unref( caps );

  // Don't modify the identity element - it passes through by default
  gst_bin_add_many( GST_BIN( pipeline ), src, decoder_bin, sink, nullptr );
  gst_element_link_many( src, decoder_bin, sink, nullptr );
  gst_element_set_state( pipeline, GST_STATE_PLAYING );

  const size_t frame_size = 320 * 240 * 3 / 2;
  for ( int i = 0; i < 40; ++i ) {
    GstBuffer *buf = gst_buffer_new_allocate( nullptr, frame_size, nullptr );
    GST_BUFFER_PTS( buf ) = i * 33 * GST_MSECOND;
    GST_BUFFER_DURATION( buf ) = 33 * GST_MSECOND;
    ASSERT_EQ( gst_app_src_push_buffer( GST_APP_SRC( src ), buf ), GST_FLOW_OK );
    std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
  }

  // Verify samples are received (decoder is working, no fallback needed)
  GstSample *sample = gst_app_sink_try_pull_sample( GST_APP_SINK( sink ), 2 * GST_SECOND );
  EXPECT_NE( sample, nullptr ) << "Expected samples when decoder works normally";
  if ( sample )
    gst_sample_unref( sample );

  // Verify no swap occurred - SmartDecoderData should show output_count > 0
  auto *data = static_cast<SmartDecoderData *>(
      g_object_get_data( G_OBJECT( decoder_bin ), "smart-decoder-data" ) );
  ASSERT_NE( data, nullptr );
  EXPECT_GT( data->output_count.load(), 0u );
  EXPECT_FALSE( data->exhausted );

  gst_element_set_state( pipeline, GST_STATE_NULL );
  gst_object_unref( pipeline );
}
