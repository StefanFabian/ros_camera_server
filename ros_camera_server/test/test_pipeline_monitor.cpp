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

#include "diagnostic_context.hpp"
#include "pipeline_monitor.hpp"

#include <gst/gst.h>
#include <gtest/gtest.h>

namespace ros_camera_server
{

namespace
{

struct EmittedRecord {
  GstDebugLevel level;
  std::string message;
};

class RecordingMonitor : public PipelineMonitor
{
public:
  std::vector<EmittedRecord> emitted;

protected:
  void emit( GstDebugLevel level, const std::string &message ) override
  { emitted.push_back( { level, message } ); }
};

class PipelineMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if ( !gst_is_initialized() )
      gst_init( nullptr, nullptr );
  }

  static GError *makeError( const char *text )
  { return g_error_new( g_quark_from_static_string( "test-domain" ), 1, "%s", text ); }
};

} // namespace

TEST_F( PipelineMonitorTest, BusErrorEmittedWithCameraId )
{
  RecordingMonitor monitor;
  GError *err = makeError( "boom" );
  GstElement *element = gst_element_factory_make( "fakesrc", "src" );
  ASSERT_NE( element, nullptr );

  monitor.observeBusError( "cam_a", GST_OBJECT( element ), err, "stack info" );
  monitor.processLogMessages();

  ASSERT_EQ( monitor.emitted.size(), 1u );
  EXPECT_EQ( monitor.emitted[0].level, GST_LEVEL_ERROR );
  EXPECT_NE( monitor.emitted[0].message.find( "cam_a" ), std::string::npos );
  EXPECT_NE( monitor.emitted[0].message.find( "boom" ), std::string::npos );
  EXPECT_NE( monitor.emitted[0].message.find( "stack info" ), std::string::npos );

  g_error_free( err );
  gst_object_unref( element );
}

TEST_F( PipelineMonitorTest, RepeatedRecordsAreCoalesced )
{
  RecordingMonitor monitor;

  for ( int i = 0; i < 5; ++i ) {
    GError *err = makeError( "repeat" );
    monitor.observeBusWarning( "cam_a", nullptr, err, nullptr );
    g_error_free( err );
  }
  monitor.processLogMessages();

  // Only the first should have been emitted; the next 4 are within REPEAT_LOG_INTERVAL.
  ASSERT_EQ( monitor.emitted.size(), 1u );
  EXPECT_EQ( monitor.emitted[0].level, GST_LEVEL_WARNING );
  EXPECT_EQ( monitor.emitted[0].message.find( "suppressed" ), std::string::npos );
}

TEST_F( PipelineMonitorTest, DistinctRecordsEmittedSeparately )
{
  RecordingMonitor monitor;

  GError *e1 = makeError( "alpha" );
  GError *e2 = makeError( "beta" );
  monitor.observeBusWarning( "cam_a", nullptr, e1, nullptr );
  monitor.observeBusWarning( "cam_a", nullptr, e2, nullptr );
  g_error_free( e1 );
  g_error_free( e2 );

  monitor.processLogMessages();
  ASSERT_EQ( monitor.emitted.size(), 2u );
}

TEST_F( PipelineMonitorTest, RingDropsOldestWhenOverflowing )
{
  RecordingMonitor monitor;

  // Push more than MAX_PENDING_RECORDS (256) to force ring drop-oldest.
  constexpr int kPushes = 400;
  for ( int i = 0; i < kPushes; ++i ) {
    GError *err = makeError( ( "msg_" + std::to_string( i ) ).c_str() );
    monitor.observeBusWarning( "cam_overflow", nullptr, err, nullptr );
    g_error_free( err );
  }
  monitor.processLogMessages();

  // Fewer than kPushes survive (plus the one dropped-records summary).
  ASSERT_FALSE( monitor.emitted.empty() );
  EXPECT_LT( monitor.emitted.size(), static_cast<size_t>( kPushes ) );
  // The newest record is kept (drop-oldest, not drop-newest).
  bool sawNewest = false;
  for ( const auto &rec : monitor.emitted ) {
    if ( rec.message.find( "msg_399" ) != std::string::npos ) {
      sawNewest = true;
      break;
    }
  }
  EXPECT_TRUE( sawNewest );
}

TEST_F( PipelineMonitorTest, WeakRefSurvivesSourceObjectDestruction )
{
  RecordingMonitor monitor;

  GstElement *element = gst_element_factory_make( "fakesrc", "transient" );
  ASSERT_NE( element, nullptr );
  GError *err = makeError( "bye" );
  monitor.observeBusError( "cam_x", GST_OBJECT( element ), err, nullptr );
  g_error_free( err );

  // Drop the only ref. The weak ref inside the queued record should now resolve to null without
  // crashing during processLogMessages().
  gst_object_unref( element );

  EXPECT_NO_THROW( monitor.processLogMessages() );
  ASSERT_EQ( monitor.emitted.size(), 1u );
  EXPECT_EQ( monitor.emitted[0].level, GST_LEVEL_ERROR );
}

TEST_F( PipelineMonitorTest, BusMessageNotDroppedWithoutDebugLogContext )
{
  // Bus messages are never dropped as duplicates, even when their text begins with "warning: " /
  // "error: " (which is the marker the dedup logic uses for debug-log replays).
  RecordingMonitor monitor;
  GError *err = makeError( "warning: bogus" );
  monitor.observeBusWarning( "cam_a", nullptr, err, nullptr );
  g_error_free( err );

  monitor.processLogMessages();
  ASSERT_EQ( monitor.emitted.size(), 1u );
}

TEST_F( PipelineMonitorTest, RtpJpegPayInvalidComponentIsRewritten )
{
  RecordingMonitor monitor;
  GError *err = makeError( "Invalid component" );
  monitor.observeBusError(
      "cam_a", nullptr, err,
      "../gst/rtp/gstrtpjpegpay.c(627): gst_rtp_jpeg_pay_read_sof (): "
      "/GstPipeline:cam_a/GstBin:rtp_output_bin_0/GstRtpJPEGPay:rtp_output_bin_0_pay" );
  g_error_free( err );

  monitor.processLogMessages();
  ASSERT_EQ( monitor.emitted.size(), 1u );
  EXPECT_NE( monitor.emitted[0].message.find( "rtpjpegpay" ), std::string::npos );
  EXPECT_NE( monitor.emitted[0].message.find( "4:2:0" ), std::string::npos );
  // The opaque original message must not appear as the surfaced cause.
  EXPECT_EQ( monitor.emitted[0].message.find( ": Invalid component " ), std::string::npos );
}

TEST_F( PipelineMonitorTest, InvalidComponentFromUnrelatedSourceIsNotRewritten )
{
  RecordingMonitor monitor;
  GError *err = makeError( "Invalid component" );
  monitor.observeBusError( "cam_a", nullptr, err, "some/other/element.c(42): foo_bar ()" );
  g_error_free( err );

  monitor.processLogMessages();
  ASSERT_EQ( monitor.emitted.size(), 1u );
  EXPECT_NE( monitor.emitted[0].message.find( "Invalid component" ), std::string::npos );
  EXPECT_EQ( monitor.emitted[0].message.find( "rtpjpegpay" ), std::string::npos );
}

TEST_F( PipelineMonitorTest, CoalescedSizeBoundedAcrossManyDistinctMessages )
{
  RecordingMonitor monitor;

  // Push more distinct messages than MAX_COALESCED_ENTRIES (128) to force evictions.
  constexpr int kDistinct = 200;
  for ( int i = 0; i < kDistinct; ++i ) {
    GError *err = makeError( ( "u_" + std::to_string( i ) ).c_str() );
    monitor.observeBusWarning( "cam_a", nullptr, err, nullptr );
    g_error_free( err );
    monitor.processLogMessages();
  }
  // Should not crash, and each distinct message should have been emitted once.
  EXPECT_EQ( monitor.emitted.size(), static_cast<size_t>( kDistinct ) );
}

} // namespace ros_camera_server
