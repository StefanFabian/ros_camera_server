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

#include "camera_pipeline.hpp"
#include "logging.hpp"
#include "pipeline/pipeline_builder.hpp"
#include "pipeline/pipeline_graph.hpp"
#include "pipeline_monitor.hpp"
#include "ros_camera_server/exceptions.hpp"
#include "ros_camera_server/helpers/reference_timestamp_helpers.hpp"

#include <chrono>
#include <gst/gst.h>

namespace ros_camera_server
{

CameraPipeline::CameraPipeline( rclcpp::Node::SharedPtr node, CameraConfiguration configuration,
                                std::shared_ptr<PipelineMonitorInterface> monitor )
    : node_( std::move( node ) ), monitor_( std::move( monitor ) ),
      configuration_( std::move( configuration ) )
{
  if ( node_ == nullptr ) {
    throw std::invalid_argument( "CameraPipeline requires a valid rclcpp::Node shared pointer" );
  }
  if ( monitor_ == nullptr ) {
    throw std::invalid_argument( "CameraPipeline requires a valid PipelineMonitor" );
  }
}

CameraPipeline::~CameraPipeline()
{
  // Release any request pads we obtained from tees so their internal state is
  // properly torn down before the parent pipeline (and the tees with it) is
  // unreffed.
  for ( auto &[tee, pad] : tee_request_pads_ ) {
    if ( tee && pad )
      gst_element_release_request_pad( tee, pad );
  }
  tee_request_pads_.clear();
}

std::string CameraPipeline::id() const { return configuration_.id; }

std::string CameraPipeline::name() const { return configuration_.name; }

bool CameraPipeline::isBuilt() const { return input_.bin != nullptr; }

std::chrono::milliseconds CameraPipeline::uptime() const
{
  if ( start_time_ == clock::time_point() ) {
    return std::chrono::milliseconds( 0 );
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>( clock::now() - start_time_ );
}

void CameraPipeline::buildPipeline()
{
  if ( input_.bin != nullptr ) {
    SERVER_LOG_ERROR_STREAM( "Tried to build pipeline twice for camera: " << configuration_.id );
    return;
  }
  SERVER_LOG_INFO_STREAM( "Creating pipeline for camera: " + configuration_.id );
  pipeline_ = GST_BIN( gst_pipeline_new( configuration_.id.c_str() ) );
  if ( pipeline_ == nullptr )
    throw PipelineBuildError( "Failed to create pipeline for camera: " + configuration_.id );

  // 1. Create Input
  input_ = configuration_.input->createInput( node_, configuration_.id );
  if ( !input_.bin )
    throw PipelineBuildError( "Failed to create input bin for camera: " + configuration_.id );
  gst_bin_add( pipeline_.get(), GST_ELEMENT( input_.bin ) );

  // Probe for timestamps
  GstPad *input_src_pad = gst_element_get_static_pad( GST_ELEMENT( input_.bin ), "src" );
  input_probe_ =
      gst_pad_add_probe( input_src_pad,
                         GstPadProbeType( GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST |
                                          GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM ),
                         inputBufferCallback, this, nullptr );
  gst_object_unref( input_src_pad );

  // 2. Build pipeline graph
  PipelineGraph graph = PipelineGraph::build( input_.format, configuration_.input->type,
                                              configuration_.outputs, configuration_.input->decoder );
  SERVER_LOG_INFO_STREAM( "Camera '" << configuration_.id << "' " << graph.toString() );

  // 3. Resolve actual codec for each output (replaces "auto" with actual transport type)
  for ( NodeId sink_id : graph.sink_nodes ) {
    const SinkConfig &sink_config = std::get<SinkConfig>( graph.getNode( sink_id ).config );
    StreamFormat resolved = graph.resolveOutputFormat( sink_id, input_.format );
    configuration_.outputs[sink_config.output_index]->onStreamFormatSelected( resolved );
  }

  // 4. Realize the graph with GStreamer elements and create outputs
  PipelineBuilder builder( pipeline_.get(), node_ );
  auto build_result =
      builder.realize( graph, GST_ELEMENT( input_.bin ), configuration_.outputs, configuration_.id );
  outputs_ = std::move( build_result.outputs );
  flow_controller_ = FlowController( std::move( build_result.flow_control ), outputs_ );
  tee_request_pads_ = std::move( build_result.tee_request_pads );

  // 5. Setup bus watch
  GstBus *bus = gst_element_get_bus( GST_ELEMENT( pipeline_.get() ) );
  gst_bus_add_watch( bus, onBusMessage, this );
  gst_object_unref( bus );
}

GstState CameraPipeline::getState() const
{
  GstState state;
  gst_element_get_state( GST_ELEMENT( pipeline_.get() ), &state, nullptr, 0 );
  return state;
}

void CameraPipeline::start()
{
  GstStateChangeReturn result =
      gst_element_set_state( GST_ELEMENT( pipeline_.get() ), GST_STATE_PLAYING );
  if ( result != GST_STATE_CHANGE_SUCCESS && result != GST_STATE_CHANGE_ASYNC ) {
    SERVER_LOG_ERROR(
        "Failed to set pipeline to READY state for camera: %s.\nPipeline state change result: %s",
        configuration_.id.c_str(), gst_element_state_change_return_get_name( result ) );
    return;
  }
  start_time_ = clock::now();
}

void CameraPipeline::stop()
{
  gst_element_set_state( GST_ELEMENT( pipeline_.get() ), GST_STATE_NULL );
  start_time_ = clock::time_point();
  // Invalidate immediately so an early buffer of the next session can't use the previous
  // session's base_time before the (async) PLAYING bus message refreshes it.
  pipeline_base_time_.store( GST_CLOCK_TIME_NONE );
}

void CameraPipeline::restart()
{
  stop();

  // Wait for up to 2 seconds for state to change to NULL to prevent async race conditions
  GstState state, pending;
  GstStateChangeReturn ret =
      gst_element_get_state( GST_ELEMENT( pipeline_.get() ), &state, &pending, 2000000000 );
  if ( ret == GST_STATE_CHANGE_ASYNC ) {
    SERVER_LOG_WARN_STREAM( "Pipeline '"
                            << configuration_.id
                            << "' state change to NULL did not complete within 2 seconds." );
  }

  current_segment_ = std::nullopt;
  start();
}

gboolean CameraPipeline::onBusMessage( GstBus *, GstMessage *msg, gpointer user_data )
{
  auto self = static_cast<CameraPipeline *>( user_data );
  GError *err = nullptr;
  gchar *debug_info = nullptr;
  switch ( GST_MESSAGE_TYPE( msg ) ) {
  case GST_MESSAGE_EOS:
    SERVER_LOG_INFO_STREAM( "End of stream reached for pipeline: " << self->configuration_.id );
    break;
  case GST_MESSAGE_STATE_CHANGED:
    if ( GstObject *src = GST_MESSAGE_SRC( msg ); !GST_IS_PIPELINE( src ) )
      return TRUE;
    GstState old_state, new_state, pending_state;
    gst_message_parse_state_changed( msg, &old_state, &new_state, &pending_state );
    // base_time is fixed for the lifetime of a PLAYING session and changes on each PLAYING transition
    if ( new_state == GST_STATE_PLAYING )
      self->pipeline_base_time_.store(
          gst_element_get_base_time( GST_ELEMENT( self->pipeline_.get() ) ) );
    else
      self->pipeline_base_time_.store( GST_CLOCK_TIME_NONE );
    break;
  case GST_MESSAGE_ERROR:
    gst_message_parse_error( msg, &err, &debug_info );
    self->monitor_->observeBusError( self->configuration_.id, GST_MESSAGE_SRC( msg ), err,
                                     debug_info );
    break;
  case GST_MESSAGE_WARNING:
    gst_message_parse_warning( msg, &err, &debug_info );
    self->monitor_->observeBusWarning( self->configuration_.id, GST_MESSAGE_SRC( msg ), err,
                                       debug_info );
    break;
  default:
    // Ignore other messages
    break;
  }
  if ( debug_info )
    g_free( debug_info );
  if ( err )
    g_error_free( err );
  return TRUE;
}

GstPadProbeReturn CameraPipeline::inputBufferCallback( GstPad *pad, GstPadProbeInfo *info,
                                                       gpointer user_data )
{
  auto *self = static_cast<CameraPipeline *>( user_data );
  if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM ) {
    GstEvent *event = GST_PAD_PROBE_INFO_EVENT( info );
    if ( GST_EVENT_TYPE( event ) == GST_EVENT_SEGMENT ) {
      if ( !self->current_segment_ ) {
        self->current_segment_ = GstSegment();
      }
      gst_event_copy_segment( event, &self->current_segment_.value() );
    }
    return GST_PAD_PROBE_OK;
  }
  if ( !self->current_segment_ ) {
    // No segment yet, cannot process timestamps
    return GST_PAD_PROBE_OK;
  }
  const guint64 ingress_time_ns = self->node_->now().nanoseconds();
  const clock::time_point now = clock::now();

  // current_segment_ can be reset to nullopt by restart() on the executor thread; copy it by value
  const GstSegment segment = self->current_segment_.value();

  // Pipeline clock + base_time are PLAYING-session lifetime state, sampled outside the hot path:
  // the clock ref in useClock() and base_time on the PLAYING bus message. Read them once here;
  // gst_clock_get_time on the monotonic system clock is a lock-free clock_gettime, so no
  // per-buffer GST_OBJECT_LOCK is needed. Both are GST_CLOCK_TIME_NONE until the pipeline plays.
  const GstClockTime base_time = self->pipeline_base_time_.load();
  const GstClockTime pipeline_now = self->pipeline_clock_.get()
                                        ? gst_clock_get_time( self->pipeline_clock_.get() )
                                        : GST_CLOCK_TIME_NONE;

  // Convert a buffer PTS into a UNIX-epoch capture timestamp. PTS is in the segment's
  // coordinate system; running_time = gst_segment_to_running_time(segment, pts) and the
  // capture instant on the pipeline clock is base_time + running_time. For sources that
  // stamp at capture (e.g. v4l2src with kernel timestamps) this is closer to the true
  // capture time than sampling wall-clock now. Falls back to ingress_time_ns when the
  // value is implausible (not playing yet, PTS unset or outside the segment, arithmetic
  // overflow, future timestamp, or > 10s in the past).
  auto deriveCaptureTime = [&]( GstClockTime pts ) -> guint64 {
    constexpr guint64 kMaxAgeNs = 10'000'000'000ULL; // 10 second sanity threshold
    if ( pts == GST_CLOCK_TIME_NONE )
      return ingress_time_ns;
    if ( pipeline_now == GST_CLOCK_TIME_NONE || base_time == GST_CLOCK_TIME_NONE )
      return ingress_time_ns; // Pipeline not playing yet
    GstClockTime running_time = gst_segment_to_running_time( &segment, GST_FORMAT_TIME, pts );
    if ( running_time == GST_CLOCK_TIME_NONE ) {
      // PTS never maps into the segment -> the feature silently degrades to ingress time for every
      // frame; warn (throttled) so the misconfiguration is visible instead of failing quietly.
      SERVER_LOG_WARN_THROTTLE(
          *self->node_->get_clock(), 20000,
          "Camera '%s': input PTS is outside the current segment; capture timestamps fall back to "
          "server ingress time. This message is throttled to once every 20s.",
          self->configuration_.id.c_str() );
      return ingress_time_ns;
    }
    GstClockTime pipeline_capture = base_time + running_time;
    // Reject unsigned overflow and capture instants in the future.
    if ( pipeline_capture < base_time || pipeline_capture > pipeline_now )
      return ingress_time_ns;
    GstClockTime age_ns = pipeline_now - pipeline_capture;
    if ( age_ns > kMaxAgeNs ) {
      SERVER_LOG_WARN_THROTTLE(
          *self->node_->get_clock(), 20000,
          "Camera '%s': derived capture time is more than 10s in the past; capture timestamps fall "
          "back to server ingress time. This message is throttled to once every 20s.",
          self->configuration_.id.c_str() );
      return ingress_time_ns;
    }
    return ingress_time_ns - age_ns;
  };

  // Stamp a single buffer: add server ingress meta to every buffer, and a PTS-derived
  // capture timestamp to any buffer that doesn't already carry one (e.g. from an
  // upstream ROS node). May replace *buf when making it writable.
  auto stampBuffer = [&]( GstBuffer **buf ) {
    bool needs_unix_meta = !buffer_get_unix_timestamp_meta( *buf );
    *buf = gst_buffer_make_writable( *buf );
    if ( needs_unix_meta )
      buffer_add_unix_timestamp_meta( *buf, deriveCaptureTime( GST_BUFFER_PTS( *buf ) ) );
    buffer_add_server_ingress_meta( *buf, ingress_time_ns );
  };

  bool processed = false;
  if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER( info );
    stampBuffer( &buffer );
    GST_PAD_PROBE_INFO_DATA( info ) = buffer;
    processed = true;
  } else if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER_LIST ) {
    GstBufferList *buffer_list = GST_PAD_PROBE_INFO_BUFFER_LIST( info );
    buffer_list = gst_buffer_list_make_writable( buffer_list );
    gst_buffer_list_foreach(
        buffer_list,
        []( GstBuffer **buf, guint /*idx*/, gpointer user_data ) -> gboolean {
          ( *static_cast<decltype( stampBuffer ) *>( user_data ) )( buf );
          return TRUE;
        },
        &stampBuffer );
    GST_PAD_PROBE_INFO_DATA( info ) = buffer_list;
    processed = true;
  }
  if ( processed ) {
    std::lock_guard lock( self->input_timestamps_mutex_ );
    self->input_buffer_timestamps_.push( now );
  }
  return GST_PAD_PROBE_OK;
}

PipelineStatistics CameraPipeline::statistics() const
{
  using namespace std::chrono;
  PipelineStatistics stats;
  auto now = clock::now();
  {
    std::lock_guard lock( input_timestamps_mutex_ );
    auto dt = duration_cast<milliseconds>( now - input_buffer_timestamps_.front() ).count();
    while ( !input_buffer_timestamps_.empty() &&
            duration_cast<milliseconds>( now - input_buffer_timestamps_.front() ).count() > 3000 ) {
      input_buffer_timestamps_.pop_front();
    }
    if ( input_buffer_timestamps_.empty() )
      stats.input_fps = 0.0;
    else
      stats.input_fps = input_buffer_timestamps_.size() / ( dt / 1000.0f );
  }
  for ( const auto &output : outputs_ ) {
    stats.output_statistics.push_back( output->statistics() );
  }
  return stats;
}

void CameraPipeline::useClock( GstClock *clock )
{
  if ( !pipeline_ ) {
    throw std::runtime_error( "Pipeline not built yet" );
  }
  gst_pipeline_use_clock( GST_PIPELINE( pipeline_.get() ), clock );
  // Keep our own ref so inputBufferCallback can read the clock without GST_ELEMENT_CLOCK + lock.
  pipeline_clock_ = clock == nullptr ? nullptr : GST_CLOCK( gst_object_ref( clock ) );
}

void CameraPipeline::updateFlowControl() { flow_controller_.update(); }

bool CameraPipeline::isOutputFlowing( size_t output_index ) const
{ return flow_controller_.isOutputFlowing( output_index ); }

} // namespace ros_camera_server
