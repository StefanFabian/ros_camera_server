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

#include "ros_camera_server/camera_server.hpp"
#include "camera_pipeline.hpp"
#include "logging.hpp"
#include "pipeline_monitor.hpp"
#include "ros_camera_server/exceptions.hpp"
#include "ros_camera_server/outputs/webrtc_output.hpp"
#include "webrtc/signaling_server.hpp"

#include <gst/gstinfo.h>
#include <gst/net/gstnettimeprovider.h>

using namespace std::chrono_literals;

namespace ros_camera_server
{

void G_GNUC_NO_INSTRUMENT CameraServer::onGStreamerLog( GstDebugCategory *category,
                                                        GstDebugLevel level, const gchar *,
                                                        const gchar *, gint, GObject *object,
                                                        GstDebugMessage *message, gpointer user_data )
{
  auto *self = static_cast<CameraServer *>( user_data );
  if ( self == nullptr ) {
    return;
  }
  self->handleGStreamerLog( category, level, object, message );
}

void CameraServer::handleGStreamerLog( GstDebugCategory *category, GstDebugLevel level,
                                       GObject *object, GstDebugMessage *message )
{ pipeline_monitor_->observeGStreamerLog( category, level, object, message ); }

CameraServer::CameraServer( const rclcpp::Node::SharedPtr &node,
                            CameraServerConfiguration configuration,
                            std::function<void()> on_initialized )
    : node_( node ), configuration_( std::move( configuration ) )
{
  pipeline_monitor_ = std::make_shared<PipelineMonitor>();
  if ( !gst_is_initialized() ) {
    gst_init( nullptr, nullptr );
  }
  if ( std::getenv( "GST_DEBUG" ) == nullptr ) {
    gst_debug_set_active( TRUE );
    if ( gst_debug_get_default_threshold() < GST_LEVEL_WARNING ) {
      gst_debug_set_default_threshold( GST_LEVEL_WARNING );
    }
    // Remove default log function to prevent duplicate logging, if GST_DEBUG is not set.
    // If it is, we assume the user wants additional log output and needs the default log function.
    ( gst_debug_remove_log_function )( nullptr );
  } else {
    SERVER_LOG_WARN(
        "GST_DEBUG is set. If log threshold is higher than WARNING, this may cause some Camera "
        "Server failure detections and recovery procedures to not work properly." );
  }
  ( gst_debug_add_log_function )( CameraServer::onGStreamerLog, this, nullptr );
  gstreamer_thread_ =
      std::thread( [this, node, configuration, on_initialized = std::move( on_initialized )]() {
        GMainContext *context = g_main_context_new();
        g_main_loop_ = g_main_loop_new( context, false );
        g_main_context_push_thread_default( context );

        GstClock *system_clock = gst_system_clock_obtain();
        g_object_set( system_clock, "clock-type", GST_CLOCK_TYPE_MONOTONIC, NULL );
        clock_provider_ = GST_OBJECT(
            gst_net_time_provider_new( system_clock, "0.0.0.0", configuration.clock_port ) );

        for ( const CameraConfiguration &camera_config : configuration_.cameras ) {
          auto pipeline = std::make_unique<CameraPipeline>( node, camera_config, pipeline_monitor_ );
          try {
            pipeline->buildPipeline();
            pipeline->useClock( system_clock );
          } catch ( PipelineBuildError &e ) {
            SERVER_LOG_ERROR_STREAM( "Failed to build pipeline for camera '"
                                     << camera_config.id << "'! Error: " << e.what() );
          }
          pipelines_.push_back( std::move( pipeline ) );
        }
        // gst_system_clock_obtain() transferred a ref to us. The time provider
        // and each pipeline take their own refs; release ours.
        gst_object_unref( system_clock );
        initializeWebRTC();
        for ( auto &pipeline : pipelines_ ) { pipeline->start(); }
        initialized_.exchange( true );
        if ( on_initialized )
          on_initialized();

        g_main_loop_run( g_main_loop_ );

        g_main_loop_unref( g_main_loop_ );
        SERVER_LOG_DEBUG( "Gstreamer main loop stopped" );
        g_main_context_unref( context );
      } );

  check_and_repair_timer_ = node->create_timer( 3s, [this]() { checkAndRepair(); } );
  diagnostic_timer_ =
      node->create_timer( 500ms, [this]() { pipeline_monitor_->processLogMessages(); } );
  status_timer_ = node->create_timer( 10s, [this]() { updateStatus(); } );
  flow_control_timer_ = node->create_timer( 1s, [this]() { updateFlowControl(); } );
}

int CameraServer::signalingPort() const
{ return signaling_server_ ? signaling_server_->port() : 0; }

CameraServer::~CameraServer()
{
  check_and_repair_timer_.reset();
  diagnostic_timer_.reset();
  status_timer_.reset();
  flow_control_timer_.reset();
  SERVER_LOG_DEBUG( "Stopping gstreamer pipelines" );
  for ( auto &pipeline : pipelines_ ) { pipeline->stop(); }
  SERVER_LOG_DEBUG( "Stopping gstreamer mainloop" );
  g_main_loop_quit( g_main_loop_ );
  gstreamer_thread_.join();
  // Destroy the pipelines (and with them the WebRTC sessions and their webrtcbins) before
  // stopping the signaling server. Dropping the webrtcbins joins their ICE threads, so no late
  // transport-state callback can dispatch onto the signaling context while stop() is unreffing it.
  pipelines_.clear();
  // libsoup is not thread-safe: stop the signaling server only after the thread running its
  // context has been joined and the webrtcbin/ICE threads are gone, so nothing touches it
  // concurrently. WebSocket close frames are not delivered; clients see the TCP connection drop.
  if ( signaling_server_ ) {
    signaling_server_->stop();
    signaling_server_.reset();
  }
  gst_debug_remove_log_function_by_data( this );
  pipeline_monitor_->processLogMessages();
}

void CameraServer::updateStatus()
{
  if ( !initialized_ )
    return;
  SERVER_LOG_DEBUG( "Updating camera server status" );
  std::stringstream status_stream;
  status_stream << "Camera Server Status:\n";
  for ( const auto &pipeline : pipelines_ ) {
    PipelineStatistics stats = pipeline->statistics();
    std::string indent = "    ";
    status_stream << "- " << pipeline->id() << std::endl;
    if ( !pipeline->isBuilt() ) {
      status_stream << indent << "state: Failed to build pipeline" << std::endl;
      continue;
    }
    status_stream << indent << "state: " << gst_element_state_get_name( pipeline->getState() )
                  << std::endl;
    if ( pipeline->getState() != GST_STATE_PLAYING ) {
      continue;
    }
    status_stream << indent << "input fps: " << std::setprecision( 3 ) << stats.input_fps
                  << std::endl;
    status_stream << indent << "outputs:" << std::endl;
    indent += "  ";
    for ( size_t i = 0; i < stats.output_statistics.size(); ++i ) {
      const auto &output_stats = stats.output_statistics[i];
      const auto &output_config = pipeline->configuration().outputs[i];
      // Build video info string: codec@wxh (e.g., h264@960x540)
      std::string video_info = output_stats.codec.empty() ? "raw" : output_stats.codec;
      if ( output_stats.width > 0 && output_stats.height > 0 ) {
        video_info +=
            "@" + std::to_string( output_stats.width ) + "x" + std::to_string( output_stats.height );
      }
      // Add encoder name if available (e.g., "nvh264enc", "x264enc")
      std::string encoder_info;
      if ( !output_stats.encoder_name.empty() ) {
        encoder_info = ", encoder: " + output_stats.encoder_name;
      }
      bool flowing = pipeline->isOutputFlowing( i );
      std::string status = flowing ? "active" : "paused (no clients)";
      status_stream << indent << "- Output " << i + 1 << " (" << output_config->type << ", "
                    << video_info << encoder_info << "):" << std::endl;
      status_stream << indent << "  status: " << status << std::endl;
      if ( flowing ) {
        status_stream << indent << "  fps: " << std::setprecision( 3 ) << output_stats.fps
                      << std::endl;
        status_stream << indent << "  processing time: "
                      << static_cast<float>( output_stats.processing_time.count() / 100 ) / 10.0f
                      << "ms";
        if ( output_stats.total_processing_time.count() >= 0 &&
             output_stats.total_processing_time != output_stats.processing_time ) {
          status_stream << " (total since capture: "
                        << static_cast<float>( output_stats.total_processing_time.count() / 100 ) /
                               10.0f
                        << "ms)";
        }
        status_stream << std::endl;
      }
      status_stream << indent << "  clients: "
                    << ( output_stats.client_count < 0 ? "Unknown"
                                                       : std::to_string( output_stats.client_count ) )
                    << std::endl;
    }
  }
  SERVER_LOG_INFO( "%s", status_stream.str().c_str() );
}

void CameraServer::checkAndRepair()
{

  if ( !initialized_ )
    return;
  SERVER_LOG_DEBUG( "Checking pipelines for issues" );
  for ( const auto &pipeline : pipelines_ ) {
    if ( !pipeline->isBuilt() ) {
      // Build, wire up WebRTC and start on the GStreamer thread, not here on the ROS timer
      // thread: buildPipeline() installs a bus watch (and other GSources) that bind to the
      // thread-default GMainContext, and the signaling server / endpoints_ are only safe to touch
      // from that context. Building here would attach the bus watch to the never-iterated global
      // default context (dead error monitoring) and race the GStreamer thread reading the
      // half-built pipeline. The build is asynchronous, so skip this pipeline for the rest of the
      // tick; later ticks see it built (or retry it if the build failed).
      struct RebuildRequest {
        CameraServer *server;
        CameraPipeline *pipeline;
      };
      g_main_context_invoke_full(
          g_main_loop_get_context( g_main_loop_ ), G_PRIORITY_DEFAULT,
          []( gpointer data ) -> gboolean {
            auto *req = static_cast<RebuildRequest *>( data );
            req->server->rebuildPipeline( req->pipeline );
            return G_SOURCE_REMOVE;
          },
          new RebuildRequest{ this, pipeline.get() },
          []( gpointer data ) { delete static_cast<RebuildRequest *>( data ); } );
      continue;
    }

    if ( pipeline->getState() == GST_STATE_PLAYING ) {
      PipelineStatistics stats = pipeline->statistics();
      if ( stats.input_fps == 0 ) {
        if ( pipeline->uptime() < 10s ) {
          SERVER_LOG_DEBUG_STREAM(
              pipeline->id()
              << " is in PLAYING state but no input frames received yet, giving it more time." );
          continue;
        }
        SERVER_LOG_WARN( "No input frames received for pipeline '%s', restarting.",
                         pipeline->id().c_str() );
        pipeline->restart();
        continue;
      }

      // Check if any output's processing time exceeds the max threshold
      double max_processing_time_s = node_->get_parameter( "max_processing_time" ).as_double();
      auto max_processing_time =
          std::chrono::microseconds( static_cast<int64_t>( max_processing_time_s * 1e6 ) );
      for ( size_t i = 0; i < stats.output_statistics.size(); ++i ) {
        const auto &output_stats = stats.output_statistics[i];
        if ( output_stats.processing_time.count() < 0 )
          continue;
        if ( output_stats.processing_time <= max_processing_time )
          continue;

        // Build breakdown string for the warning
        std::stringstream breakdown;
        float proc_ms = static_cast<float>( output_stats.processing_time.count() ) / 1000.0f;
        float limit_ms = static_cast<float>( max_processing_time.count() ) / 1000.0f;
        breakdown << "Processing time for '" << pipeline->id() << "'/output " << i + 1
                  << " exceeded limit (" << std::fixed << std::setprecision( 1 ) << proc_ms
                  << "ms > " << limit_ms << "ms).";
        if ( output_stats.encoder_latency.count() >= 0 ) {
          auto post_enc = output_stats.processing_time - output_stats.pre_encoder_time -
                          output_stats.encoder_latency;
          breakdown << " Breakdown: pre-enc: "
                    << static_cast<float>( output_stats.pre_encoder_time.count() ) / 1000.0f
                    << "ms, enc: "
                    << static_cast<float>( output_stats.encoder_latency.count() ) / 1000.0f
                    << "ms, post-enc: " << static_cast<float>( post_enc.count() ) / 1000.0f << "ms.";
        }
        if ( output_stats.total_processing_time.count() >= 0 &&
             output_stats.total_processing_time != output_stats.processing_time ) {
          breakdown << " Total (incl. upstream): "
                    << static_cast<float>( output_stats.total_processing_time.count() ) / 1000.0f
                    << "ms.";
        }
        breakdown << " Restarting pipeline.";
        SERVER_LOG_WARN( "%s", breakdown.str().c_str() );
        pipeline->restart();
        break;
      }
      continue;
    }
    if ( pipeline->uptime() > 10s ) {
      SERVER_LOG_WARN( "Pipeline '%s' has been in non-PLAYING state for >10s, restarting.",
                       pipeline->id().c_str() );
      pipeline->restart();
    } else if ( pipeline->uptime() < 3s ) {
      SERVER_LOG_DEBUG_STREAM( pipeline->id()
                               << " is not in PLAYING state yet, giving it more time." );
    } else {
      SERVER_LOG_DEBUG_STREAM( pipeline->id() << " is not in PLAYING state, attempting to start." );
      pipeline->start();
    }
  }
}

void CameraServer::initializeWebRTC()
{

  if ( !signaling_server_ ) {
    // Check if any pipeline has WebRTC outputs
    bool has_webrtc = false;
    for ( const auto &pipeline : pipelines_ ) {
      if ( !pipeline->isBuilt() )
        continue;
      for ( const auto &output_config : pipeline->configuration().outputs ) {
        if ( output_config->type == WebrtcOutputConfiguration::TYPE ) {
          has_webrtc = true;
          break;
        }
      }
      if ( has_webrtc )
        break;
    }

    if ( !has_webrtc )
      return;

    // Create and start signaling server
    signaling_server_ = std::make_unique<SignalingServer>( configuration_.signaling_port );
    if ( !signaling_server_->start() ) {
      SERVER_LOG_ERROR( "Failed to start WebRTC signaling server!" );
      signaling_server_.reset();
      return;
    }
  }

  // Wire up each WebRTC output to the signaling server
  for ( const auto &pipeline : pipelines_ ) {
    if ( !pipeline->isBuilt() )
      continue;
    const auto &outputs = pipeline->outputs();
    const auto &output_configs = pipeline->configuration().outputs;
    for ( size_t i = 0; i < output_configs.size() && i < outputs.size(); ++i ) {
      if ( output_configs[i]->type == WebrtcOutputConfiguration::TYPE ) {
        std::string path = "/" + pipeline->id() + "/" + std::to_string( i );
        if ( !signaling_server_->hasEndpoint( path ) ) {
          webrtc_output_set_signaling_server( outputs[i].get(), signaling_server_.get(), path );
        }
      }
    }
  }
}

void CameraServer::rebuildPipeline( CameraPipeline *pipeline )
{
  if ( pipeline->isBuilt() )
    return; // already rebuilt by an earlier tick's request
  SERVER_LOG_INFO( "Trying to build pipeline for camera '%s' again.", pipeline->id().c_str() );
  // The monotonic system clock is the GStreamer singleton configured in the constructor; the
  // pipelines built at startup share it, so reuse it here to give the rebuilt pipeline the same
  // clock for consistent capture timestamps. gst_system_clock_obtain() refs it for us.
  GstClock *system_clock = gst_system_clock_obtain();
  bool built = false;
  try {
    pipeline->buildPipeline();
    pipeline->useClock( system_clock );
    built = true;
  } catch ( PipelineBuildError &e ) {
    SERVER_LOG_ERROR_STREAM( "Failed to rebuild pipeline for camera '"
                             << pipeline->id() << "'! Error: " << e.what() );
  }
  gst_object_unref( system_clock );
  if ( !built )
    return;
  // Wire up WebRTC outputs (creates the signaling server on first use; idempotent for already
  // wired pipelines), then start the freshly built pipeline.
  initializeWebRTC();
  pipeline->start();
}

void CameraServer::updateFlowControl()
{
  if ( !initialized_ )
    return;
  for ( const auto &pipeline : pipelines_ ) {
    if ( pipeline->isBuilt() )
      pipeline->updateFlowControl();
  }
}

} // namespace ros_camera_server
