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

#ifndef ROS_CAMERA_SERVER_PIPELINE_FLOW_CONTROLLER_HPP
#define ROS_CAMERA_SERVER_PIPELINE_FLOW_CONTROLLER_HPP

#include "ros_camera_server/outputs/pipeline_output.hpp"

#include <chrono>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <optional>
#include <string>
#include <vector>

#include "../logging.hpp"

namespace ros_camera_server
{

/// A valve element and the output indices whose data flows through it.
struct BranchValve {
  GstElement *valve = nullptr;
  std::vector<size_t> output_indices;
};

/// Valve elements inserted by PipelineBuilder for flow control.
struct FlowControlInfo {
  std::vector<BranchValve> branch_valves;
};

/// Periodically checks output client counts and opens/closes valve elements
/// to avoid processing when no clients are connected downstream.
///
/// Branch valves are placed after tee elements to gate per-branch processing
/// (encoding, scaling) when no clients need that branch's output.
/// A grace period prevents rapid toggling during brief client reconnections
/// and ensures the pipeline runs long enough at startup to validate itself.
class FlowController
{
public:
  FlowController() = default;

  FlowController( FlowControlInfo info, const std::vector<PipelineOutput::Ptr> &outputs )
      : info_( std::move( info ) ), outputs_( &outputs ),
        branch_valve_open_( info_.branch_valves.size(), true ),
        branch_inactive_since_( info_.branch_valves.size() )
  {
  }

  /// Check client counts and update valve states.
  void update()
  {
    if ( !outputs_ )
      return;

    auto now = clock::now();

    for ( size_t i = 0; i < info_.branch_valves.size(); ++i ) {
      bool branch_active = isBranchActive( info_.branch_valves[i] );

      if ( branch_active ) {
        // Client connected: open valve immediately, reset grace timer
        branch_inactive_since_[i].reset();
        if ( !branch_valve_open_[i] ) {
          openValve( info_.branch_valves[i].valve );
          branch_valve_open_[i] = true;
        }
      } else {
        // No clients: start grace timer if not already running
        if ( !branch_inactive_since_[i].has_value() ) {
          branch_inactive_since_[i] = now;
        }
        // Only close after the grace period has elapsed
        if ( branch_valve_open_[i] && now - *branch_inactive_since_[i] >= GRACE_PERIOD ) {
          closeValve( info_.branch_valves[i].valve );
          branch_valve_open_[i] = false;
          SERVER_LOG_INFO( "Flow control: closed valve for outputs %s (no clients for %ds)",
                           formatOutputIndices( info_.branch_valves[i] ).c_str(),
                           static_cast<int>( GRACE_PERIOD.count() ) );
        }
      }
    }
  }

  /// Returns true if data is flowing to the given output index.
  /// An output may be covered by multiple nested valves (e.g., root tee and inner tee).
  /// Data only reaches the output if all covering valves are open.
  bool isOutputFlowing( size_t output_index ) const
  {
    for ( size_t i = 0; i < info_.branch_valves.size(); ++i ) {
      const auto &indices = info_.branch_valves[i].output_indices;
      if ( std::find( indices.begin(), indices.end(), output_index ) != indices.end() ) {
        if ( !branch_valve_open_[i] )
          return false;
      }
    }
    return true;
  }

private:
  static constexpr std::chrono::seconds GRACE_PERIOD{ 10 };
  using clock = std::chrono::steady_clock;

  bool isOutputActive( size_t output_index ) const
  {
    if ( output_index >= outputs_->size() )
      return true;
    int count = ( *outputs_ )[output_index]->getClientCount();
    // -1 means "unknown" → treat as always active
    return count != 0;
  }

  bool isBranchActive( const BranchValve &branch ) const
  {
    for ( size_t idx : branch.output_indices ) {
      if ( isOutputActive( idx ) )
        return true;
    }
    return false;
  }

  static void openValve( GstElement *valve )
  {
    g_object_set( valve, "drop", FALSE, nullptr );
    // Send force-key-unit event upstream so encoders produce an IDR frame.
    // Must use the src pad: gst_pad_send_event on a src pad sends upstream.
    GstPad *src_pad = gst_element_get_static_pad( valve, "src" );
    if ( src_pad ) {
      GstEvent *event = gst_video_event_new_upstream_force_key_unit( GST_CLOCK_TIME_NONE, TRUE, 0 );
      gst_pad_send_event( src_pad, event );
      gst_object_unref( src_pad );
    }
  }

  static void closeValve( GstElement *valve ) { g_object_set( valve, "drop", TRUE, nullptr ); }

  static std::string formatOutputIndices( const BranchValve &branch )
  {
    std::string result;
    for ( size_t i = 0; i < branch.output_indices.size(); ++i ) {
      if ( i > 0 )
        result += ", ";
      result += std::to_string( branch.output_indices[i] + 1 );
    }
    return result;
  }

  FlowControlInfo info_;
  const std::vector<PipelineOutput::Ptr> *outputs_ = nullptr;
  std::vector<bool> branch_valve_open_;
  std::vector<std::optional<clock::time_point>> branch_inactive_since_;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_FLOW_CONTROLLER_HPP
