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

#ifndef ROS_CAMERA_SERVER_PIPELINE_PIPELINE_BUILDER_HPP
#define ROS_CAMERA_SERVER_PIPELINE_PIPELINE_BUILDER_HPP

#include "../codecs/codec_common.hpp"
#include "../diagnostic_context.hpp"
#include "flow_controller.hpp"
#include "pipeline_graph.hpp"

#include <gst/gst.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <unordered_map>

class FramerateLimiterRuntimeTest;

namespace ros_camera_server
{

struct PipelineBuildResult {
  std::vector<PipelineOutput::Ptr> outputs;
  FlowControlInfo flow_control;
  // (tee, request_pad) pairs that must be released via
  // gst_element_release_request_pad before the pipeline is unreffed.
  // Pointers are non-owning; the tee's parent bin keeps the pad alive.
  std::vector<std::pair<GstElement *, GstPad *>> tee_request_pads;
};

/**
 * @brief Realizes a PipelineGraph into actual GStreamer elements.
 *
 * Takes an abstract PipelineGraph and creates the corresponding GStreamer
 * elements, adding tees where nodes have multiple outputs and queues for
 * buffering between stages. Also creates and links output bins.
 */
class PipelineBuilder
{
public:
  explicit PipelineBuilder( GstBin *pipeline, rclcpp::Node::SharedPtr node );

  /**
   * @brief Realize the entire pipeline graph including outputs.
   *
   * @param graph The abstract pipeline graph to realize
   * @param input_element The GStreamer element for the source (must already be added to pipeline)
   * @param output_configs The output configurations for creating output bins
   * @return PipelineBuildResult containing created outputs and flow control info
   */
  PipelineBuildResult realize( PipelineGraph &graph, GstElement *input_element,
                               const std::vector<std::shared_ptr<OutputConfiguration>> &output_configs,
                               const std::string &camera_id );

  // Collect downstream h264/h265 encoder node IDs reachable from a node.
  // Sets has_raw_sink if any sink path has no upstream encoder.
  static void collectDownstreamEncoders( const PipelineGraph &graph, NodeId node_id,
                                         std::vector<NodeId> &out, bool &has_raw_sink );

private:
  GstBin *pipeline_;
  rclcpp::Node::SharedPtr node_;
  int queue_counter_ = 0;
  int valve_counter_ = 0;

  // Mapping from NodeId to realized GStreamer elements
  std::unordered_map<NodeId, GstElement *> realized_elements_;
  std::unordered_map<NodeId, GstElement *> output_tees_;  // For nodes with multiple outputs
  std::unordered_map<NodeId, std::string> encoder_names_; // Encoder factory names for stats
  std::unordered_map<NodeId, std::shared_ptr<struct EncoderTimingStats>> encoder_timing_stats_;
  std::unordered_map<NodeId, uint32_t>
      encoder_preselected_; // Pre-selected backend flag per encoder (for HW scaler selection)
  std::unordered_map<NodeId, MemoryFeature> scale_target_memory_; // Target memory for HW scaling
  std::string camera_id_;

  // Flow control valve tracking
  FlowControlInfo flow_control_info_;

  // Tee request pads handed out via gst_element_request_pad_simple. Stored as
  // non-owning pointers (the +1 caller-owned ref is dropped after linking).
  // Released by CameraPipeline before the parent pipeline is unreffed.
  std::vector<std::pair<GstElement *, GstPad *>> tee_request_pads_;

  // Realize processing nodes (everything except sinks)
  void realizeProcessingNodes( PipelineGraph &graph, GstElement *input_element );

  // Create and link outputs for sink nodes
  std::vector<PipelineOutput::Ptr>
  realizeOutputs( const PipelineGraph &graph,
                  const std::vector<std::shared_ptr<OutputConfiguration>> &output_configs,
                  const std::string &camera_id );

  // Get the source element for a sink node
  GstElement *getSourceForSink( const PipelineGraph &graph, NodeId sink_id );

  // Create GStreamer element for a node
  GstElement *createElementForNode( const GraphNode &node );

  // Create specific element types
  GstElement *createDecoderElement( NodeId node_id, const DecoderKey &key );
  static GstElement *createFramerateLimiterElement( NodeId node_id, const FramerateKey &key );
  GstElement *createScaleElement( NodeId node_id, const ScaleKey &key, MemoryFeature target_memory );
  GstElement *createEncoderElement( NodeId node_id, const EncoderKey &key );

  // Pre-select encoder backends for all h264/h265 encoder nodes
  void preselectEncoderBackends( const PipelineGraph &graph );

  // Determine HW scaling targets for Scale nodes based on downstream encoders
  void determineHwScaling( const PipelineGraph &graph );

  // Get encoder factory name from element or bin
  static std::string getEncoderName( GstElement *element );

  // Find encoder name for a sink by walking back the graph
  std::string findEncoderForSink( const PipelineGraph &graph, NodeId sink_id );

  // Find encoder NodeId for a sink by walking back the graph (-1 if none)
  static NodeId findEncoderNodeForSink( const PipelineGraph &graph, NodeId sink_id );

  // Link elements with queue (optionally inserting a valve before the queue)
  void linkWithQueue( GstElement *src, GstElement *dest, const std::string &queue_name,
                      bool is_interdependent_format = false, GstElement *valve = nullptr,
                      std::shared_ptr<const DiagnosticContext> context = nullptr );

  // Check if element is a tee
  static bool isTee( GstElement *element );

  // Get name of GStreamer element safely (handles NULL)
  static std::string getElementName( GstElement *element );

  // Get string representation of node type
  static std::string nodeTypeToString( GraphNodeType type );

  std::shared_ptr<const DiagnosticContext> makeDiagnosticContext( const PipelineGraph &graph,
                                                                  const GraphNode &node ) const;

  // Topological sort for correct creation order
  std::vector<NodeId> topologicalSort( const PipelineGraph &graph );

  // Collect all output indices reachable from a node in the graph
  static void collectDownstreamOutputs( const PipelineGraph &graph, NodeId node_id,
                                        std::vector<size_t> &out );

  // Generate a textual description of the realized pipeline starting from the source
  std::string dumpPipeline( const PipelineGraph &graph );

  friend class ::FramerateLimiterRuntimeTest;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_PIPELINE_BUILDER_HPP
