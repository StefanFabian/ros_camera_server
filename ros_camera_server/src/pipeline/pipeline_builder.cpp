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

#include "pipeline_builder.hpp"
#include "../codecs/h264.hpp"
#include "../codecs/h265.hpp"
#include "../codecs/jpeg.hpp"
#include "../codecs/png.hpp"
#include "../codecs/smart_video_decoder.hpp"
#include "../logging.hpp"
#include "ros_camera_server/exceptions.hpp"
#include "ros_camera_server/helpers/reference_timestamp_helpers.hpp"
#include "ros_camera_server/helpers/smart_gst_pointer.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <set>

namespace ros_camera_server
{

PipelineBuilder::PipelineBuilder( GstBin *pipeline, rclcpp::Node::SharedPtr node )
    : pipeline_( pipeline ), node_( std::move( node ) )
{
  if ( node_ == nullptr ) {
    throw std::invalid_argument( "PipelineBuilder requires a valid rclcpp::Node shared pointer" );
  }
}

PipelineBuildResult
PipelineBuilder::realize( PipelineGraph &graph, GstElement *input_element,
                          const std::vector<std::shared_ptr<OutputConfiguration>> &output_configs,
                          const std::string &camera_id )
{
  SERVER_LOG_DEBUG( "Realizing pipeline for camera '%s' with %zu output(s)", camera_id.c_str(),
                    output_configs.size() );
  // Clear previous state
  camera_id_ = camera_id;
  realized_elements_.clear();
  output_tees_.clear();
  encoder_names_.clear();
  encoder_preselected_.clear();
  scale_target_memory_.clear();
  encoder_timing_stats_.clear();
  flow_control_info_ = {};
  tee_request_pads_.clear();
  queue_counter_ = 0;
  valve_counter_ = 0;

  // Realize all processing nodes
  realizeProcessingNodes( graph, input_element );

  // Create and link outputs
  PipelineBuildResult result;
  result.outputs = realizeOutputs( graph, output_configs, camera_id );
  result.flow_control = std::move( flow_control_info_ );
  result.tee_request_pads = std::move( tee_request_pads_ );

  SERVER_LOG_DEBUG( "%s", dumpPipeline( graph ).c_str() );
  return result;
}

void PipelineBuilder::realizeProcessingNodes( PipelineGraph &graph, GstElement *input_element )
{
  // Pre-select encoder backends so Scale nodes can determine HW scaler compatibility
  preselectEncoderBackends( graph );
  determineHwScaling( graph );

  // Get nodes in topological order
  auto sorted_nodes = topologicalSort( graph );
  {
    std::stringstream ss;
    for ( NodeId id : sorted_nodes ) ss << id << " ";
    SERVER_LOG_DEBUG( "Topological sort: %s", ss.str().c_str() );
  }

  // Create elements for each node
  for ( NodeId id : sorted_nodes ) {
    GraphNode &node = graph.getNode( id );

    if ( node.type == GraphNodeType::Source ) {
      realized_elements_[id] = input_element;
      attachDiagnosticContext( G_OBJECT( input_element ), makeDiagnosticContext( graph, node ) );
      SERVER_LOG_DEBUG( "Node %u: Source (input element)", id );
    } else if ( node.type != GraphNodeType::Sink ) {
      GstElement *element = createElementForNode( node );
      if ( !element ) {
        throw PipelineBuildError( "Failed to create element for node " + std::to_string( id ) );
      }
      if ( !gst_bin_add( pipeline_, element ) ) {
        throw PipelineBuildError( "Failed to add element for node " + std::to_string( id ) +
                                  " to pipeline." );
      }
      realized_elements_[id] = element;
      attachDiagnosticContext( G_OBJECT( element ), makeDiagnosticContext( graph, node ) );
      SERVER_LOG_DEBUG( "Node %u: created %s element %p", id, nodeTypeToString( node.type ).c_str(),
                        static_cast<void *>( element ) );

      // Track encoder names for statistics and add passthrough probes
      if ( node.type == GraphNodeType::Encoder ) {
        encoder_names_[id] = getEncoderName( element );
        SERVER_LOG_DEBUG( "Node %u: Encoder '%s'", id, encoder_names_[id].c_str() );
        // Add probes to preserve capture time meta through HW encoders that strip it
        encoder_timing_stats_[id] = add_reference_timestamp_passthrough_probes( element );
      }
    } else {
      SERVER_LOG_DEBUG( "Node %u: realization skipped (type=%s)", id,
                        nodeTypeToString( node.type ).c_str() );
    }

    // Create tee if node has multiple outputs
    if ( node.outputs.size() > 1 && node.type != GraphNodeType::Sink ) {
      std::string tee_name = "tee_node_" + std::to_string( id );
      GstElement *tee = gst_element_factory_make( "tee", tee_name.c_str() );
      if ( !tee ) {
        throw PipelineBuildError( "Failed to create tee for node " + std::to_string( id ) );
      }
      if ( !gst_bin_add( pipeline_, tee ) ) {
        throw PipelineBuildError( "Failed to add tee for node " + std::to_string( id ) +
                                  " to pipeline." );
      }
      output_tees_[id] = tee;
      attachDiagnosticContext( G_OBJECT( tee ), makeDiagnosticContext( graph, node ) );
      SERVER_LOG_DEBUG( "Node %u: created tee '%s' for %zu outputs", id, tee_name.c_str(),
                        node.outputs.size() );

      // Link element to its tee
      GstElement *element = realized_elements_[id];
      if ( !gst_element_link( element, tee ) ) {
        throw PipelineBuildError( "Failed to link element to tee for node " + std::to_string( id ) );
      }
    }
  }

  // Link all nodes (skip source and sinks)
  for ( NodeId id : sorted_nodes ) {
    GraphNode &node = graph.getNode( id );

    if ( node.type == GraphNodeType::Source || node.type == GraphNodeType::Sink ) {
      continue;
    }

    if ( !node.input.has_value() ) {
      continue;
    }

    NodeId parent_id = node.input.value();

    // Determine if parent produces inter-dependent frames (H.264/H.265)
    const GraphNode &parent_node = graph.getNode( parent_id );
    bool is_interdependent_format = false;
    if ( parent_node.type == GraphNodeType::Encoder ) {
      const EncoderKey &encoder_key = std::get<EncoderKey>( parent_node.config );
      is_interdependent_format = ( encoder_key.codec == "h264" || encoder_key.codec == "h265" );
    }

    // Get source element (tee if parent has multiple outputs)
    GstElement *src =
        output_tees_.count( parent_id ) ? output_tees_[parent_id] : realized_elements_[parent_id];

    GstElement *dest = realized_elements_[id];
    if ( !dest ) {
      SERVER_LOG_ERROR( "Node %u: destination element not found in realized map", id );
      throw PipelineBuildError( "Link error: Node " + std::to_string( id ) +
                                " has no realized element." );
    }
    if ( !GST_IS_ELEMENT( dest ) ) {
      SERVER_LOG_ERROR( "Node %u: realized entry %p is not a valid GStreamer element", id,
                        (void *)dest );
      throw PipelineBuildError( "Link error: Node " + std::to_string( id ) +
                                " has an invalid element pointer." );
    }

    // Insert a valve on tee branches for flow control
    GstElement *valve = nullptr;
    if ( isTee( src ) ) {
      std::string valve_name = "valve_" + std::to_string( valve_counter_++ );
      valve = gst_element_factory_make( "valve", valve_name.c_str() );
      if ( valve ) {
        BranchValve bv;
        bv.valve = valve;
        collectDownstreamOutputs( graph, id, bv.output_indices );
        flow_control_info_.branch_valves.push_back( std::move( bv ) );
        attachDiagnosticContext( G_OBJECT( valve ), makeDiagnosticContext( graph, node ) );
        SERVER_LOG_DEBUG( "Node %u: inserted valve '%s' on tee branch", id, valve_name.c_str() );
      }
    }

    std::string queue_name = "q_" + std::to_string( queue_counter_++ );
    SERVER_LOG_DEBUG( "Node %u (%s) -> Node %u (%s): linking with queue '%s' (interdependent=%s)",
                      parent_id, nodeTypeToString( parent_node.type ).c_str(), id,
                      nodeTypeToString( node.type ).c_str(), queue_name.c_str(),
                      is_interdependent_format ? "true" : "false" );
    linkWithQueue( src, dest, queue_name, is_interdependent_format, valve,
                   makeDiagnosticContext( graph, node ) );
  }
}

std::vector<PipelineOutput::Ptr> PipelineBuilder::realizeOutputs(
    const PipelineGraph &graph,
    const std::vector<std::shared_ptr<OutputConfiguration>> &output_configs,
    const std::string &camera_id )
{
  // Pre-size the vector to ensure outputs are placed at correct indices
  std::vector<PipelineOutput::Ptr> outputs( output_configs.size() );

  for ( NodeId sink_id : graph.sink_nodes ) {
    const GraphNode &sink_node = graph.getNode( sink_id );
    const SinkConfig &sink_config = std::get<SinkConfig>( sink_node.config );
    int output_index = sink_config.output_index;

    auto &output_config = output_configs[output_index];

    SERVER_LOG_DEBUG( "Realizing sink node %u -> output index %d", sink_id, output_index );
    // Create output bin
    PipelineOutput::Ptr output = output_config->createOutput( node_, camera_id, output_index );
    if ( !output || !output->bin ) {
      throw PipelineBuildError( "Failed to create output " + std::to_string( output_index ) );
    }
    SERVER_LOG_DEBUG( "Sink Node %u: created output index %d (pointer %p)", sink_id, output_index,
                      (void *)output->bin );
    attachDiagnosticContext( G_OBJECT( output->bin ), makeDiagnosticContext( graph, sink_node ) );
    if ( !gst_bin_add( pipeline_, GST_ELEMENT( output->bin ) ) ) {
      throw PipelineBuildError( "Failed to add output " + std::to_string( output_index ) +
                                " bin to pipeline." );
    }
    realized_elements_[sink_id] = GST_ELEMENT( output->bin );

    // Determine if parent produces inter-dependent frames (H.264/H.265)
    NodeId parent_id = sink_node.input.value();
    const GraphNode &parent_node = graph.getNode( parent_id );
    bool is_interdependent_format = false;
    if ( parent_node.type == GraphNodeType::Encoder ) {
      const EncoderKey &encoder_key = std::get<EncoderKey>( parent_node.config );
      is_interdependent_format = ( encoder_key.codec == "h264" || encoder_key.codec == "h265" );
    }

    // Link output to its source, inserting a valve on tee branches
    GstElement *source = getSourceForSink( graph, sink_id );
    GstElement *valve = nullptr;
    if ( isTee( source ) ) {
      std::string valve_name = "valve_" + std::to_string( valve_counter_++ );
      valve = gst_element_factory_make( "valve", valve_name.c_str() );
      if ( valve ) {
        BranchValve bv;
        bv.valve = valve;
        bv.output_indices.push_back( static_cast<size_t>( output_index ) );
        flow_control_info_.branch_valves.push_back( std::move( bv ) );
        attachDiagnosticContext( G_OBJECT( valve ), makeDiagnosticContext( graph, sink_node ) );
        SERVER_LOG_DEBUG( "Output %d: inserted valve '%s' on tee branch", output_index,
                          valve_name.c_str() );
      }
    }
    std::string output_queue_name = "output_queue_" + std::to_string( output_index );
    SERVER_LOG_DEBUG( "Sink Node %u -> Output %d: linking with queue '%s' (interdependent=%s)",
                      sink_id, output_index, output_queue_name.c_str(),
                      is_interdependent_format ? "true" : "false" );
    linkWithQueue( source, GST_ELEMENT( output->bin ), output_queue_name, is_interdependent_format,
                   valve, makeDiagnosticContext( graph, sink_node ) );

    // Set encoder name, codec, and timing stats for statistics
    output->encoder_name = findEncoderForSink( graph, sink_id );
    NodeId encoder_node = findEncoderNodeForSink( graph, sink_id );
    if ( encoder_node >= 0 ) {
      const EncoderKey &key = std::get<EncoderKey>( graph.getNode( encoder_node ).config );
      output->configured_codec = key.codec;

      auto it = encoder_timing_stats_.find( encoder_node );
      if ( it != encoder_timing_stats_.end() ) {
        output->setEncoderTimingStats( it->second );
      }
    }

    // Place output at its correct index to match output_configs order
    outputs[output_index] = std::move( output );
  }

  return outputs;
}

GstElement *PipelineBuilder::getSourceForSink( const PipelineGraph &graph, NodeId sink_id )
{
  const GraphNode &sink_node = graph.getNode( sink_id );
  if ( sink_node.type != GraphNodeType::Sink || !sink_node.input.has_value() ) {
    throw PipelineBuildError( "Invalid sink node" );
  }

  NodeId parent_id = sink_node.input.value();

  // Return tee if parent has multiple outputs, otherwise the element itself
  if ( output_tees_.count( parent_id ) ) {
    return output_tees_[parent_id];
  }
  return realized_elements_[parent_id];
}

GstElement *PipelineBuilder::createElementForNode( const GraphNode &node )
{
  switch ( node.type ) {
  case GraphNodeType::Decoder: {
    StreamFormat format = std::get<StreamFormat>( node.config );
    SERVER_LOG_DEBUG( "Node %u: creating Decoder for format '%s'", node.id,
                      to_string( format ).c_str() );
    return createDecoderElement( node.id, format );
  }
  case GraphNodeType::FramerateLimit: {
    const FramerateKey &key = std::get<FramerateKey>( node.config );
    SERVER_LOG_DEBUG( "Node %u: creating FramerateLimit max=%s", node.id,
                      key.max_framerate.toString().c_str() );
    return createFramerateLimiterElement( node.id, key );
  }
  case GraphNodeType::Scale: {
    const ScaleKey &key = std::get<ScaleKey>( node.config );
    auto it = scale_target_memory_.find( node.id );
    MemoryFeature target = it != scale_target_memory_.end() ? it->second : MemoryFeature::NONE;
    SERVER_LOG_DEBUG( "Node %u: creating Scale max=%dx%d target_memory=0x%x", node.id,
                      key.max_width, key.max_height, static_cast<uint32_t>( target ) );
    return createScaleElement( node.id, key, target );
  }
  case GraphNodeType::Encoder: {
    const EncoderKey &key = std::get<EncoderKey>( node.config );
    SERVER_LOG_DEBUG( "Node %u: creating Encoder codec=%s encoder=%s %dx%d@%s bitrate=%dkbps",
                      node.id, key.codec.c_str(), key.encoder.c_str(), key.width, key.height,
                      key.framerate.toString().c_str(), key.bitrate );
    GstElement *element = createEncoderElement( node.id, key );
    if ( !element ) {
      throw PipelineBuildError( "Failed to create encoder '" + key.codec + "' (backend: " +
                                key.encoder + ") for node " + std::to_string( node.id ) );
    }
    return element;
  }
  default:
    return nullptr;
  }
}

GstElement *PipelineBuilder::createDecoderElement( NodeId node_id, StreamFormat format )
{
  std::string name = "decoder_node_" + std::to_string( node_id );
  switch ( format ) {
  case StreamFormat::JPEG:
    return createSmartVideoDecoder( name, jpegDecoderDescriptor() );
  case StreamFormat::PNG:
    return createPngDecoder( name );
  default:
    throw PipelineBuildError( "Unsupported decoder format: " + to_string( format ) );
  }
}

GstElement *PipelineBuilder::createFramerateLimiterElement( NodeId node_id, const FramerateKey &key )
{
  std::string name = "framerate_" + std::to_string( key.max_framerate.numerator ) + "-" +
                     std::to_string( key.max_framerate.denominator ) + "_node_" +
                     std::to_string( node_id );

  SmartGstPointer<GstElement> bin = gst_bin_new( name.c_str() );
  SmartGstPointer<GstElement> videorate = gst_element_factory_make( "videorate", "rate" );
  SmartGstPointer<GstElement> capsfilter = gst_element_factory_make( "capsfilter", "caps" );

  if ( !videorate || !capsfilter || !bin ) {
    return nullptr;
  }

  // Configure videorate to drop frames only (not duplicate)
  g_object_set( videorate.get(), "drop-only", TRUE, nullptr );

  // Set caps with framerate range (0 to max) for all supported videorate types
  constexpr std::array<const char *, 4> media_types = { "video/x-raw", "video/x-bayer",
                                                        "image/jpeg", "image/png" };
  GstCaps *caps = gst_caps_new_empty();
  for ( const char *type : media_types ) {
    GstStructure *s =
        gst_structure_new( type, "framerate", GST_TYPE_FRACTION_RANGE, 0, 1,
                           key.max_framerate.numerator, key.max_framerate.denominator, nullptr );
    gst_caps_append_structure( caps, s );
  }

  g_object_set( capsfilter.get(), "caps", caps, nullptr );
  gst_caps_unref( caps );

  // Keep raw pointers for linking after ownership transfer
  GstElement *raw_videorate = videorate.get();
  GstElement *raw_capsfilter = capsfilter.get();
  GstElement *raw_bin = bin.get();

  // Add to bin (transfers ownership)
  GstBin *gst_bin = GST_BIN( bin.release() );
  if ( !gst_bin_add( gst_bin, videorate.release() ) ||
       !gst_bin_add( gst_bin, capsfilter.release() ) ) {
    return nullptr;
  }
  gst_element_link( raw_videorate, raw_capsfilter );

  // Ghost pads
  GstPad *sink_pad = gst_element_get_static_pad( raw_videorate, "sink" );
  GstPad *src_pad = gst_element_get_static_pad( raw_capsfilter, "src" );
  gst_element_add_pad( raw_bin, gst_ghost_pad_new( "sink", sink_pad ) );
  gst_element_add_pad( raw_bin, gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( sink_pad );
  gst_object_unref( src_pad );

  return raw_bin;
}

GstElement *PipelineBuilder::createScaleElement( NodeId node_id, const ScaleKey &key,
                                                 MemoryFeature target_memory )
{
  std::string name = "scale_" + std::to_string( key.max_width ) + "x" +
                     std::to_string( key.max_height ) + "_node_" + std::to_string( node_id );

  // Build caps dimensions for upper-limit semantics
  int max_width =
      ( key.max_width != std::numeric_limits<int>::max() && key.max_width > 0 ) ? key.max_width : 0;
  int max_height = ( key.max_height != std::numeric_limits<int>::max() && key.max_height > 0 )
                       ? key.max_height
                       : 0;

  // Try HW scalers if a target memory type was determined
  if ( target_memory != MemoryFeature::NONE ) {
    for ( const auto &candidate : HW_SCALER_CANDIDATES ) {
      MemoryFeature overlap = highestPriorityOverlap( candidate.output_memory, target_memory );
      if ( overlap == MemoryFeature::NONE )
        continue;

      GstElement *scaler = gst_element_factory_make( candidate.factory, "scaler" );
      if ( !scaler )
        continue;

      GstElement *upload = nullptr;
      if ( candidate.upload_factory ) {
        upload = gst_element_factory_make( candidate.upload_factory, "upload" );
        if ( !upload ) {
          gst_object_unref( scaler );
          continue;
        }
      }

      // HW scaler available - build the bin
      const char *caps_feature = capsFeatureForMemoryFlag( overlap );
      GstElement *hw_bin = gst_bin_new( name.c_str() );
      GstElement *capsfilter = gst_element_factory_make( "capsfilter", "caps" );
      if ( !capsfilter ) {
        gst_object_unref( scaler );
        if ( upload )
          gst_object_unref( upload );
        gst_object_unref( hw_bin );
        continue;
      }

      // Set caps: video/x-raw(memory:XXX), format=NV12, width/height ranges
      GstCaps *caps = gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, "NV12", nullptr );
      gst_caps_set_features( caps, 0, gst_caps_features_new( caps_feature, nullptr ) );
      GstStructure *s = gst_caps_get_structure( caps, 0 );
      if ( max_width > 0 )
        gst_structure_set( s, "width", GST_TYPE_INT_RANGE, 1, max_width, nullptr );
      if ( max_height > 0 )
        gst_structure_set( s, "height", GST_TYPE_INT_RANGE, 1, max_height, nullptr );
      gst_structure_set( s, "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, nullptr );
      g_object_set( capsfilter, "caps", caps, nullptr );
      gst_caps_unref( caps );

      // Assemble bin: [upload ->] scaler -> capsfilter
      GstElement *first_element = upload ? upload : scaler;
      if ( upload ) {
        if ( !gst_bin_add( GST_BIN( hw_bin ), upload ) ) {
          gst_object_unref( hw_bin );
          gst_object_unref( upload );
          gst_object_unref( scaler );
          gst_object_unref( capsfilter );
          return nullptr;
        }
        if ( !gst_bin_add( GST_BIN( hw_bin ), scaler ) ) {
          gst_object_unref( hw_bin );
          gst_object_unref( scaler );
          gst_object_unref( capsfilter );
          return nullptr;
        }
        if ( !gst_bin_add( GST_BIN( hw_bin ), capsfilter ) ) {
          gst_object_unref( hw_bin );
          gst_object_unref( capsfilter );
          return nullptr;
        }
        gst_element_link_many( upload, scaler, capsfilter, nullptr );
      } else {
        if ( !gst_bin_add( GST_BIN( hw_bin ), scaler ) ||
             !gst_bin_add( GST_BIN( hw_bin ), capsfilter ) ) {
          gst_object_unref( hw_bin );
          gst_object_unref( scaler );
          gst_object_unref( capsfilter );
          return nullptr;
        }
        gst_element_link( scaler, capsfilter );
      }

      GstPad *sink_pad = gst_element_get_static_pad( first_element, "sink" );
      GstPad *src_pad = gst_element_get_static_pad( capsfilter, "src" );
      gst_element_add_pad( hw_bin, gst_ghost_pad_new( "sink", sink_pad ) );
      gst_element_add_pad( hw_bin, gst_ghost_pad_new( "src", src_pad ) );
      gst_object_unref( sink_pad );
      gst_object_unref( src_pad );

      SERVER_LOG_INFO( "Using HW scaler '%s' with %s for %s", candidate.factory, caps_feature,
                       name.c_str() );
      return hw_bin;
    }

    SERVER_LOG_WARN(
        "No HW scaler available for target memory 0x%x, falling back to software for %s",
        static_cast<uint32_t>( target_memory ), name.c_str() );
  }

  // Software fallback: videoconvertscale + capsfilter
  SmartGstPointer<GstElement> bin = gst_bin_new( name.c_str() );
  SmartGstPointer<GstElement> videoconvertscale =
      gst_element_factory_make( "videoconvertscale", "convertscale" );
  SmartGstPointer<GstElement> capsfilter = gst_element_factory_make( "capsfilter", "caps" );

  if ( !videoconvertscale || !capsfilter || !bin ) {
    throw PipelineBuildError( "Failed to create scale elements" );
  }

  GstCaps *caps = gst_caps_new_empty_simple( "video/x-raw" );
  GstStructure *s = gst_caps_get_structure( caps, 0 );
  if ( max_width > 0 )
    gst_structure_set( s, "width", GST_TYPE_INT_RANGE, 1, max_width, nullptr );
  if ( max_height > 0 )
    gst_structure_set( s, "height", GST_TYPE_INT_RANGE, 1, max_height, nullptr );
  gst_structure_set( s, "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, nullptr );

  g_object_set( capsfilter.get(), "caps", caps, nullptr );
  gst_caps_unref( caps );

  GstElement *raw_convertscale = videoconvertscale.get();
  GstElement *raw_capsfilter = capsfilter.get();
  GstElement *raw_bin = bin.get();

  GstBin *gst_bin = GST_BIN( bin.release() );
  if ( !gst_bin_add( gst_bin, videoconvertscale.release() ) ||
       !gst_bin_add( gst_bin, capsfilter.release() ) ) {
    // Note: bin owns what was added, release() already called
    return nullptr;
  }
  gst_element_link( raw_convertscale, raw_capsfilter );

  GstPad *sink_pad = gst_element_get_static_pad( raw_convertscale, "sink" );
  GstPad *src_pad = gst_element_get_static_pad( raw_capsfilter, "src" );
  gst_element_add_pad( raw_bin, gst_ghost_pad_new( "sink", sink_pad ) );
  gst_element_add_pad( raw_bin, gst_ghost_pad_new( "src", src_pad ) );
  gst_object_unref( sink_pad );
  gst_object_unref( src_pad );

  return raw_bin;
}

GstElement *PipelineBuilder::createEncoderElement( NodeId node_id, const EncoderKey &key )
{
  std::string name = "encoder_node_" + std::to_string( node_id );
  if ( key.codec == "h264" ) {
    return createH264Encoder( key.encoder, name, CodecOptions{ key.bitrate } );
  } else if ( key.codec == "h265" ) {
    return createH265Encoder( key.encoder, name, CodecOptions{ key.bitrate } );
  } else if ( key.codec == "jpeg" ) {
    return createJpegEncoder( key.encoder, name );
  } else if ( key.codec == "png" ) {
    return createPngEncoder( key.encoder, name );
  } else {
    throw PipelineBuildError( "Unsupported codec: " + key.codec );
  }
}

void PipelineBuilder::preselectEncoderBackends( const PipelineGraph &graph )
{
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type != GraphNodeType::Encoder )
      continue;
    const EncoderKey &key = std::get<EncoderKey>( node.config );
    if ( key.codec != "h264" && key.codec != "h265" )
      continue;

    const CodecDescriptor &desc = key.codec == "h264" ? h264EncoderDescriptor() : h265_encoder_desc;
    uint32_t desired = codecFromString( desc, key.encoder );
    uint32_t selected = codecSelect( desc, desired );

    if ( selected != 0 ) {
      encoder_preselected_[id] = selected;
      SERVER_LOG_DEBUG( "Pre-selected encoder backend 0x%x for node %d (%s)", selected, id,
                        key.codec.c_str() );
    }
  }
}

void PipelineBuilder::collectDownstreamEncoders( const PipelineGraph &graph, NodeId node_id,
                                                 std::vector<NodeId> &out, bool &has_raw_sink )
{
  const GraphNode &node = graph.getNode( node_id );
  if ( node.type == GraphNodeType::Sink ) {
    has_raw_sink = true;
    return;
  }
  if ( node.type == GraphNodeType::Encoder ) {
    const EncoderKey &key = std::get<EncoderKey>( node.config );
    // Skip jpeg/png encoders - they don't use createEncoderBin with videoconvert
    if ( key.codec == "h264" || key.codec == "h265" ) {
      out.push_back( node_id );
    }
    return;
  }
  for ( NodeId child : node.outputs ) {
    collectDownstreamEncoders( graph, child, out, has_raw_sink );
  }
}

void PipelineBuilder::determineHwScaling( const PipelineGraph &graph )
{
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type != GraphNodeType::Scale )
      continue;

    std::vector<NodeId> downstream_encoders;
    bool has_raw_sink = false;
    for ( NodeId child : node.outputs ) {
      collectDownstreamEncoders( graph, child, downstream_encoders, has_raw_sink );
    }

    if ( has_raw_sink || downstream_encoders.empty() ) {
      SERVER_LOG_DEBUG( "Scale node %d: software scaling (raw_sink=%s, encoders=%zu)", id,
                        has_raw_sink ? "true" : "false", downstream_encoders.size() );
      continue;
    }

    // Compute common memory flags across all downstream encoders
    uint32_t common = 0xFFFFFFFF;
    for ( NodeId enc_id : downstream_encoders ) {
      auto it = encoder_preselected_.find( enc_id );
      if ( it == encoder_preselected_.end() ) {
        common = 0;
        break;
      }
      auto backend_flag = static_cast<CodecBackendFlag>( it->second );
      common &= static_cast<uint32_t>( encoderInputMemoryFlags( backend_flag ) );
    }

    if ( common == 0 ) {
      SERVER_LOG_DEBUG( "Scale node %d: software scaling (no common HW memory across %zu encoders)",
                        id, downstream_encoders.size() );
      continue;
    }

    scale_target_memory_[id] = static_cast<MemoryFeature>( common );
    SERVER_LOG_DEBUG( "Scale node %d: target HW memory 0x%x for %zu downstream encoder(s)", id,
                      common, downstream_encoders.size() );
  }
}

std::string PipelineBuilder::getEncoderName( GstElement *element )
{
  if ( !element )
    return "";

  // If it's a bin, look for a child element named "encoder"
  if ( GST_IS_BIN( element ) ) {
    GstElement *encoder = gst_bin_get_by_name( GST_BIN( element ), "encoder" );
    if ( encoder ) {
      GstElementFactory *factory = gst_element_get_factory( encoder );
      std::string name =
          factory ? gst_plugin_feature_get_name( GST_PLUGIN_FEATURE( factory ) ) : "unknown";
      gst_object_unref( encoder );
      return name;
    }
  }

  // Direct element
  GstElementFactory *factory = gst_element_get_factory( element );
  return factory ? gst_plugin_feature_get_name( GST_PLUGIN_FEATURE( factory ) ) : "unknown";
}

std::string PipelineBuilder::findEncoderForSink( const PipelineGraph &graph, NodeId sink_id )
{
  NodeId encoder_id = findEncoderNodeForSink( graph, sink_id );
  if ( encoder_id < 0 )
    return "";
  auto it = encoder_names_.find( encoder_id );
  return it != encoder_names_.end() ? it->second : "";
}

NodeId PipelineBuilder::findEncoderNodeForSink( const PipelineGraph &graph, NodeId sink_id )
{
  // Walk back from sink to find encoder node
  NodeId current = sink_id;
  while ( true ) {
    const GraphNode &node = graph.getNode( current );
    if ( node.type == GraphNodeType::Encoder ) {
      return current;
    }
    if ( !node.input.has_value() ) {
      break;
    }
    current = node.input.value();
  }
  return -1;
}

void PipelineBuilder::linkWithQueue( GstElement *src, GstElement *dest, const std::string &queue_name,
                                     bool is_interdependent_format, GstElement *valve,
                                     std::shared_ptr<const DiagnosticContext> context )
{
  if ( !src || !dest || !GST_IS_ELEMENT( src ) || !GST_IS_ELEMENT( dest ) ) {
    std::string src_name = getElementName( src );
    std::string dest_name = getElementName( dest );
    std::string error_msg = "Cannot link elements: src='" + src_name + "' (" +
                            std::to_string( (uintptr_t)src ) + ") or dest='" + dest_name + "' (" +
                            std::to_string( (uintptr_t)dest ) + ") is invalid or NULL.";
    SERVER_LOG_ERROR( "%s", error_msg.c_str() );
    throw PipelineBuildError( error_msg );
  }

  std::string src_name = getElementName( src );
  std::string dest_name = getElementName( dest );
  SERVER_LOG_DEBUG( "Linking %s -> queue(%s) -> %s (interdependent=%s)", src_name.c_str(),
                    queue_name.c_str(), dest_name.c_str(),
                    is_interdependent_format ? "true" : "false" );

  // Get source pad (request from tee if it's a tee, otherwise static)
  GstPad *src_pad = nullptr;
  bool src_pad_is_request = false;

  if ( isTee( src ) ) {
    src_pad = gst_element_request_pad_simple( src, "src_%u" );
    src_pad_is_request = true;
  } else {
    src_pad = gst_element_get_static_pad( src, "src" );
  }

  if ( !src_pad ) {
    throw PipelineBuildError( "Failed to get source pad for linking '" + src_name + "' to '" +
                              dest_name + "'" );
  }
  if ( src_pad_is_request ) {
    // Record the request pad so CameraPipeline can release it before unrefing
    // the parent pipeline. The +1 caller-owned ref is dropped below; the tee
    // keeps the pad alive until release_request_pad is called.
    tee_request_pads_.emplace_back( src, src_pad );
  }

  // Create queue
  GstElement *queue = gst_element_factory_make( "queue", queue_name.c_str() );
  if ( !queue ) {
    gst_object_unref( src_pad );
    throw PipelineBuildError( "Failed to create queue '" + queue_name + "' for linking '" +
                              src_name + "' to '" + dest_name + "'" );
  }
  attachDiagnosticContext( G_OBJECT( queue ), context );

  // Configure queue based on frame format characteristics
  if ( is_interdependent_format ) {
    // Inter-dependent formats (H.264/H.265): P-frames depend on previous frames
    // Use max-size-time limit without leaky to block if the downstream can't keep up, making upstream queues drop frames instead.
    g_object_set( queue, "max-size-time", (guint64)100'000'000 /* 100ms */, "leaky",
                  2 /* downstream */, nullptr );
  } else {
    // Independent formats (raw video, JPEG): each frame is self-contained
    // Use aggressive single-buffer limit to minimize latency
    g_object_set( queue, "max-size-buffers", 1, "leaky", 2 /* downstream */, nullptr );
  }
  gst_bin_add( pipeline_, queue );

  if ( valve ) {
    // Insert valve between source and queue: src_pad → valve → queue → dest
    gst_bin_add( pipeline_, valve );
    GstPad *valve_sink = gst_element_get_static_pad( valve, "sink" );
    if ( !valve_sink ) {
      gst_object_unref( src_pad );
      throw PipelineBuildError( "Failed to get sink pad for valve '" + getElementName( valve ) + "'" );
    }
    if ( gst_pad_link( src_pad, valve_sink ) != GST_PAD_LINK_OK ) {
      gst_object_unref( src_pad );
      gst_object_unref( valve_sink );
      throw PipelineBuildError( "Failed to link source pad of '" + src_name + "' to valve sink." );
    }
    gst_object_unref( src_pad );
    gst_object_unref( valve_sink );

    if ( !gst_element_link( valve, queue ) ) {
      throw PipelineBuildError( "Failed to link valve to queue '" + queue_name + "'." );
    }
  } else {
    // Link directly: src_pad → queue
    GstPad *q_sink = gst_element_get_static_pad( queue, "sink" );
    if ( !q_sink ) {
      gst_object_unref( src_pad );
      throw PipelineBuildError( "Failed to get sink pad for queue '" + queue_name + "'" );
    }
    if ( gst_pad_link( src_pad, q_sink ) != GST_PAD_LINK_OK ) {
      gst_object_unref( src_pad );
      gst_object_unref( q_sink );
      throw PipelineBuildError( "Failed to link source pad of '" + src_name + "' to queue sink." );
    }
    gst_object_unref( src_pad );
    gst_object_unref( q_sink );
  }

  // Link queue -> dest
  if ( !gst_element_link( queue, dest ) ) {
    throw PipelineBuildError( "Failed to link queue '" + queue_name + "' to destination '" +
                              dest_name + "'." );
  }
}

bool PipelineBuilder::isTee( GstElement *element )
{
  if ( !element )
    return false;

  GstElementFactory *factory = gst_element_get_factory( element );
  if ( !factory )
    return false;

  const gchar *factory_name = gst_plugin_feature_get_name( GST_PLUGIN_FEATURE( factory ) );
  return factory_name && g_strcmp0( factory_name, "tee" ) == 0;
}

std::string PipelineBuilder::getElementName( GstElement *element )
{
  if ( !element || !GST_IS_ELEMENT( element ) ) {
    return "NULL";
  }
  gchar *name = gst_element_get_name( element );
  if ( !name ) {
    return "unknown";
  }
  std::string result( name );
  g_free( name );
  return result;
}

std::string PipelineBuilder::nodeTypeToString( GraphNodeType type )
{
  switch ( type ) {
  case GraphNodeType::Source:
    return "Source";
  case GraphNodeType::Decoder:
    return "Decoder";
  case GraphNodeType::FramerateLimit:
    return "FramerateLimit";
  case GraphNodeType::Scale:
    return "Scale";
  case GraphNodeType::Encoder:
    return "Encoder";
  case GraphNodeType::Sink:
    return "Sink";
  default:
    return "Unknown";
  }
}

std::shared_ptr<const DiagnosticContext>
PipelineBuilder::makeDiagnosticContext( const PipelineGraph &graph, const GraphNode &node ) const
{
  auto context = std::make_shared<DiagnosticContext>();
  context->camera_id = camera_id_;
  context->graph_node_id = node.id;
  if ( node.type == GraphNodeType::Sink ) {
    const SinkConfig &sink_config = std::get<SinkConfig>( node.config );
    context->affected_outputs = { static_cast<size_t>( sink_config.output_index ) };
  } else {
    collectDownstreamOutputs( graph, node.id, context->affected_outputs );
  }
  return context;
}

std::vector<NodeId> PipelineBuilder::topologicalSort( const PipelineGraph &graph )
{
  std::vector<NodeId> result;
  std::set<NodeId> visited;
  std::set<NodeId> in_stack;

  std::function<void( NodeId )> visit = [&]( NodeId id ) {
    if ( visited.count( id ) ) {
      return;
    }
    if ( in_stack.count( id ) ) {
      throw PipelineBuildError( "Cycle detected in pipeline graph" );
    }

    in_stack.insert( id );

    const GraphNode &node = graph.getNode( id );
    if ( node.input.has_value() ) {
      visit( node.input.value() );
    }

    in_stack.erase( id );
    visited.insert( id );
    result.push_back( id );
  };

  for ( const auto &[id, node] : graph.nodes ) { visit( id ); }

  return result;
}

void PipelineBuilder::collectDownstreamOutputs( const PipelineGraph &graph, NodeId node_id,
                                                std::vector<size_t> &out )
{
  const GraphNode &node = graph.getNode( node_id );
  if ( node.type == GraphNodeType::Sink ) {
    out.push_back( static_cast<size_t>( std::get<SinkConfig>( node.config ).output_index ) );
    return;
  }
  for ( NodeId child : node.outputs ) { collectDownstreamOutputs( graph, child, out ); }
}

std::string PipelineBuilder::dumpPipeline( const PipelineGraph &graph )
{
  std::stringstream ss;

  // Helper to get element description
  auto elementDescription = [this, &graph]( NodeId id ) -> std::string {
    const GraphNode &node = graph.getNode( id );
    GstElement *element = realized_elements_.count( id ) ? realized_elements_.at( id ) : nullptr;

    std::string desc;
    if ( element ) {
      gchar *name = gst_element_get_name( element );
      desc = std::string( name );
      g_free( name );
    } else if ( node.type == GraphNodeType::Sink ) {
      const SinkConfig &cfg = std::get<SinkConfig>( node.config );
      desc = "Sink[" + std::to_string( cfg.output_index + 1 ) + "]";
    } else {
      desc = "NULL";
    }

    if ( output_tees_.count( id ) ) {
      gchar *tee_name = gst_element_get_name( output_tees_.at( id ) );
      desc += " -> " + std::string( tee_name );
      g_free( tee_name );
    }
    return desc;
  };

  // Recursive function to print tree
  std::function<void( NodeId, const std::string &, bool )> printNode =
      [&]( NodeId id, const std::string &prefix, bool is_last ) {
        const GraphNode &node = graph.getNode( id );
        ss << prefix;
        ss << ( is_last ? "└── " : "├── " );
        ss << elementDescription( id ) << "\n";

        std::string child_prefix = prefix + ( is_last ? "    " : "│   " );
        for ( size_t i = 0; i < node.outputs.size(); ++i ) {
          printNode( node.outputs[i], child_prefix, i == node.outputs.size() - 1 );
        }
      };

  ss << "Realized GStreamer Pipeline Structure:\n";
  printNode( graph.source_node, "", true );

  return ss.str();
}

} // namespace ros_camera_server
