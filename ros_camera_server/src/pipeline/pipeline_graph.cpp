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

#include "pipeline_graph.hpp"

#include "../logging.hpp"
#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace ros_camera_server
{

namespace
{

struct ChainElement {
  GraphNodeType type;
  NodeConfig config;
};

struct OutputChain {
  int output_index;
  std::string sink_type;
  std::vector<ChainElement> elements; // Source and Sink are implicit
};

// Check if format is compressed (requires decoding to apply transforms)
bool isCompressedFormat( StreamFormat format )
{
  return format == StreamFormat::JPEG || format == StreamFormat::PNG ||
         format == StreamFormat::H264 || format == StreamFormat::H265;
}

// Check if output supports the given format
bool supportsFormat( const OutputConfiguration &config, StreamFormat format )
{
  return std::find( config.supported_input_formats.begin(), config.supported_input_formats.end(),
                    format ) != config.supported_input_formats.end();
}

// Determine the target compressed format for an output, or INVALID if none wanted
StreamFormat targetCompressedFormat( const OutputConfiguration &config )
{
  if ( supportsFormat( config, StreamFormat::H265 ) )
    return StreamFormat::H265;
  if ( supportsFormat( config, StreamFormat::H264 ) )
    return StreamFormat::H264;
  if ( supportsFormat( config, StreamFormat::JPEG ) )
    return StreamFormat::JPEG;
  if ( supportsFormat( config, StreamFormat::PNG ) )
    return StreamFormat::PNG;
  return StreamFormat::INVALID;
}

// Phase 1: Build a simple linear chain for a single output
OutputChain computeChain( int index, const OutputConfiguration &config, StreamFormat input_format,
                          const std::string &input_decoder )
{
  OutputChain chain;
  chain.output_index = index;
  chain.sink_type = config.type;

  bool needs_scale = ( config.width != std::numeric_limits<int>::max() && config.width > 0 ) ||
                     ( config.height != std::numeric_limits<int>::max() && config.height > 0 );
  bool needs_framerate = config.framerate.isValid();

  // Determine what the output wants
  StreamFormat target_compressed = targetCompressedFormat( config );
  bool wants_compressed = target_compressed != StreamFormat::INVALID;

  // Passthrough: output supports the input format and no transforms needed
  if ( !needs_scale && !needs_framerate && supportsFormat( config, input_format ) ) {
    return chain; // empty chain = passthrough
  }

  // Determine if we need to decode the input
  bool input_compressed = isCompressedFormat( input_format );
  bool needs_decode = false;
  if ( input_compressed ) {
    if ( needs_scale ) {
      // Must decode to scale
      needs_decode = true;
    } else if ( wants_compressed && target_compressed != input_format ) {
      // Transcoding: different codec
      needs_decode = true;
    } else if ( !supportsFormat( config, input_format ) ) {
      // Output doesn't support the input format -> must decode
      needs_decode = true;
    }
  }

  // After this point, determine if we need encoding
  // Stream is RAW if: input was RAW, or we decoded
  bool stream_is_raw = ( input_format == StreamFormat::RAW ) || needs_decode;
  bool needs_encode = wants_compressed && stream_is_raw;

  // Build the chain: [FramerateLimit] -> [Decoder] -> [Scale] -> [Encoder]
  if ( needs_framerate ) {
    chain.elements.push_back( { GraphNodeType::FramerateLimit, FramerateKey{ config.framerate } } );
  }

  if ( needs_decode ) {
    chain.elements.push_back( { GraphNodeType::Decoder, DecoderKey{ input_format, input_decoder } } );
  }

  if ( needs_scale ) {
    chain.elements.push_back( { GraphNodeType::Scale, ScaleKey{ config.width, config.height } } );
  }

  if ( needs_encode ) {
    EncoderKey key = config.createEncoderKey();
    // For JPEG/PNG encoding, override the codec in the key
    if ( target_compressed == StreamFormat::JPEG ) {
      key.codec = "jpeg";
    } else if ( target_compressed == StreamFormat::PNG ) {
      key.codec = "png";
    }
    chain.elements.push_back( { GraphNodeType::Encoder, key } );
  }

  return chain;
}

// Check if f1 is an integer multiple of f2
bool isMultiple( const Framerate &f1, const Framerate &f2 )
{
  if ( !f1.isValid() || !f2.isValid() )
    return false;
  // f1 is multiple of f2 if (n1/d1) / (n2/d2) is integer -> (n1 * d2) % (d1 * n2) == 0
  uint64_t num = static_cast<uint64_t>( f1.numerator ) * f2.denominator;
  uint64_t den = static_cast<uint64_t>( f1.denominator ) * f2.numerator;
  return num % den == 0;
}

// Find the index of the Scale element in a chain, or -1 if none
int findScaleIndex( const OutputChain &chain )
{
  for ( size_t i = 0; i < chain.elements.size(); ++i ) {
    if ( chain.elements[i].type == GraphNodeType::Scale )
      return static_cast<int>( i );
  }
  return -1;
}

// Find the index of the first FramerateLimit element before a given position, or -1
int findFramerateBefore( const OutputChain &chain, int before_pos )
{
  for ( int i = 0; i < before_pos; ++i ) {
    if ( chain.elements[i].type == GraphNodeType::FramerateLimit )
      return i;
  }
  return -1;
}

// Phase 2 Step A: Optimize framerate/scale groups
// Chains with the same ScaleKey can share a scale node if their framerates are compatible.
// This function rewrites chains to enable sharing by moving framerate elements around.
void optimizeFramerateScaleGroups( std::vector<OutputChain> &chains )
{
  // Group chain indices by their ScaleKey
  std::map<uint64_t, std::vector<size_t>> scale_groups;

  for ( size_t ci = 0; ci < chains.size(); ++ci ) {
    int scale_idx = findScaleIndex( chains[ci] );
    if ( scale_idx < 0 )
      continue;
    const ScaleKey &sk = std::get<ScaleKey>( chains[ci].elements[scale_idx].config );
    uint64_t key = ( static_cast<uint64_t>( sk.max_width ) << 32 ) |
                   ( static_cast<uint32_t>( sk.max_height ) );
    scale_groups[key].push_back( ci );
  }

  for ( auto &[res_key, group_indices] : scale_groups ) {
    if ( group_indices.size() <= 1 )
      continue;

    // Check if any chain in the group has no framerate before scale
    bool some_no_limit = false;
    for ( size_t ci : group_indices ) {
      int scale_idx = findScaleIndex( chains[ci] );
      int fr_idx = findFramerateBefore( chains[ci], scale_idx );
      if ( fr_idx < 0 ) {
        some_no_limit = true;
        break;
      }
    }

    if ( some_no_limit ) {
      // Move all pre-scale framerate elements to after-scale
      for ( size_t ci : group_indices ) {
        int scale_idx = findScaleIndex( chains[ci] );
        int fr_idx = findFramerateBefore( chains[ci], scale_idx );
        if ( fr_idx >= 0 ) {
          // Remove FR from before scale, insert after scale
          ChainElement fr_elem = chains[ci].elements[fr_idx];
          chains[ci].elements.erase( chains[ci].elements.begin() + fr_idx );
          // scale_idx shifted by -1 after erase
          int new_scale_idx = findScaleIndex( chains[ci] );
          chains[ci].elements.insert( chains[ci].elements.begin() + new_scale_idx + 1, fr_elem );
        }
      }
      continue;
    }

    // All chains have a framerate before scale. Partition into compatible sub-groups.
    // Collect (framerate, chain_index) pairs
    struct FrEntry {
      Framerate fr;
      size_t chain_idx;
    };
    std::vector<FrEntry> entries;
    for ( size_t ci : group_indices ) {
      int scale_idx = findScaleIndex( chains[ci] );
      int fr_idx = findFramerateBefore( chains[ci], scale_idx );
      const FramerateKey &fk = std::get<FramerateKey>( chains[ci].elements[fr_idx].config );
      entries.push_back( { fk.max_framerate, ci } );
    }

    // Sort descending by framerate
    std::sort( entries.begin(), entries.end(),
               []( const FrEntry &a, const FrEntry &b ) { return a.fr > b.fr; } );

    // Greedy grouping: assign each to first compatible sub-group
    struct SubGroup {
      Framerate leader; // highest framerate (first assigned)
      std::vector<FrEntry> members;
    };
    std::vector<SubGroup> sub_groups;

    for ( const auto &entry : entries ) {
      bool assigned = false;
      for ( auto &sg : sub_groups ) {
        if ( isMultiple( sg.leader, entry.fr ) ) {
          sg.members.push_back( entry );
          assigned = true;
          break;
        }
      }
      if ( !assigned ) {
        sub_groups.push_back( { entry.fr, { entry } } );
      }
    }

    // For each sub-group, rewrite chains: use leader's FR before scale, add sub-FR after
    for ( const auto &sg : sub_groups ) {
      if ( sg.members.size() <= 1 )
        continue;

      for ( const auto &member : sg.members ) {
        size_t ci = member.chain_idx;
        int scale_idx = findScaleIndex( chains[ci] );
        int fr_idx = findFramerateBefore( chains[ci], scale_idx );

        Framerate original_fr =
            std::get<FramerateKey>( chains[ci].elements[fr_idx].config ).max_framerate;

        // Replace pre-scale FR with leader's FR
        chains[ci].elements[fr_idx].config = FramerateKey{ sg.leader };

        // If original was lower than leader, add sub-FR after scale
        if ( original_fr < sg.leader ) {
          int new_scale_idx = findScaleIndex( chains[ci] );
          // GCC false positive on std::variant move-ctor analysis with multiple
          // string-bearing alternatives — variant init is correct here.
#if defined( __GNUC__ ) && !defined( __clang__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
          chains[ci].elements.insert( chains[ci].elements.begin() + new_scale_idx + 1,
                                      { GraphNodeType::FramerateLimit, FramerateKey{ original_fr } } );
#if defined( __GNUC__ ) && !defined( __clang__ )
#pragma GCC diagnostic pop
#endif
        }
      }
    }
  }
}

class GraphBuilder
{
public:
  GraphBuilder( StreamFormat input_format, std::string input_type, std::string input_decoder )
      : input_format_( input_format ), input_type_( std::move( input_type ) ),
        input_decoder_( std::move( input_decoder ) )
  {
  }

  PipelineGraph build( const std::vector<std::shared_ptr<OutputConfiguration>> &outputs )
  {
    // Always create source node
    NodeId source = createNode( GraphNodeType::Source, SourceConfig{ input_type_ } );
    graph_.source_node = source;

    if ( outputs.empty() ) {
      return graph_;
    }

    // Phase 1: Build simple per-output chains
    std::vector<OutputChain> chains;
    for ( size_t i = 0; i < outputs.size(); ++i ) {
      chains.push_back(
          computeChain( static_cast<int>( i ), *outputs[i], input_format_, input_decoder_ ) );
    }

    // Phase 2: Optimize and materialize
    optimizeFramerateScaleGroups( chains );
    materializeAndMerge( source, chains );

    return graph_;
  }

private:
  void materializeAndMerge( NodeId source, const std::vector<OutputChain> &chains )
  {
    // For each chain, track the current parent node
    std::vector<NodeId> current_parents( chains.size(), source );

    // Find max chain depth
    size_t max_depth = 0;
    for ( const auto &chain : chains ) { max_depth = std::max( max_depth, chain.elements.size() ); }

    // Process level by level to enable merging
    for ( size_t depth = 0; depth < max_depth; ++depth ) {
      for ( size_t ci = 0; ci < chains.size(); ++ci ) {
        if ( depth >= chains[ci].elements.size() )
          continue;

        const ChainElement &elem = chains[ci].elements[depth];
        NodeId parent = current_parents[ci];

        current_parents[ci] = findOrCreateNode( parent, elem );
      }
    }

    // Attach sinks
    for ( size_t ci = 0; ci < chains.size(); ++ci ) {
      NodeId sink = createSink( chains[ci].output_index, chains[ci].sink_type );
      linkNode( sink, current_parents[ci] );
    }
  }

  NodeId findOrCreateNode( NodeId parent, const ChainElement &elem )
  {
    // Check existing children of parent for a match
    const auto &parent_node = graph_.nodes[parent];
    for ( NodeId child_id : parent_node.outputs ) {
      const auto &child = graph_.nodes[child_id];
      if ( child.type == elem.type && child.config == elem.config ) {
        return child_id;
      }
    }
    // No match, create new node
    NodeId new_node = createNode( elem.type, elem.config );
    linkNode( new_node, parent );
    return new_node;
  }

  NodeId createNode( GraphNodeType type, NodeConfig config )
  {
    NodeId id = next_node_id_++;
    graph_.nodes[id] = GraphNode{ id, type, std::move( config ), std::nullopt, {} };
    return id;
  }

  NodeId createSink( int output_index, std::string output_type )
  {
    NodeId id =
        createNode( GraphNodeType::Sink, SinkConfig{ output_index, std::move( output_type ) } );
    graph_.sink_nodes.push_back( id );
    return id;
  }

  void linkNode( NodeId child, NodeId parent )
  {
    graph_.nodes[child].input = parent;
    graph_.nodes[parent].outputs.push_back( child );
  }

  StreamFormat input_format_;
  std::string input_type_;
  std::string input_decoder_;
  PipelineGraph graph_;
  NodeId next_node_id_ = 0;
};

} // namespace

PipelineGraph PipelineGraph::build( StreamFormat input_format, const std::string &input_type,
                                    const std::vector<std::shared_ptr<OutputConfiguration>> &outputs,
                                    const std::string &input_decoder )
{
  GraphBuilder builder( input_format, input_type, input_decoder );
  return builder.build( outputs );
}

StreamFormat PipelineGraph::resolveOutputFormat( NodeId sink_id, StreamFormat input_format ) const
{
  NodeId current = sink_id;
  while ( true ) {
    const GraphNode &node = getNode( current );
    switch ( node.type ) {
    case GraphNodeType::Encoder: {
      const EncoderKey &key = std::get<EncoderKey>( node.config );
      return stream_format_from_codec( key.codec );
    }
    case GraphNodeType::Decoder:
    case GraphNodeType::Scale:
      // These nodes produce RAW video
      return StreamFormat::RAW;
    case GraphNodeType::FramerateLimit:
      // FramerateLimit passes through format - keep walking
      break;
    case GraphNodeType::Source:
      // Passthrough: output receives the input format directly
      return input_format;
    default:
      break;
    }
    if ( !node.input.has_value() )
      break;
    current = node.input.value();
  }
  return StreamFormat::INVALID;
}

std::string PipelineGraph::toString() const
{
  std::stringstream ss;

  // Helper to get node description
  auto nodeDescription = [this]( NodeId id ) -> std::string {
    const GraphNode &node = getNode( id );
    const std::string str_id = "[" + std::to_string( id ) + "]";
    switch ( node.type ) {
    case GraphNodeType::Source: {
      const SourceConfig &cfg = std::get<SourceConfig>( node.config );
      return str_id + " Source" + ( cfg.type.empty() ? "" : "(" + cfg.type + ")" );
    }
    case GraphNodeType::Decoder: {
      const DecoderKey &key = std::get<DecoderKey>( node.config );
      std::string s = str_id + " Decode(" + ros_camera_server::to_string( key.format );
      if ( !key.decoder.empty() && key.decoder != "auto" )
        s += ", " + key.decoder;
      s += ")";
      return s;
    }
    case GraphNodeType::FramerateLimit: {
      const FramerateKey &key = std::get<FramerateKey>( node.config );
      return str_id + " Framerate(" + key.max_framerate.toString() + ")";
    }
    case GraphNodeType::Scale: {
      const ScaleKey &key = std::get<ScaleKey>( node.config );
      std::string w =
          key.max_width == std::numeric_limits<int>::max() ? "*" : std::to_string( key.max_width );
      std::string h =
          key.max_height == std::numeric_limits<int>::max() ? "*" : std::to_string( key.max_height );
      return str_id + " Scale(" + w + "x" + h + ")";
    }
    case GraphNodeType::Encoder: {
      const EncoderKey &key = std::get<EncoderKey>( node.config );
      return str_id + " Encode(" + key.codec + ", " +
             ( key.encoder.empty() ? "auto" : key.encoder ) + ")";
    }
    case GraphNodeType::Sink: {
      const SinkConfig &cfg = std::get<SinkConfig>( node.config );
      return str_id + " Output" + std::to_string( cfg.output_index + 1 ) +
             ( cfg.type.empty() ? "" : "(" + cfg.type + ")" );
    }
    default:
      return "Unknown";
    }
  };

  // Recursive function to print tree
  std::function<void( NodeId, const std::string &, bool )> printNode =
      [&]( NodeId id, const std::string &prefix, bool is_last ) {
        const GraphNode &node = getNode( id );
        ss << prefix;
        ss << ( is_last ? "└── " : "├── " );
        ss << nodeDescription( id ) << "\n";

        std::string child_prefix = prefix + ( is_last ? "    " : "│   " );
        for ( size_t i = 0; i < node.outputs.size(); ++i ) {
          printNode( node.outputs[i], child_prefix, i == node.outputs.size() - 1 );
        }
      };

  ss << "Pipeline Graph:\n";
  printNode( source_node, "", true );

  return ss.str();
}

} // namespace ros_camera_server
