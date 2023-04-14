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

#ifndef ROS_CAMERA_SERVER_PIPELINE_GRAPH_HPP
#define ROS_CAMERA_SERVER_PIPELINE_GRAPH_HPP

#include "ros_camera_server/configuration.hpp"
#include "ros_camera_server/transcoding/encoder_key.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace ros_camera_server
{

using NodeId = int32_t;

enum class GraphNodeType {
  Source,         // Input source (root of graph)
  Decoder,        // Format decoder (JPEG->RAW, PNG->RAW)
  FramerateLimit, // videorate with drop-only (reduces framerate to upper limit)
  Scale,          // videoconvertscale (changes resolution to upper limit)
  Encoder,        // RAW -> H264/H265
  Sink            // Output sink (leaf of graph, references output index)
};

struct FramerateKey {
  Framerate max_framerate;

  bool operator==( const FramerateKey &other ) const
  { return max_framerate == other.max_framerate; }
};

struct ScaleKey {
  int max_width;
  int max_height;

  bool operator==( const ScaleKey &other ) const
  { return max_width == other.max_width && max_height == other.max_height; }
};

struct SourceConfig {
  std::string type;

  bool operator==( const SourceConfig &other ) const { return type == other.type; }
};

struct SinkConfig {
  int output_index;
  std::string type;

  bool operator==( const SinkConfig &other ) const
  { return output_index == other.output_index && type == other.type; }
};

// Node configuration types
using NodeConfig =
    std::variant<SourceConfig, StreamFormat, FramerateKey, ScaleKey, EncoderKey, SinkConfig>;

struct GraphNode {
  NodeId id;
  GraphNodeType type;
  NodeConfig config;
  std::optional<NodeId> input; // Single parent (tree structure, nullopt for Source)
  std::vector<NodeId> outputs; // Children nodes
};

struct PipelineGraph {
  NodeId source_node = 0;
  std::vector<NodeId> sink_nodes;
  std::unordered_map<NodeId, GraphNode> nodes;

  // Access nodes
  const GraphNode &getNode( NodeId id ) const { return nodes.at( id ); }
  GraphNode &getNode( NodeId id ) { return nodes.at( id ); }

  // Build the graph from input format and output configurations
  static PipelineGraph build( StreamFormat input_format, const std::string &input_type,
                              const std::vector<std::shared_ptr<OutputConfiguration>> &outputs );

  // Resolve the actual StreamFormat that a given sink node receives
  StreamFormat resolveOutputFormat( NodeId sink_id, StreamFormat input_format ) const;

  // Generate a human-readable representation of the graph
  std::string toString() const;
};

} // namespace ros_camera_server

// Hash specializations
template<>
struct std::hash<ros_camera_server::FramerateKey> {
  std::size_t operator()( const ros_camera_server::FramerateKey &key ) const
  {
    return std::hash<int>()( key.max_framerate.numerator ) ^
           ( std::hash<int>()( key.max_framerate.denominator ) << 1 );
  }
};

template<>
struct std::hash<ros_camera_server::ScaleKey> {
  std::size_t operator()( const ros_camera_server::ScaleKey &key ) const
  { return std::hash<int>()( key.max_width ) ^ ( std::hash<int>()( key.max_height ) << 1 ); }
};

#endif // ROS_CAMERA_SERVER_PIPELINE_GRAPH_HPP
