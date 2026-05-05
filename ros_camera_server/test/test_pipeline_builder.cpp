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

#include "pipeline/pipeline_builder.hpp"
#include "pipeline/pipeline_graph.hpp"
#include "pipeline_test_helpers.hpp"

#include <gtest/gtest.h>

using namespace ros_camera_server;

// -----------------------------------------------------------------------------
// Graph Building Tests (No GStreamer Required)
// -----------------------------------------------------------------------------
class PipelineGraphTest : public ::testing::Test
{
protected:
  // Helper to count nodes of a specific type
  int countNodesByType( const PipelineGraph &graph, GraphNodeType type )
  {
    int count = 0;
    for ( const auto &[id, node] : graph.nodes ) {
      if ( node.type == type ) {
        count++;
      }
    }
    return count;
  }

  // Helper to find a node by type
  std::optional<NodeId> findNodeByType( const PipelineGraph &graph, GraphNodeType type )
  {
    for ( const auto &[id, node] : graph.nodes ) {
      if ( node.type == type ) {
        return id;
      }
    }
    return std::nullopt;
  }
};

TEST_F( PipelineGraphTest, EmptyOutputsProducesMinimalGraph )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Source ), 1 );
  EXPECT_TRUE( graph.sink_nodes.empty() );
}

TEST_F( PipelineGraphTest, ToStringAnnotatesSourceAndSinkTypes )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );
  const std::string s = graph.toString();

  // Source node carries input type
  EXPECT_NE( s.find( "Source(v4l2)" ), std::string::npos ) << s;
  // TestOutputConfiguration uses type "test"
  EXPECT_NE( s.find( "Output1(test)" ), std::string::npos ) << s;
}

TEST_F( PipelineGraphTest, ToStringOmitsParensWhenTypeEmpty )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "", outputs );
  const std::string s = graph.toString();

  EXPECT_NE( s.find( " Source\n" ), std::string::npos ) << s;
  EXPECT_EQ( s.find( "Source(" ), std::string::npos ) << s;
}

TEST_F( PipelineGraphTest, SingleRawOutputNoTransform )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Source ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 0 );
}

TEST_F( PipelineGraphTest, JpegInputRequiresDecoder )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::JPEG, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 1 );
}

TEST_F( PipelineGraphTest, H264InputToRawNeedsDecoder )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::H264, "rtp", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 1 );
  auto decoder_id = findNodeByType( graph, GraphNodeType::Decoder );
  ASSERT_TRUE( decoder_id.has_value() );
  const auto &key = std::get<DecoderKey>( graph.getNode( *decoder_id ).config );
  EXPECT_EQ( key.format, StreamFormat::H264 );
  EXPECT_EQ( key.decoder, "auto" );
}

TEST_F( PipelineGraphTest, H265InputToRawNeedsDecoder )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::H265, "rtp", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 1 );
  auto decoder_id = findNodeByType( graph, GraphNodeType::Decoder );
  ASSERT_TRUE( decoder_id.has_value() );
  const auto &key = std::get<DecoderKey>( graph.getNode( *decoder_id ).config );
  EXPECT_EQ( key.format, StreamFormat::H265 );
}

TEST_F( PipelineGraphTest, H264PassthroughNoDecoder )
{
  // H264 input to H264 output without transforms should bypass decoding entirely.
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H264 ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::H264, "rtp", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 0 );
}

TEST_F( PipelineGraphTest, H264TranscodeToH265InsertsDecodeAndEncode )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H265, 640, 480, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::H264, "rtp", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 1 );
}

TEST_F( PipelineGraphTest, DecoderPreferencePropagatesToDecoderKey )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::H264, "rtp", outputs, "nv|sw" );

  auto decoder_id = findNodeByType( graph, GraphNodeType::Decoder );
  ASSERT_TRUE( decoder_id.has_value() );
  const auto &key = std::get<DecoderKey>( graph.getNode( *decoder_id ).config );
  EXPECT_EQ( key.decoder, "nv|sw" );
}

TEST_F( PipelineGraphTest, H264OutputRequiresEncoder )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 1 );
}

TEST_F( PipelineGraphTest, ScalingCreatesScaleNode )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480 ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );
}

TEST_F( PipelineGraphTest, FramerateLimitCreatesFramerateNode )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 0, 0, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 1 );
}

TEST_F( PipelineGraphTest, SameResolutionSharesScale )
{
  // Two outputs with same resolution should share scale node
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 15, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // Should have exactly one scale node (shared)
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );
  // Should have two framerate nodes (one per output)
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 2 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 2 );
}

TEST_F( PipelineGraphTest, SameFramerateSharesFramerateLimiter )
{
  // Two outputs with same framerate but different resolutions should share framerate
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 320, 240, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // Should have exactly one framerate node (shared)
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 1 );
  // Should have two scale nodes (different resolutions)
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 2 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 2 );
}

TEST_F( PipelineGraphTest, DifferentResolutionsAndFrameratesBranchEarly )
{
  // Two outputs with different resolutions AND different framerates
  // Should branch early with separate chains
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 320, 240, Framerate( 15, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // Should have separate framerate and scale nodes for each output
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 2 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 2 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 2 );
}

TEST_F( PipelineGraphTest, SameResolutionAndFramerateSharesEverything )
{
  // Two outputs with identical transform requirements should share everything
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::FramerateLimit ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 2 );
}

TEST_F( PipelineGraphTest, EncodersSameKeyShareEncoder )
{
  // Two H264 outputs with same parameters should share encoder
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  auto config1 = createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 30, 1 ) );
  auto config2 = createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 30, 1 ) );
  outputs.push_back( config1 );
  outputs.push_back( config2 );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // Should share the encoder
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 2 );
}

TEST_F( PipelineGraphTest, SharedChainPutsFramerateBeforeScale )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  auto scale_id = findNodeByType( graph, GraphNodeType::Scale );
  auto rate_id = findNodeByType( graph, GraphNodeType::FramerateLimit );
  ASSERT_TRUE( scale_id.has_value() );
  ASSERT_TRUE( rate_id.has_value() );

  // Rate should be parent of Scale (optimization: limit fps before scaling)
  EXPECT_EQ( graph.nodes.at( *scale_id ).input, rate_id );
}

TEST_F( PipelineGraphTest, SameResolutionDifferentFrameratesMultipleSharesScale )
{
  // 30fps and 15fps: 30 is multiple of 15. Should share scale.
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 15, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );
}

TEST_F( PipelineGraphTest, JpegInputJpegOutputPassthrough )
{
  // JPEG input to JPEG output with no transforms should be a passthrough
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::JPEG ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::JPEG, "v4l2", outputs );

  // No decoder or encoder - direct passthrough
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 0 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Sink ), 1 );

  // Sink should be directly connected to source
  NodeId sink_id = graph.sink_nodes[0];
  EXPECT_EQ( graph.getNode( sink_id ).input, graph.source_node );
}

TEST_F( PipelineGraphTest, JpegInputJpegOutputWithScaleReencodes )
{
  // JPEG input to JPEG output with scaling requires decode+scale+encode
  auto config = std::make_shared<TestOutputConfiguration>();
  config->supported_input_formats = { StreamFormat::JPEG };
  config->width = 320;
  config->height = 240;
  config->codec = "jpeg";

  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( config );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::JPEG, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Decoder ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Encoder ), 1 );
}

TEST_F( PipelineGraphTest, SameResolutionDifferentFrameratesNotMultipleBranchesEarly )
{
  // 30fps and 20fps: 30 is NOT multiple of 20. Should branch early (separate scales to avoid jitter).
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 20, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 2 );
}

// -----------------------------------------------------------------------------
// HW Scaler Memory Flag Tests (No GStreamer Required)
// -----------------------------------------------------------------------------

TEST( MemoryFeatureTest, EncoderInputMemoryFlags )
{
  using namespace ros_camera_server;

  // VA and VA_LP accept VA + DMABuf memory
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::VA ),
             MemoryFeature::VA | MemoryFeature::DMA_BUF );
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::VA_LP ),
             MemoryFeature::VA | MemoryFeature::DMA_BUF );

  // VAAPI accepts VASurface + DMABuf
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::VAAPI ),
             MemoryFeature::VA_SURFACE | MemoryFeature::DMA_BUF );

  // NV accepts CUDA only
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::NV ), MemoryFeature::CUDA );

  // NVV4L2 accepts NVMM only
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::NVV4L2 ), MemoryFeature::NVMM );

  // Software backends (MPP, x264, etc.) return NONE
  EXPECT_EQ( encoderInputMemoryFlags( CodecBackendFlag::MPP ), MemoryFeature::NONE );
  EXPECT_EQ( encoderInputMemoryFlags( static_cast<CodecBackendFlag>( 0x4000 ) ), MemoryFeature::NONE );
  EXPECT_EQ( encoderInputMemoryFlags( static_cast<CodecBackendFlag>( 0x8000 ) ), MemoryFeature::NONE );
}

TEST( MemoryFeatureTest, CapsFeatureMapping )
{
  using namespace ros_camera_server;

  EXPECT_STREQ( capsFeatureForMemoryFlag( MemoryFeature::VA ), "memory:VAMemory" );
  EXPECT_STREQ( capsFeatureForMemoryFlag( MemoryFeature::VA_SURFACE ), "memory:VASurface" );
  EXPECT_STREQ( capsFeatureForMemoryFlag( MemoryFeature::DMA_BUF ), "memory:DMABuf" );
  EXPECT_STREQ( capsFeatureForMemoryFlag( MemoryFeature::CUDA ), "memory:CUDAMemory" );
  EXPECT_STREQ( capsFeatureForMemoryFlag( MemoryFeature::NVMM ), "memory:NVMM" );
  EXPECT_EQ( capsFeatureForMemoryFlag( MemoryFeature::NONE ), nullptr );
}

TEST( MemoryFeatureTest, HighestPriorityOverlap )
{
  using namespace ros_camera_server;

  // Single overlap
  EXPECT_EQ( highestPriorityOverlap( MemoryFeature::VA | MemoryFeature::DMA_BUF, MemoryFeature::VA ),
             MemoryFeature::VA );

  // Multiple overlapping bits - returns lowest bit (highest priority)
  EXPECT_EQ( highestPriorityOverlap( MemoryFeature::VA | MemoryFeature::DMA_BUF,
                                     MemoryFeature::VA | MemoryFeature::DMA_BUF ),
             MemoryFeature::VA );

  // No overlap
  EXPECT_EQ( highestPriorityOverlap( MemoryFeature::CUDA, MemoryFeature::VA ), MemoryFeature::NONE );

  // CUDA | NVMM with NVMM
  EXPECT_EQ( highestPriorityOverlap( MemoryFeature::CUDA | MemoryFeature::NVMM, MemoryFeature::NVMM ),
             MemoryFeature::NVMM );
}

TEST( MemoryFeatureTest, HwScalerCandidateTable )
{
  using namespace ros_camera_server;

  EXPECT_EQ( HW_SCALER_CANDIDATES.size(), 5u );

  const auto &candidates = HW_SCALER_CANDIDATES;

  // vapostproc is first (highest priority)
  EXPECT_STREQ( candidates[0].factory, "vapostproc" );
  EXPECT_EQ( candidates[0].upload_factory, nullptr );
  EXPECT_EQ( candidates[0].output_memory, MemoryFeature::VA );

  // cudascale requires cudaupload
  EXPECT_STREQ( candidates[4].factory, "cudascale" );
  EXPECT_STREQ( candidates[4].upload_factory, "cudaupload" );
  EXPECT_EQ( candidates[4].output_memory, MemoryFeature::CUDA );
}

TEST( MemoryFeatureTest, IntersectsOperator )
{
  using namespace ros_camera_server;

  EXPECT_TRUE( intersects( MemoryFeature::VA | MemoryFeature::DMA_BUF, MemoryFeature::VA ) );
  EXPECT_TRUE( intersects( MemoryFeature::CUDA | MemoryFeature::NVMM, MemoryFeature::NVMM ) );
  EXPECT_FALSE( intersects( MemoryFeature::CUDA, MemoryFeature::VA ) );
  EXPECT_FALSE( intersects( MemoryFeature::NONE, MemoryFeature::VA ) );
}

// -----------------------------------------------------------------------------
// collectDownstreamEncoders Tests (static method, graph-level only)
// -----------------------------------------------------------------------------

TEST_F( PipelineGraphTest, CollectDownstreamEncodersFindsH264 )
{
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 30, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  auto scale_id = findNodeByType( graph, GraphNodeType::Scale );
  ASSERT_TRUE( scale_id.has_value() );

  std::vector<NodeId> encoders;
  bool has_raw_sink = false;
  PipelineBuilder::collectDownstreamEncoders( graph, *scale_id, encoders, has_raw_sink );

  EXPECT_EQ( encoders.size(), 1u );
  EXPECT_FALSE( has_raw_sink );
}

TEST_F( PipelineGraphTest, CollectDownstreamEncodersDetectsRawSink )
{
  // Scale with a raw output should detect raw sink
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480 ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  auto scale_id = findNodeByType( graph, GraphNodeType::Scale );
  ASSERT_TRUE( scale_id.has_value() );

  std::vector<NodeId> encoders;
  bool has_raw_sink = false;
  PipelineBuilder::collectDownstreamEncoders( graph, *scale_id, encoders, has_raw_sink );

  EXPECT_TRUE( encoders.empty() );
  EXPECT_TRUE( has_raw_sink );
}

TEST_F( PipelineGraphTest, UniformEncodersAllowHwScaling )
{
  // Two H264 outputs at same resolution - shared scale with uniform encoders
  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::H264, 640, 480, Framerate( 15, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // Should share one scale node with two downstream encoder paths
  EXPECT_EQ( countNodesByType( graph, GraphNodeType::Scale ), 1 );

  auto scale_id = findNodeByType( graph, GraphNodeType::Scale );
  ASSERT_TRUE( scale_id.has_value() );

  std::vector<NodeId> encoders;
  bool has_raw_sink = false;
  PipelineBuilder::collectDownstreamEncoders( graph, *scale_id, encoders, has_raw_sink );

  // All downstream encoders are h264 - compatible for HW scaling
  EXPECT_GE( encoders.size(), 1u );
  EXPECT_FALSE( has_raw_sink );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
