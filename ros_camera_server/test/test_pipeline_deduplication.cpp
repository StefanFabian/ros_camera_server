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
#include "ros_camera_server/configuration.hpp"

#include "pipeline_test_helpers.hpp"

using namespace ros_camera_server;

class PipelineDeduplicationTest : public ::testing::Test
{
protected:
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
};

TEST_F( PipelineDeduplicationTest, Case4DeduplicationWithoutFramerateLimits )
{
  // This test mirrors the user's situation:
  // Multiple outputs with same scale but different overall resolutions.
  // No framerate limits specified (all use input framerate).

  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  // Output 1: 600x480, no framerate limit
  outputs.push_back( createOutputConfig( StreamFormat::H264, 600, 480 ) );
  // Output 2: 600x480, no framerate limit
  outputs.push_back( createOutputConfig( StreamFormat::H265, 600, 480 ) );
  // Output 3: 1920x1080, no framerate limit
  outputs.push_back( createOutputConfig( StreamFormat::H265, 1920, 1080 ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );

  // This hits Case 4 because any_needs_framerate is false.
  std::cout << graph.toString() << std::endl;

  int scale_600_480_count = 0;
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type == GraphNodeType::Scale ) {
      const ScaleKey &key = std::get<ScaleKey>( node.config );
      if ( key.max_width == 600 && key.max_height == 480 )
        scale_600_480_count++;
    }
  }

  EXPECT_EQ( scale_600_480_count, 1 )
      << "Should have only one Scale(600x480) node even without framerate limits";
}

TEST_F( PipelineDeduplicationTest, Case4MixedFramerateLimits )
{
  // Output 1: 600x480, 30fps
  // Output 2: 600x480, no limit (30fps)
  // Output 3: 1920x1080, no limit

  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::H264, 600, 480, Framerate( 30, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::H265, 600, 480 ) ); // no limit
  outputs.push_back( createOutputConfig( StreamFormat::H265, 1920, 1080 ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );
  std::cout << graph.toString() << std::endl;

  int scale_600_480_count = 0;
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type == GraphNodeType::Scale ) {
      const ScaleKey &key = std::get<ScaleKey>( node.config );
      if ( key.max_width == 600 && key.max_height == 480 )
        scale_600_480_count++;
    }
  }
  EXPECT_EQ( scale_600_480_count, 1 )
      << "Should share scale even if one output has framerate limit and another not";
}

TEST_F( PipelineDeduplicationTest, ChainedFramerateLimiters )
{
  // 30fps input -> 15fps output -> 5fps output. 15 is multiple of 5.
  // Should (ideally) chain: Source -> 15fps -> [Scale, 5fps -> Scale]
  // Or: Source -> Scale -> [15fps, 5fps]

  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 15, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 5, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );
  std::cout << graph.toString() << std::endl;

  // We should have at most one Scale node for 640x480.
  int scale_count = 0;
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type == GraphNodeType::Scale )
      scale_count++;
  }
  EXPECT_EQ( scale_count, 1 );
}

TEST_F( PipelineDeduplicationTest, BranchedFramerateLimiters )
{
  // 15fps and 10fps. 15 is NOT a multiple of 10.
  // Should NOT chain. Should branch at source (30fps).

  std::vector<std::shared_ptr<OutputConfiguration>> outputs;
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 15, 1 ) ) );
  outputs.push_back( createOutputConfig( StreamFormat::RAW, 640, 480, Framerate( 10, 1 ) ) );

  PipelineGraph graph = PipelineGraph::build( StreamFormat::RAW, "v4l2", outputs );
  std::cout << graph.toString() << std::endl;

  // Non-multiple framerates should NOT share scalers (to avoid jitter)
  int scale_count = 0;
  for ( const auto &[id, node] : graph.nodes ) {
    if ( node.type == GraphNodeType::Scale )
      scale_count++;
  }
  EXPECT_EQ( scale_count, 2 );
}
