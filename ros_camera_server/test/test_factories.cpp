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

#include <gtest/gtest.h>
#include <ros_camera_server/configuration.hpp>
#include <ros_camera_server/factories/pipeline_input_factory.hpp>
#include <ros_camera_server/factories/pipeline_output_factory.hpp>

class MockInputConfiguration : public ros_camera_server::InputConfiguration
{
public:
  ros_camera_server::StreamInput createInput( const rclcpp::Node::SharedPtr &,
                                              const std::string & ) const override
  { return {}; }
};

class MockOutputConfiguration : public ros_camera_server::OutputConfiguration
{
public:
  MockOutputConfiguration() : OutputConfiguration( "mock" ) { }

  YAML::Node toYaml() const override { return YAML::Node(); }

  ros_camera_server::PipelineOutput::Ptr createOutput( const rclcpp::Node::SharedPtr &,
                                                       const std::string &, int index ) const override
  { return nullptr; }
};

TEST( FactoryTest, TestInputFactoryValues )
{
  // Clear registry if possible? No method to clear.
  // We have to use unique names for tests to avoid conflicts if tests run in same process.
  std::string input_type = "test_input_1";

  auto creator = []( const YAML::Node & ) -> std::shared_ptr<ros_camera_server::InputConfiguration> {
    return std::make_shared<MockInputConfiguration>();
  };

  ros_camera_server::PipelineInputFactory::registerInput( input_type, creator );

  YAML::Node config;
  auto input = ros_camera_server::PipelineInputFactory::createInput( input_type, config );
  EXPECT_NE( input, nullptr );

  auto missing = ros_camera_server::PipelineInputFactory::createInput( "non_existent", config );
  EXPECT_EQ( missing, nullptr );
}

TEST( FactoryTest, TestInputFactoryDuplicate )
{
  std::string input_type = "test_input_2";

  auto creator = []( const YAML::Node & ) -> std::shared_ptr<ros_camera_server::InputConfiguration> {
    return std::make_shared<MockInputConfiguration>();
  };

  ros_camera_server::PipelineInputFactory::registerInput( input_type, creator );

  EXPECT_THROW(
      { ros_camera_server::PipelineInputFactory::registerInput( input_type, creator ); },
      ros_camera_server::FactoryException );
}

TEST( FactoryTest, TestOutputFactory )
{
  std::string output_type = "test_output_1";

  auto creator = []( const YAML::Node & ) -> std::shared_ptr<ros_camera_server::OutputConfiguration> {
    return std::make_shared<MockOutputConfiguration>();
  };

  ros_camera_server::PipelineOutputFactory::registerOutput( output_type, creator );

  YAML::Node config;
  auto output = ros_camera_server::PipelineOutputFactory::createOutput( output_type, config );
  EXPECT_NE( output, nullptr );

  auto missing = ros_camera_server::PipelineOutputFactory::createOutput( "non_existent", config );
  EXPECT_EQ( missing, nullptr );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
