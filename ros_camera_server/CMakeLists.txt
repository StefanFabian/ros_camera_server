cmake_minimum_required(VERSION 3.5.1)
project(ros_camera_server)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Wpedantic -Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros_camera_server_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FindPkgConfig)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0 gstreamer-app-1.0 gstreamer-net-1.0
  gstreamer-plugins-base-1.0 gstreamer-pbutils-1.0 gstreamer-rtp-1.0
  gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
pkg_check_modules(SOUP REQUIRED libsoup-3.0)


###########
## Build ##
###########

set(SOURCES
  src/factories/pipeline_input_factory.cpp
  src/factories/pipeline_output_factory.cpp
  src/diagnostic_context.cpp
  src/helpers/reference_timestamp_helpers.cpp
  src/helpers/rtp_timestamp_extension.cpp
  src/helpers/video_codec_info.cpp
  src/inputs/openseekthermal_input.cpp
  src/inputs/ros2_input.cpp
  src/inputs/rtp_input.cpp
  src/inputs/v4l2_input.cpp
  src/outputs/pipeline_output.cpp
  src/outputs/ros2_output.cpp
  src/outputs/rtp_output.cpp
  src/outputs/srt_output.cpp
  src/outputs/webrtc_output.cpp
  src/webrtc/signaling_server.cpp
  src/webrtc/webrtc_peer.cpp
  src/pipeline/pipeline_graph.cpp
  src/pipeline/pipeline_builder.cpp

  src/camera_pipeline.cpp
  src/pipeline_monitor.cpp
  src/camera_server.cpp
  src/camera_server_node.cpp
  src/camera_server_standalone.cpp
  src/configuration.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  ${GST_LIBRARIES}
  ${SOUP_LIBRARIES}
  rclcpp::rclcpp
)
target_link_libraries(
  ${PROJECT_NAME} PRIVATE
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp ros_camera_server_msgs)
target_include_directories(${PROJECT_NAME}
  PUBLIC ${GST_INCLUDE_DIRS} ${SOUP_INCLUDE_DIRS})

add_executable(camera_server_standalone src/camera_server_standalone.cpp)
target_link_libraries(camera_server_standalone ${PROJECT_NAME})

add_executable(enumerate_encoders src/tools/enumerate_encoders.cpp)
target_link_libraries(enumerate_encoders
  ${PROJECT_NAME}
  ${GST_LIBRARIES}
  rclcpp::rclcpp
)
target_include_directories(enumerate_encoders PUBLIC ${GST_INCLUDE_DIRS})

add_executable(v4l2_caps_investigator src/tools/v4l2_caps_investigator.cpp)
target_link_libraries(v4l2_caps_investigator ${GST_LIBRARIES})
target_include_directories(v4l2_caps_investigator PUBLIC ${GST_INCLUDE_DIRS})

add_executable(v4l2_controls_inspector src/tools/v4l2_controls_inspector.cpp)
target_link_libraries(v4l2_controls_inspector ${GST_LIBRARIES})
target_include_directories(v4l2_controls_inspector PUBLIC ${GST_INCLUDE_DIRS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(TARGETS camera_server_standalone enumerate_encoders
                v4l2_caps_investigator v4l2_controls_inspector
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp rclcpp ros_camera_server_msgs yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_configuration test/test_configuration.cpp)
  target_link_libraries(test_configuration ${PROJECT_NAME} yaml-cpp::yaml-cpp)

  ament_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer ${PROJECT_NAME})

  ament_add_gtest(test_factories test/test_factories.cpp)
  target_link_libraries(test_factories ${PROJECT_NAME} yaml-cpp::yaml-cpp)

  ament_add_gtest(test_streaming test/test_streaming.cpp)
  target_link_libraries(test_streaming ${PROJECT_NAME} yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_streaming sensor_msgs)
  target_include_directories(test_streaming PRIVATE src)

  ament_add_gtest(test_pipeline_builder test/test_pipeline_builder.cpp)
  target_link_libraries(test_pipeline_builder ${PROJECT_NAME} yaml-cpp::yaml-cpp)
  target_include_directories(test_pipeline_builder PRIVATE src)

  ament_add_gtest(test_smart_decoder test/test_smart_decoder.cpp)
  target_link_libraries(test_smart_decoder ${PROJECT_NAME})
  target_include_directories(test_smart_decoder PRIVATE src)

  ament_add_gtest(test_pipeline_deduplication test/test_pipeline_deduplication.cpp)
  target_link_libraries(test_pipeline_deduplication ${PROJECT_NAME} yaml-cpp::yaml-cpp)
  target_include_directories(test_pipeline_deduplication PRIVATE src)

  ament_add_gtest(test_signaling_server test/test_signaling_server.cpp)
  target_link_libraries(test_signaling_server ${PROJECT_NAME} ${SOUP_LIBRARIES})
  target_include_directories(test_signaling_server PRIVATE src)

  ament_add_gtest(test_pipeline_monitor test/test_pipeline_monitor.cpp)
  target_link_libraries(test_pipeline_monitor ${PROJECT_NAME})
  target_include_directories(test_pipeline_monitor PRIVATE src)

  ament_add_gtest(test_gstreamer_log_filter test/test_gstreamer_log_filter.cpp)
  target_include_directories(test_gstreamer_log_filter PRIVATE src)

  # Prevent gstreamer from forking gst-plugin-scanner. Under ASAN, the scanner
  # subprocess hits a cyclonedds exit-handler bad-free that aborts the
  # subprocess and floods test logs without affecting the actual test.
  # Also point LeakSanitizer at our suppression file so leaks in
  # gstreamer/libav internals (which we cannot fix here) don't fail the run.
  set(_ros_camera_server_test_targets
      test_configuration
      test_ring_buffer
      test_factories
      test_streaming
      test_pipeline_builder
      test_smart_decoder
      test_pipeline_deduplication
      test_signaling_server
      test_pipeline_monitor
      test_gstreamer_log_filter)
  set(_ros_camera_server_lsan_supp "${CMAKE_CURRENT_SOURCE_DIR}/test/lsan.supp")
  foreach(_test_name IN LISTS _ros_camera_server_test_targets)
    if(TEST ${_test_name})
      set_property(TEST ${_test_name} APPEND PROPERTY ENVIRONMENT
          "GST_REGISTRY_FORK=no"
          "LSAN_OPTIONS=suppressions=${_ros_camera_server_lsan_supp}")
    endif()
  endforeach()
endif()

ament_package()
