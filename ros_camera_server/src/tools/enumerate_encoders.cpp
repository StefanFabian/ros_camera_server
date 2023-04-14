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

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include "../codecs/h264.hpp"
#include "../codecs/h265.hpp"
#include "ros_camera_server/helpers/smart_gst_pointer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

using namespace ros_camera_server;

namespace
{
struct Resolution {
  int width;
  int height;
  std::string toString() const { return std::to_string( width ) + "x" + std::to_string( height ); }
};

struct PipelineData {
  SmartGstPointer<GstElement> pipeline;
  std::atomic<uint64_t> frame_count{ 0 };
  std::deque<std::chrono::steady_clock::time_point> frame_timestamps;
  std::vector<double> latencies_ms;
  std::mutex mtx;
};
} // namespace

// Called after encoding, on sink pad
static GstPadProbeReturn frame_count_probe( GstPad * /*pad*/, GstPadProbeInfo *info,
                                            gpointer user_data )
{
  if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
    auto *data = static_cast<PipelineData *>( user_data );
    data->frame_count.fetch_add( 1, std::memory_order_relaxed );
    // Pop timestamp and record latency
    std::lock_guard<std::mutex> lock( data->mtx );
    if ( !data->frame_timestamps.empty() ) {
      auto t0 = data->frame_timestamps.front();
      data->frame_timestamps.pop_front();
      auto t1 = std::chrono::steady_clock::now();
      double latency_ms =
          std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000.0;
      data->latencies_ms.push_back( latency_ms );
    }
  }
  return GST_PAD_PROBE_OK;
}

// Called on src pad, before encoding
static GstPadProbeReturn timestamp_probe( GstPad * /*pad*/, GstPadProbeInfo *info, gpointer user_data )
{
  if ( GST_PAD_PROBE_INFO_TYPE( info ) & GST_PAD_PROBE_TYPE_BUFFER ) {
    auto *data = static_cast<PipelineData *>( user_data );
    std::lock_guard<std::mutex> lock( data->mtx );
    data->frame_timestamps.push_back( std::chrono::steady_clock::now() );
  }
  return GST_PAD_PROBE_OK;
}

struct RunStreamsResult {
  bool success;
  std::vector<double> latencies_ms;
};

RunStreamsResult
run_streams( const std::string &encoder_name, const Resolution &res, int count,
             std::function<GstElement *( const std::string &, int )> create_encoder_func )
{
  constexpr double TARGET_FPS = 30.0;
  constexpr double MIN_FPS_RATIO = 0.9; // Must achieve at least 90% of target fps
  constexpr int TEST_DURATION_SECS = 3;

  std::vector<std::unique_ptr<PipelineData>> pipeline_data;
  bool success = true;

  for ( int i = 0; i < count; ++i ) {
    auto data = std::make_unique<PipelineData>();
    data->pipeline = gst_pipeline_new( nullptr );
    GstElement *src = gst_element_factory_make( "videotestsrc", nullptr );
    GstElement *capsfilter = gst_element_factory_make( "capsfilter", nullptr );
    GstElement *enc_bin = create_encoder_func( encoder_name, 1000 );
    GstElement *sink = gst_element_factory_make( "fakesink", nullptr );

    if ( !data->pipeline || !src || !capsfilter || !enc_bin || !sink ) {
      if ( src )
        gst_object_unref( src );
      if ( capsfilter )
        gst_object_unref( capsfilter );
      if ( enc_bin )
        gst_object_unref( enc_bin );
      if ( sink )
        gst_object_unref( sink );
      success = false;
      break;
    }

    g_object_set( src, "is-live", TRUE, nullptr );
    g_object_set( sink, "sync", FALSE, nullptr ); // Don't sync to clock, encode as fast as possible

    GstCaps *caps =
        gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT,
                             res.width, "height", G_TYPE_INT, res.height, "framerate",
                             GST_TYPE_FRACTION, static_cast<int>( TARGET_FPS ), 1, nullptr );
    g_object_set( capsfilter, "caps", caps, nullptr );
    gst_caps_unref( caps );

    gst_bin_add_many( GST_BIN( data->pipeline.get() ), src, capsfilter, enc_bin, sink, nullptr );
    if ( !gst_element_link_many( src, capsfilter, enc_bin, sink, nullptr ) ) {
      success = false;
      break;
    }

    // Add probe to count frames after encoder (latency end)
    GstPad *sink_pad = gst_element_get_static_pad( sink, "sink" );
    if ( sink_pad ) {
      gst_pad_add_probe( sink_pad, GST_PAD_PROBE_TYPE_BUFFER, frame_count_probe, data.get(), nullptr );
      gst_object_unref( sink_pad );
    }
    // Add probe to src pad (latency start)
    GstPad *src_pad = gst_element_get_static_pad( src, "src" );
    if ( src_pad ) {
      gst_pad_add_probe( src_pad, GST_PAD_PROBE_TYPE_BUFFER, timestamp_probe, data.get(), nullptr );
      gst_object_unref( src_pad );
    }

    pipeline_data.push_back( std::move( data ) );
  }

  if ( success ) {
    for ( const auto &data : pipeline_data ) {
      if ( gst_element_set_state( data->pipeline, GST_STATE_PLAYING ) == GST_STATE_CHANGE_FAILURE ) {
        success = false;
        break;
      }
    }
  }

  if ( success ) {
    // Run for test duration and check for errors
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds( TEST_DURATION_SECS );

    while ( rclcpp::ok() && std::chrono::steady_clock::now() < end_time ) {
      bool error_found = false;
      for ( const auto &data : pipeline_data ) {
        GstBus *bus = gst_element_get_bus( data->pipeline );
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, 10 * GST_MSECOND,
            static_cast<GstMessageType>( GST_MESSAGE_ERROR | GST_MESSAGE_EOS ) );

        if ( msg ) {
          if ( GST_MESSAGE_TYPE( msg ) == GST_MESSAGE_ERROR ) {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error( msg, &err, &debug_info );
            // std::cerr << "Error detected: " << err->message << std::endl;
            g_clear_error( &err );
            g_free( debug_info );
            error_found = true;
          }
          gst_message_unref( msg );
        }
        gst_object_unref( bus );
        if ( error_found )
          break;
      }
      if ( error_found ) {
        success = false;
        break;
      }
    }

    // Check if all pipelines achieved the target framerate
    if ( success ) {
      auto actual_duration = std::chrono::steady_clock::now() - start_time;
      double duration_secs =
          std::chrono::duration_cast<std::chrono::milliseconds>( actual_duration ).count() / 1000.0;
      double min_required_fps = TARGET_FPS * MIN_FPS_RATIO;

      for ( const auto &data : pipeline_data ) {
        uint64_t frames = data->frame_count.load( std::memory_order_relaxed );
        double actual_fps = static_cast<double>( frames ) / duration_secs;
        if ( actual_fps < min_required_fps ) {
          success = false;
          break;
        }
      }
    }
  }

  // Collect all latencies BEFORE deleting PipelineData
  std::vector<double> all_latencies;
  for ( const auto &data : pipeline_data ) {
    all_latencies.insert( all_latencies.end(), data->latencies_ms.begin(), data->latencies_ms.end() );
  }
  for ( const auto &data : pipeline_data ) {
    gst_element_set_state( data->pipeline, GST_STATE_NULL );
  }
  return { success, std::move( all_latencies ) };
}

void test_encoders( const std::vector<std::string> &encoders,
                    const std::vector<Resolution> &resolutions,
                    std::function<bool( const std::string & )> check_availability,
                    std::function<GstElement *( const std::string &, int )> create_encoder )
{
  for ( const auto &enc : encoders ) {
    if ( !rclcpp::ok() )
      break;

    if ( !check_availability( enc ) ) {
      std::cout << "Encoder " << enc << " not available." << std::endl;
      continue;
    }

    std::cout << "Testing encoder: " << enc << std::endl;
    for ( const auto &res : resolutions ) {
      if ( !rclcpp::ok() )
        break;
      int max_supported = 0;
      int low = 1;
      int high = 32; // Upper bound for search

      std::vector<double> best_latencies;
      while ( low <= high && rclcpp::ok() ) {
        int mid = low + ( high - low ) / 2;
        RunStreamsResult result = run_streams( enc, res, mid, create_encoder );
        if ( result.success ) {
          max_supported = mid;
          best_latencies = std::move( result.latencies_ms );
          low = mid + 1;
        } else {
          high = mid - 1;
        }
      }
      std::cout << "  Max streams for " << enc << " @ " << res.toString() << ": " << max_supported;
      if ( !best_latencies.empty() ) {
        double sum = 0.0;
        for ( double l : best_latencies ) sum += l;
        double avg_latency = sum / best_latencies.size();
        std::sort( best_latencies.begin(), best_latencies.end() );
        size_t idx95 = static_cast<size_t>( 0.95 * best_latencies.size() );
        if ( idx95 >= best_latencies.size() )
          idx95 = best_latencies.size() - 1;
        double p95_latency = best_latencies[idx95];
        std::cout << " (avg latency = " << avg_latency << " ms, p95 latency = " << p95_latency
                  << " ms)";
      }
      std::cout << std::endl;
    }
  }
}

static std::vector<std::string> split( const std::string &s, char delimiter )
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream( s );
  while ( std::getline( tokenStream, token, delimiter ) ) { tokens.push_back( token ); }
  return tokens;
}

Resolution parse_resolution( const std::string &s )
{
  auto parts = split( s, 'x' );
  if ( parts.size() != 2 )
    return { 0, 0 };
  try {
    return { std::stoi( parts[0] ), std::stoi( parts[1] ) };
  } catch ( ... ) {
    return { 0, 0 };
  }
}

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  gst_init( &argc, &argv );

  std::string codec_filter = "all";
  std::vector<std::string> encoder_filter;
  std::vector<Resolution> resolutions = {
      { 640, 480 }, { 1280, 720 }, { 1920, 1080 }, { 3840, 2160 } };

  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( arg == "-h" || arg == "--help" ) {
      std::cout
          << "Usage: enumerate_encoders [OPTIONS]\n\n"
          << "Test available encoders for maximum concurrent streams.\n\n"
          << "Options:\n"
          << "  -c, --codec CODEC       Filter by codec (h264, h265, all). Default: all\n"
          << "  -e, --encoder ENCODERS  Filter by encoder names (comma-separated, e.g. MPP,NV)\n"
          << "  -r, --resolution RES    Filter by resolutions (comma-separated, e.g. "
             "640x480,1280x720)\n"
          << "  -h, --help              Show help message\n";
      return 0;
    }
    if ( ( arg == "-c" || arg == "--codec" ) && i + 1 < argc ) {
      codec_filter = argv[++i];
      std::transform( codec_filter.begin(), codec_filter.end(), codec_filter.begin(),
                      []( unsigned char c ) { return std::tolower( c ); } );
    } else if ( ( arg == "-e" || arg == "--encoder" ) && i + 1 < argc ) {
      encoder_filter = split( argv[++i], ',' );
    } else if ( ( arg == "-r" || arg == "--resolution" ) && i + 1 < argc ) {
      auto res_strings = split( argv[++i], ',' );
      std::vector<Resolution> custom_resolutions;
      for ( const auto &rs : res_strings ) {
        Resolution res = parse_resolution( rs );
        if ( res.width > 0 && res.height > 0 ) {
          custom_resolutions.push_back( res );
        }
      }
      if ( !custom_resolutions.empty() ) {
        resolutions = custom_resolutions;
      }
    }
  }

  auto filter_encoders = [&]( std::vector<std::string> &encoders ) {
    if ( encoder_filter.empty() )
      return;
    std::vector<std::string> filtered;
    for ( const auto &enc : encoders ) {
      if ( std::find( encoder_filter.begin(), encoder_filter.end(), enc ) != encoder_filter.end() ) {
        filtered.push_back( enc );
      }
    }
    encoders = std::move( filtered );
  };

  std::cout << "ROS Camera Server Encoder test" << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "Testing available encoders for maximum concurrent streams at 30 fps." << std::endl;
  std::cout << "Each encoder must sustain at least 90% of target framerate (27 fps)." << std::endl;
  std::cout << "Press Ctrl+C to abort." << std::endl;
  std::cout << "Maximum test is 32 streams, if your system can handle more, it will show as 32."
            << std::endl;

  // H264
  if ( codec_filter == "all" || codec_filter == "h264" ) {
    std::vector<std::string> encoders_h264 = { "MPP", "NV", "NVV4L2", "VA", "VA_LP", "VAAPI" };
    filter_encoders( encoders_h264 );
    if ( !encoders_h264.empty() ) {
      std::cout << "\n=== H264 Encoders ===" << std::endl;
      test_encoders(
          encoders_h264, resolutions,
          []( const std::string &enc ) {
            const auto &desc = h264EncoderDescriptor();
            uint32_t desired = codecFromString( desc, enc );
            uint32_t selected =
                codecSelect( desc, desired, uint32_t( H264Encoder::AUTO ) & ~desired );
            return selected != 0;
          },
          []( const std::string &enc, int bitrate ) {
            return createH264Encoder( enc, "encoder_bin", CodecOptions{ bitrate } );
          } );
    }
  }

  // H265
  if ( codec_filter == "all" || codec_filter == "h265" ) {
    std::vector<std::string> encoders_h265 = { "MPP", "NV", "NVV4L2", "VA", "VA_LP", "VAAPI" };
    filter_encoders( encoders_h265 );
    if ( !encoders_h265.empty() ) {
      std::cout << "\n=== H265 Encoders ===" << std::endl;
      test_encoders(
          encoders_h265, resolutions,
          []( const std::string &enc ) {
            uint32_t desired = codecFromString( h265_encoder_desc, enc );
            uint32_t selected =
                codecSelect( h265_encoder_desc, desired, uint32_t( H265Encoder::AUTO ) & ~desired );
            return selected != 0;
          },
          []( const std::string &enc, int bitrate ) {
            return createH265Encoder( enc, "encoder_bin", CodecOptions{ bitrate } );
          } );
    }
  }

  rclcpp::shutdown();
  return 0;
}
