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

#ifndef ROS_CAMERA_SERVER_PIPELINE_MONITOR_HPP
#define ROS_CAMERA_SERVER_PIPELINE_MONITOR_HPP

#include "diagnostic_context.hpp"
#include "gst_weak_object_ref.hpp"
#include "pipeline_monitor_interface.hpp"

#include <chrono>
#include <gst/gstinfo.h>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros_camera_server
{

class PipelineMonitor : public PipelineMonitorInterface
{
public:
  PipelineMonitor() = default;

  void observeGStreamerLog( GstDebugCategory *category, GstDebugLevel level, GObject *object,
                            GstDebugMessage *message ) override;
  void observeBusWarning( const std::string &camera_id, GstObject *source, const GError *error,
                          const char *debug_info ) override;
  void observeBusError( const std::string &camera_id, GstObject *source, const GError *error,
                        const char *debug_info ) override;

  // Drain pending records and emit them. Single-threaded; call from one timer/thread only.
  void processLogMessages();

protected:
  // Emit a formatted, deduplicated diagnostic message. Override in tests or to redirect logs.
  virtual void emit( GstDebugLevel level, const std::string &message );

private:
  enum class Source { DebugLog, Bus };

  struct DiagnosticRecord {
    Source source = Source::DebugLog;
    GstDebugLevel level = GST_LEVEL_NONE;
    std::string camera_id;
    std::string category;
    std::string message;
    std::string debug_info;
    std::string object_id;
    WeakObjectRef weak_object;
    DiagnosticContextPtr context; // resolved on processing thread
  };

  struct CoalescedLogState {
    size_t suppressed = 0;
    std::chrono::steady_clock::time_point last_logged;
  };

  static constexpr size_t MAX_PENDING_RECORDS = 256;
  static constexpr size_t MAX_COALESCED_ENTRIES = 128;
  static constexpr auto REPEAT_LOG_INTERVAL = std::chrono::seconds( 10 );

  void observeBusMessage( GstDebugLevel level, const std::string &camera_id, GstObject *source,
                          const GError *error, const char *debug_info );
  void enqueue( DiagnosticRecord record );
  void process( DiagnosticRecord &record );
  bool shouldDropDebugRecord( const DiagnosticRecord &record ) const;
  bool shouldLogRecord( const DiagnosticRecord &record, std::string &suffix );
  static bool isStructuredPipelineMessage( const std::string &message );
  static std::string recordKey( const DiagnosticRecord &record );
  static std::string formatRecord( const DiagnosticRecord &record, const std::string &suffix );

  // mutex_ protects pending_ and dropped_ only. coalesced_logs_ is touched exclusively from
  // processLogMessages (timer thread) and therefore needs no lock.
  std::mutex mutex_;
  std::vector<DiagnosticRecord> pending_;
  size_t dropped_ = 0;

  std::unordered_map<std::string, CoalescedLogState> coalesced_logs_;
};

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_PIPELINE_MONITOR_HPP
