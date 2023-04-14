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

#include "pipeline_monitor.hpp"
#include "gstreamer_log_filter.hpp"
#include "logging.hpp"

#include <sstream>

namespace ros_camera_server
{

void PipelineMonitor::observeGStreamerLog( GstDebugCategory *category, GstDebugLevel level,
                                           GObject *object, GstDebugMessage *message )
{
  if ( level != GST_LEVEL_WARNING && level != GST_LEVEL_ERROR ) {
    return;
  }

  DiagnosticRecord record;
  record.source = Source::DebugLog;
  record.level = level;
  const gchar *category_name =
      category == nullptr ? nullptr : gst_debug_category_get_name( category );
  const gchar *message_text = message == nullptr ? nullptr : gst_debug_message_get( message );
  const gchar *object_id = message == nullptr ? nullptr : gst_debug_message_get_id( message );
  record.category = category_name == nullptr ? "" : category_name;
  record.message = message_text == nullptr ? "" : message_text;
  record.object_id = object_id == nullptr ? "" : object_id;
  if ( object != nullptr && G_IS_OBJECT( object ) ) {
    record.weak_object.set( object );
  }
  enqueue( std::move( record ) );
}

void PipelineMonitor::observeBusWarning( const std::string &camera_id, GstObject *source,
                                         const GError *error, const char *debug_info )
{ observeBusMessage( GST_LEVEL_WARNING, camera_id, source, error, debug_info ); }

void PipelineMonitor::observeBusError( const std::string &camera_id, GstObject *source,
                                       const GError *error, const char *debug_info )
{ observeBusMessage( GST_LEVEL_ERROR, camera_id, source, error, debug_info ); }

void PipelineMonitor::observeBusMessage( GstDebugLevel level, const std::string &camera_id,
                                         GstObject *source, const GError *error,
                                         const char *debug_info )
{
  DiagnosticRecord record;
  record.source = Source::Bus;
  record.level = level;
  record.camera_id = camera_id;
  record.message = error == nullptr || error->message == nullptr ? "" : error->message;
  record.debug_info = debug_info == nullptr ? "" : debug_info;
  if ( source != nullptr && G_IS_OBJECT( source ) ) {
    record.weak_object.set( G_OBJECT( source ) );
    const gchar *name = GST_OBJECT_NAME( source );
    record.object_id = name == nullptr ? "" : name;
  }
  enqueue( std::move( record ) );
}

void PipelineMonitor::processLogMessages()
{
  std::vector<DiagnosticRecord> drained;
  size_t dropped_records;
  {
    std::lock_guard lock( mutex_ );
    drained.swap( pending_ );
    dropped_records = dropped_;
    dropped_ = 0;
  }

  if ( dropped_records > 0 ) {
    SERVER_LOG_WARN( "Dropped %zu GStreamer diagnostic records because monitor queue was full.",
                     dropped_records );
  }

  for ( DiagnosticRecord &record : drained ) { process( record ); }
}

void PipelineMonitor::enqueue( DiagnosticRecord record )
{
  std::lock_guard lock( mutex_ );
  if ( pending_.size() >= MAX_PENDING_RECORDS ) {
    pending_.erase( pending_.begin() );
    ++dropped_;
  }
  pending_.push_back( std::move( record ) );
}

void PipelineMonitor::process( DiagnosticRecord &record )
{
  // Resolve diagnostic context now (timer thread, safe to walk parent chain). If the underlying
  // GObject was already destroyed, weak_object.acquire() returns nullptr and we fall back to
  // logging without structured context.
  if ( GObject *object = record.weak_object.acquire(); object != nullptr ) {
    record.context = getDiagnosticContext( object );
    g_object_unref( object );
  }

  if ( shouldDropDebugRecord( record ) ) {
    return;
  }

  std::string suffix;
  if ( !shouldLogRecord( record, suffix ) ) {
    return;
  }

  emit( record.level, formatRecord( record, suffix ) );
}

void PipelineMonitor::emit( GstDebugLevel level, const std::string &message )
{
  if ( level == GST_LEVEL_ERROR ) {
    SERVER_LOG_ERROR_STREAM( message );
  } else {
    SERVER_LOG_WARN_STREAM( message );
  }
}

bool PipelineMonitor::shouldDropDebugRecord( const DiagnosticRecord &record ) const
{
  if ( record.source != Source::DebugLog ) {
    return false;
  }
  if ( detail::shouldIgnoreGStreamerLog( record.category, record.message ) ) {
    return true;
  }
  // gst_element_message_full() formats posted bus warnings/errors as "warning: <text>" /
  // "error: <text>" in the debug log (gstelement.c). We see the bus path version too via
  // observeBusWarning/Error - drop the debug-log replay when we resolved a context (i.e. the
  // originating element belongs to one of our pipelines) to avoid duplicate logs.
  return record.context != nullptr && isStructuredPipelineMessage( record.message );
}

bool PipelineMonitor::shouldLogRecord( const DiagnosticRecord &record, std::string &suffix )
{
  const std::string key = recordKey( record );
  const auto now = std::chrono::steady_clock::now();
  auto it = coalesced_logs_.find( key );
  if ( it == coalesced_logs_.end() ) {
    if ( coalesced_logs_.size() >= MAX_COALESCED_ENTRIES ) {
      // Evict idle entries (nothing pending). Active coalescers stay so their suppressed counts
      // aren't lost. If everything is active (degenerate flood), the map keeps growing slightly
      // past the cap until something settles - acceptable.
      for ( auto cand = coalesced_logs_.begin(); cand != coalesced_logs_.end(); ) {
        if ( cand->second.suppressed == 0 ) {
          cand = coalesced_logs_.erase( cand );
        } else {
          ++cand;
        }
      }
    }
    coalesced_logs_.emplace( key, CoalescedLogState{ 0, now } );
    return true;
  }

  CoalescedLogState &state = it->second;
  if ( now - state.last_logged >= REPEAT_LOG_INTERVAL ) {
    if ( state.suppressed > 0 ) {
      suffix = " (suppressed " + std::to_string( state.suppressed ) + " repeated messages)";
      state.suppressed = 0;
    }
    state.last_logged = now;
    return true;
  }
  ++state.suppressed;
  return false;
}

bool PipelineMonitor::isStructuredPipelineMessage( const std::string &message )
{ return detail::hasPrefix( message, "warning: " ) || detail::hasPrefix( message, "error: " ); }

std::string PipelineMonitor::recordKey( const DiagnosticRecord &record )
{
  std::stringstream stream;
  stream << static_cast<int>( record.source ) << '|' << static_cast<int>( record.level ) << '|'
         << record.camera_id << '|' << record.category << '|' << record.message << '|'
         << formatDiagnosticContext( record.context );
  return stream.str();
}

std::string PipelineMonitor::formatRecord( const DiagnosticRecord &record, const std::string &suffix )
{
  std::stringstream stream;
  stream << "GStreamer ";
  stream << ( record.source == Source::Bus ? "bus" : "log" );
  if ( record.level == GST_LEVEL_ERROR ) {
    stream << " error";
  } else {
    stream << " warning";
  }
  if ( !record.category.empty() ) {
    stream << " [" << record.category << "]";
  }

  const std::string context = formatDiagnosticContext( record.context );
  if ( !context.empty() ) {
    stream << " [" << context << "]";
  } else if ( !record.camera_id.empty() ) {
    stream << " [" << record.camera_id << "]";
  } else if ( !record.object_id.empty() ) {
    stream << " [" << record.object_id << "]";
  }

  if ( !record.message.empty() ) {
    stream << ": " << record.message;
  }
  if ( !record.debug_info.empty() ) {
    stream << " (" << record.debug_info << ")";
  }
  stream << suffix;
  return stream.str();
}

} // namespace ros_camera_server
