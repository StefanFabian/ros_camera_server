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

#include "diagnostic_context.hpp"

#include <sstream>

namespace ros_camera_server
{
namespace
{

void destroyContext( gpointer data ) { delete static_cast<DiagnosticContextPtr *>( data ); }

DiagnosticContextPtr getDirectDiagnosticContext( GObject *object )
{
  assert( G_IS_OBJECT( object ) );
  auto *context =
      static_cast<DiagnosticContextPtr *>( g_object_get_data( object, DIAGNOSTIC_CONTEXT_KEY ) );
  if ( context == nullptr ) {
    return {};
  }
  return *context;
}

} // namespace

void attachDiagnosticContext( GObject *object, DiagnosticContextPtr context )
{
  if ( object == nullptr || context == nullptr ) {
    return;
  }
  g_object_set_data_full( object, DIAGNOSTIC_CONTEXT_KEY,
                          new DiagnosticContextPtr( std::move( context ) ), destroyContext );
}

DiagnosticContextPtr getDiagnosticContext( GObject *object )
{
  if ( object == nullptr ) {
    return {};
  }

  DiagnosticContextPtr context = getDirectDiagnosticContext( object );
  if ( context != nullptr || !GST_IS_OBJECT( object ) ) {
    return context;
  }

  GstObject *current = GST_OBJECT( gst_object_ref( GST_OBJECT( object ) ) );
  while ( current != nullptr ) {
    context = getDirectDiagnosticContext( G_OBJECT( current ) );
    if ( context != nullptr ) {
      gst_object_unref( current );
      return context;
    }

    GstObject *parent = gst_object_get_parent( current );
    gst_object_unref( current );
    current = parent;
  }

  return {};
}

std::string formatDiagnosticContext( const DiagnosticContextPtr &context )
{
  if ( context == nullptr ) {
    return {};
  }

  std::stringstream stream;
  stream << context->camera_id;
  if ( !context->affected_outputs.empty() ) {
    stream << ":";
    if ( context->affected_outputs.size() == 1 ) {
      stream << context->affected_outputs[0];
    } else {
      stream << "[";
      for ( size_t i = 0; i < context->affected_outputs.size(); ++i ) {
        if ( i != 0 ) {
          stream << ",";
        }
        stream << context->affected_outputs[i];
      }
      stream << "]";
    }
  }
  if ( context->graph_node_id.has_value() ) {
    stream << " node=" << context->graph_node_id.value();
  }
  return stream.str();
}

} // namespace ros_camera_server
