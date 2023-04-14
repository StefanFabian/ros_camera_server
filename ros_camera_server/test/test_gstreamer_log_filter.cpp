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

#include "gstreamer_log_filter.hpp"

#include <gtest/gtest.h>

namespace ros_camera_server::detail
{

// Tests cover the match modes themselves, not every ignore-list row. Add a new test only when
// introducing a new LogMatchMode or a tricky pattern (e.g. encoding-dependent regex).

TEST( GStreamerLogFilterTest, ExactMatch )
{
  EXPECT_TRUE( shouldIgnoreGStreamerLog( "vadisplay", "vaInitialize: unknown libva error" ) );
  EXPECT_FALSE( shouldIgnoreGStreamerLog( "vadisplay", "different message" ) );
  EXPECT_FALSE( shouldIgnoreGStreamerLog( "wrong_category", "vaInitialize: unknown libva error" ) );
}

TEST( GStreamerLogFilterTest, WebRtcResolveRegexMatchesBothQuoteEncodings )
{
  // Regex must match both ASCII " and UTF-8 smart quotes (U+201C/U+201D) since libnice's quoting
  // depends on glib version. This is the non-trivial property worth pinning.
  EXPECT_TRUE( shouldIgnoreGStreamerLog(
      "webrtcnice",
      "failed to resolve: Error resolving \"abc.local\": Name or service not known" ) );
  EXPECT_TRUE( shouldIgnoreGStreamerLog(
      "webrtcnice", "Could not resolve candidate address: Error resolving \xE2\x80\x9C"
                    "abc.local\xE2\x80\x9D: Name or service not known" ) );
  EXPECT_FALSE( shouldIgnoreGStreamerLog( "webrtcnice", "ICE failed for some other reason" ) );
}

} // namespace ros_camera_server::detail
