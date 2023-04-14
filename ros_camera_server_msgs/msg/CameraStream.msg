# Transport can be one of: ros2, srt, rtp
string transport
# The codec used for the stream (e.g. h264, vp8, vp9, ...)
# If transport is ros2, this is the name of the transport
string codec
# The uri of the stream (e.g. srt://localhost:7001)
# If transport is ros2, this is the topic name
string uri
uint16 width
uint16 height
float32 framerate
