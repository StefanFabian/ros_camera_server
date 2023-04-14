# Outputs

```yaml
robot: "my_robot"
address: "192.168.1.100"   # Address advertised to clients in stream URIs
signaling_port: 8443 # For webrtc outputs, the port of the signaling server
cameras:
  front_camera:
    name: "Front Camera"
    input:
      ... See INPUTS.md
    outputs:
      - type: srt
        codec: h265
        port: 7100
        bitrate: 1500
```

Each camera has one or more outputs. Output `width` and `height` are upper limits - aspect ratio is preserved and frames are only downscaled, never upscaled. See [README.md](README.md#%EF%B8%8F-configuration) for the surrounding config structure and [INPUTS.md](INPUTS.md) for input types.

> [!NOTE]
> All outputs have the properties `width`, `height` and `framerate` as upper limits.
> Even if they are not used in every example here.

## SRT

Streams encoded video over [SRT (Secure Reliable Transport)](https://www.haivision.com/products/srt-technology/).

```yaml
- type: srt
  codec: h265         # h264 | h265
  port: 7100
  bitrate: 800        # in kbps
  width: 1920
  height: 1080
  framerate: "30/1"   # Limits framerate (frames are dropped, never duplicated)
  latency: 100        # SRT buffer latency in ms, optional, default: 100ms
```

See [README.md](README.md#-viewing-an-srt-stream) for client commands.

## ROS 2

Publishes frames back to a ROS 2 image topic.

```yaml
- type: ros2
  topic: "/camera_server/front"
  frame_id: "front_camera_optical_frame"
  codec: auto         # auto | raw | compressed
  format: "rgb8"      # Only used when codec is raw
                      # Supported: rgb8 | bgr8 | rgba8 | bgra8 | mono8 | mono16 | uyvy | yuyv | nv21 | nv24
  camera_info_url: "file:///home/user/.ros/camera_info/front_camera.yaml"  # Optional
```

`frame_id` is written to both published image headers and `CameraInfo` headers.
If `camera_info_url` is set, `rbfimagesink` publishes `sensor_msgs/msg/CameraInfo` on standard sibling topic of `topic`.
Examples: `topic: /camera/image_raw` -> `/camera/camera_info`, `topic: /camera_server/front` -> `/camera_server/camera_info`.
Supported URL schemes include `file:///absolute/path/to/calibration.yaml` and `package://my_pkg/calibrations/front.yaml`.
The calibration has to be for the resolution of the output, otherwise it will be rejected and not published.

## WebRTC

Streams via WebRTC using a built-in signaling server.

```yaml
- type: webrtc
  codec: h264         # h264 | h265
  width: 1920
  height: 1080
  framerate: "30/1"
  bitrate: 2000       # in kbps
```

Configure the signaling server port at the top level:

```yaml
robot: "my_robot"
address: "192.168.1.100"
signaling_port: 8443
```

> [!NOTE]
> You can also open [127.0.0.1:8443](127.0.0.1:8443) or whatever your signaling port is set to, for a small web page that lists your webrtc streams and allows to connect to them for live video.
> Some browsers only support H.264 for this at the moment.
