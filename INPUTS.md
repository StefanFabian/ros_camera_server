# Inputs

```yaml
robot: "my_robot"
address: "192.168.1.100"   # Address advertised to clients in stream URIs
signaling_port: 8443 # For webrtc outputs, the port of the signaling server
cameras:
  front_camera:
    name: "Front Camera"
    input:
      type: v4l2
      device: "/dev/video0"
      width: 1920
      height: 1080
      framerate: "30/1"
    outputs:
      - ...
```

Each camera has exactly one input. See [README.md](README.md#%EF%B8%8F-configuration) for the surrounding config structure and [OUTPUTS.md](OUTPUTS.md) for output types.

## V4L2

Captures from a Video4Linux2 device.

```yaml
input:
  type: v4l2
  device: "/dev/video0"
  width: 1920
  height: 1080
  framerate: "30/1"
  # controls:             # Optional: initial V4L2 controls (also settable via ros2 param set)
  #   brightness: 128
```

> [!TIP]
> Use `ros2 run ros_camera_server v4l2_caps_investigator` to discover supported resolutions and framerates, and `ros2 run ros_camera_server v4l2_controls_inspector` to list available controls.

V4L2 controls can be set in the config and are also exposed as reconfigurable ROS 2 parameters enabling changes at runtime:

```bash
ros2 param set /ros_camera_server front_camera.controls.brightness 200
```

> [!TIP]
> While you can change parameters using command line, we recommend using [RQml](https://github.com/StefanFabian/rqml)

## ROS 2

Subscribes to a ROS 2 image topic.

```yaml
input:
  type: ros2
  topic: "/image_raw"
  format: raw     # raw | jpeg | png
  # framerate: "15/1"   # Optional but recommended; auto-detected from the first 5 frames if omitted
```

## RTP

Receives an RTP video stream over UDP (unicast or multicast). The stream codec must be declared because RTP itself does not advertise it.

```yaml
input:
  type: rtp
  codec: h264              # h264 | h265 | jpeg
  port: 5004               # UDP port to listen on
  address: "0.0.0.0"       # bind address; for multicast, the multicast group to join
  multicast: false         # set true to enable IGMP join on `address`
  payload_type: 96         # optional; default 96
  latency_ms: 200          # optional; rtpjitterbuffer latency in milliseconds (default 200)
  drop_on_latency: false   # optional; drop late packets instead of waiting (default false)
  # framerate: "30/1"      # optional specify the framerate if known
  # decoder: auto          # optional decoder backend preference (see Decoder selection below)
```

`rtpjitterbuffer` smooths sequence-number reordering. If the sender embeds an `x-timestamp` RTP header extension (this server does on its RTP/SRT/WebRTC outputs), the original capture time is recovered and propagated through the pipeline as a `timestamp/x-unix` reference.

## Decoder selection

When an input produces an encoded format (H.264, H.265, JPEG, PNG) and a downstream consumer needs raw video — for example a ROS 2 `format: raw` output, a different output codec, or a scaling/framerate transform — the pipeline inserts a decoder. The optional `decoder:` field selects the preferred backend using the same syntax as the output `encoder:` field (see [OUTPUTS.md](OUTPUTS.md)). Pipe-delimited tokens express priority order; `auto` (the default) tries hardware backends first and falls back to software, hot-swapping at runtime if a backend stops producing frames.

```yaml
decoder: auto       # default — VA / NV / NVV4L2 / VAAPI hardware first, software fallback
decoder: sw         # force software (avdec_h264 / openh264dec / x265dec / pngdec / etc.)
decoder: nv|sw      # prefer NVIDIA, then any software backend
```

The field is ignored when no decoder is needed (e.g. raw inputs, or codec-passthrough outputs).

## SeekThermal

Captures from a SeekThermal camera via the `openseekthermalsrc` GStreamer element.

```yaml
input:
  type: openseekthermal
  # Optional: Bus 1, hub on port 3 and port 2 of that hub (if not specified uses first available)
  device: "1-3.2"
  # Optional: serial number to select a specific camera (not all models have one, though)
  serial: ""
```
