# Quickstart

Get a camera streaming in a few minutes. Assumes [build steps from README](README.md#%EF%B8%8F-building-from-source) done and workspace sourced.

## 1. Find Cameras

List all V4L2 devices on the system with their supported formats, resolutions, and framerates:

```bash
ros2 run ros_camera_server v4l2_caps_investigator
```

Sample output:

```bash
/dev/video0
  Name: HD Pro Webcam C920
  video/x-raw
    format: YUY2
      640 x 480    @  30/1, 24/1, 20/1, 15/1, 10/1, 7/1, 5/1
      1280 x 720   @  10/1, 7/1, 5/1
      1920 x 1080  @  5/1
  image/jpeg
    format: (default)
      640 x 480    @  30/1, 24/1, 20/1, 15/1, 10/1, 7/1, 5/1
      1280 x 720   @  30/1, 24/1, 20/1, 15/1, 10/1, 7/1, 5/1
      1920 x 1080  @  30/1, 24/1, 20/1, 15/1, 10/1, 7/1, 5/1
```

Inspect a single device:

```bash
ros2 run ros_camera_server v4l2_caps_investigator /dev/video0
```

Pick one resolution + framerate combination from the listing - those are the values your config must use.

> [!TIP]
> Many webcams expose both raw (`video/x-raw`, e.g. YUY2) and `image/jpeg` paths. MJPEG usually allows higher resolutions/framerates but adds a decode step on the host.

## 2. Find Available Controls (optional)

To see which V4L2 controls (brightness, exposure, focus, ...) the device exposes:

```bash
ros2 run ros_camera_server v4l2_controls_inspector /dev/video0
```

These can be set in config under `input.controls` and changed at runtime via `ros2 param set`.

## 3. Check Available Encoders (optional)

List GStreamer encoders detected on the system and how many concurrent instances each can run:

```bash
ros2 run ros_camera_server enumerate_encoders
```

Hardware encoders (NVENC, VA-API, Rockchip MPP) are picked automatically when present; software fallback is used otherwise. No config needed.
You can force the camera server to use a specific encoder but that is not recommended.
Best leave it at `auto` and let the software choose the encoder automatically.

## 4. Write a Config

Create `my_cameras.yaml` based on values from step 1:

```yaml
robot: "my_robot"
address: "127.0.0.1"      # Address advertised to clients in stream URIs
signaling_port: 8443      # Only needed for WebRTC outputs

cameras:
  webcam:
    name: "My Webcam"
    input:
      type: v4l2
      device: "/dev/video0"
      width: 1280
      height: 720
      framerate: "30/1"
      # media_type: "image/jpeg"   # Uncomment if caps listed it under image/jpeg
    outputs:
      - type: srt
        codec: h265
        port: 7100
        bitrate: 1500       # kbps
      - type: ros2
        topic: "/camera_server/webcam"
        frame_id: "webcam_optical_frame"
        codec: auto
```

Reference: full annotated example at [ros_camera_server/config/config.yaml](ros_camera_server/config/config.yaml).

> [!TIP]
> You can also specify `width`, `height` and `framerate` at the output to scale down and/or
> reduce the framerate of the output.

## 5. Launch

```bash
ros2 launch ros_camera_server server.launch.yaml config_path:=/absolute/path/to/my_cameras.yaml
```

## 6. View the Stream

```bash
gst-launch-1.0 -v srtsrc uri=srt://127.0.0.1:7100 ! application/x-rtp ! rtph265depay ! h265parse ! decodebin ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! autovideosink sync=false
```

Or view the ROS live video using [RQml](https://github.com/StefanFabian/rqml):

* Launch `rqml`
* Press `Ctrl+P` to open the add plugins dialog or choose from Plugins menu
* Add ImageView and select topic

## Next Steps

* Add more cameras under `cameras:` - each gets its own input + outputs.
* Add WebRTC outputs and open `http://<address>:<signaling_port>` in a browser to view them.
* See [README.md](README.md#%EF%B8%8F-configuration) for all input/output types and options.
