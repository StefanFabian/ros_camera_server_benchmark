# ros_camera_server_benchmarks

End-to-end latency benchmark harness comparing [`ros_camera_server`](https://www.github.com/StefanFabian/ros_camera_server)
against `usb_cam`, `gscam`, `ros-gst-bridge`, `web_video_server`,
`rtsp_image_transport`, and `ffmpeg_image_transport`. Each frame carries
a pixel-domain marker with a `CLOCK_MONOTONIC` timestamp; receivers decode
it after their normal display path and report producer-to-consumer latency.

The methodology, fairness controls, pipeline commands, and cost-layer
breakdown live in [METHODOLOGY.md](METHODOLOGY.md).
Setup, run instructions, full env-var reference, and the per-scenario
matrix live in [REPRODUCING.md](REPRODUCING.md).

## Stacks compared

| Stack                    | Role                   | Transports exercised         |
| ------------------------ | ---------------------- | ---------------------------- |
| `ros_camera_server`      | Subject under test     | UDP-RTP, WebRTC, ROS topic   |
| `usb_cam`                | V4L2 → ROS driver      | ROS topic (+ bridge to RTP)  |
| `gscam`                  | V4L2 → ROS driver      | ROS topic (+ bridge to RTP)  |
| `ros-gst-bridge`         | ROS → GStreamer bridge | UDP-RTP H.264                |
| `web_video_server`       | ROS → HTTP             | HTTP MJPEG                   |
| `rtsp_image_transport`   | ROS → RTSP (FKIE)      | RTSP H.264                   |
| `ffmpeg_image_transport` | image_transport plugin | H.264 (libav) over ROS topic |

## Scenario groups

| Group        | Measures                         |
| ------------ | -------------------------------- |
| `cam-ros`    | Camera → ROS topic only          |
| `cam-stream` | Camera → encoded stream (no ROS) |
| `ros-stream` | ROS image input → encoded stream |
| `cam-both`   | Camera → both stream + ROS topic |

## Headline takeaway

On the notebook results in [RESULTS.md](RESULTS.md):

- **Lowest end-to-end latency on the camera-sourced encoded-stream
  groups (cam-stream, cam-both) is `ros_camera_server`.** Its
  `yuv-rtp` and `yuv-webrtc` paths sit ~0.9–2.0 ms above the bare
  gst-launch UDP-RTP floor (8.44 ms), with WebRTC slightly ahead of
  RTP (cam-stream: 9.29 ms vs 10.23 ms; cam-both: 9.44 ms vs 10.39 ms).
- **`rtsp_image_transport` (FKIE)** is the closest non-rcs stream
  stack at ~11.5–16 ms (and on `ros-stream` it edges below rcs only by
  dropping the ~100 ms jitter buffer a real RTSP client adds — see the
  caveat in [METHODOLOGY.md](METHODOLOGY.md#caveats)).
- **`gst_bridge`-based stacks** add ~8 ms over the floor — that gap
  is pipeline overhead, not framing cost (the wire format is identical).
- **On `cam-ros` (raw topic only)** `rcs-yuv`, `usb_cam-yuv`, and
  `gscam-yuv` are within ~0.3 ms of each other (~6.4–6.6 ms).
  `cam-ros-rcs-mjpeg` passthrough is uniquely cheap (2.44 ms) because
  the ROS topic carries `CompressedImage` and no decode happens.

In summary, this shows that despite the significantly easier configuration, the [ros_camera_server](https://www.github.com/StefanFabian/ros_camera_server) is at least competitive and in some cases even faster than alternative tuned approaches.

## Plots

![Result plot](results/notebook/plots/cam-both.png)
![Result plot](results/notebook/plots/cam-ros.png)
![Result plot](results/notebook/plots/cam-stream.png)
![Result plot](results/notebook/plots/ros-stream.png)

## Further reading

- [RESULTS.md](RESULTS.md) — full per-scenario tables.
- [METHODOLOGY.md](METHODOLOGY.md) — marker, fairness controls,
  pipeline commands, caveats, and stacks intentionally excluded.
- [REPRODUCING.md](REPRODUCING.md) — setup, env vars, scenario matrix.
