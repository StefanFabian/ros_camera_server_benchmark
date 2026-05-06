# Results

Latency columns are milliseconds. `p50_med` / `p50_min` / `p50_max`
are the median, min, and max of per-run p50 across 5 repeats;
`p95_med` is the median of per-run p95. `cpu_mean` / `cpu_p95` are
%CPU summed across the stack-under-test processes only (Linux/`top`
convention: 100% per fully-loaded logical core); harness producers
and receivers are excluded. **Bold rows are `ros_camera_server`** (the
subject under test). Definitions, fairness controls, and caveats are
in [METHODOLOGY.md](METHODOLOGY.md).

## Notebook

- **CPU**: 13th Gen Intel Core i9-13900H (20 logical cores)
- **iGPU (VA-API)**: Intel Raptor Lake-P Iris Xe Graphics
- **OS**: Ubuntu 24.04.4 LTS, kernel 6.17.0-23-generic
- **GStreamer**: 1.24.2
- **Encoder**: VA-API (`vah264lpenc`) — `BENCH_ENCODER=va`

### cam-both — camera → ROS image + H.264 stream

`ros_camera_server` (RTP and WebRTC) leads on stream latency by ~3 ms
over the next non-rcs stack; on the ROS topic side, the rcs `mjpeg`
passthrough at 1.89 ms is uniquely cheap (no decode), and the rcs raw
paths (~3.0 ms) tie the camera-driver baselines.

| Stack                                 | recv         |  p50_med |  p50_min |  p50_max |  p95_med |  cpu_mean |   cpu_p95 |
| ------------------------------------- | ------------ | -------: | -------: | -------: | -------: | --------: | --------: |
| cam-both-gscam+gst_bridge-mjpeg-rtp   | stream       |     9.73 |     9.35 |    10.58 |    11.99 |     22.31 |     26.50 |
| cam-both-gscam+gst_bridge-mjpeg-rtp   | ros_raw      |     4.21 |     4.08 |     4.67 |     5.75 |     22.31 |     26.50 |
| cam-both-gscam+gst_bridge-yuv-rtp     | stream       |     8.74 |     8.58 |     8.83 |     9.60 |     18.35 |     20.75 |
| cam-both-gscam+gst_bridge-yuv-rtp     | ros_raw      |     3.40 |     3.35 |     3.45 |     3.96 |     18.35 |     20.75 |
| **cam-both-rcs-mjpeg-decoded-rtp**    | **stream**   | **5.80** | **5.76** | **5.87** | **6.45** | **15.23** | **22.75** |
| **cam-both-rcs-mjpeg-decoded-rtp**    | **ros_raw**  | **4.61** | **4.53** | **4.64** | **6.38** | **15.23** | **22.75** |
| **cam-both-rcs-mjpeg-rtp**            | **stream**   | **5.63** | **5.55** | **5.76** | **6.31** |  **6.27** | **10.75** |
| **cam-both-rcs-mjpeg-rtp**            | **ros_jpeg** | **1.89** | **1.82** | **1.97** | **2.49** |  **6.27** | **10.75** |
| **cam-both-rcs-yuv-rtp**              | **stream**   | **5.18** | **5.08** | **5.29** | **5.74** | **12.96** | **14.00** |
| **cam-both-rcs-yuv-rtp**              | **ros_raw**  | **3.03** | **2.97** | **3.17** | **3.45** | **12.96** | **14.00** |
| **cam-both-rcs-yuv-webrtc**           | **stream**   | **5.28** | **5.20** | **5.42** | **6.04** | **12.27** | **14.00** |
| **cam-both-rcs-yuv-webrtc**           | **ros_raw**  | **3.02** | **2.96** | **3.09** | **3.62** | **12.27** | **14.00** |
| cam-both-usb_cam+gst_bridge-mjpeg-rtp | stream       |     9.55 |     9.37 |     9.86 |    10.87 |     21.58 |     25.50 |
| cam-both-usb_cam+gst_bridge-mjpeg-rtp | ros_raw      |     4.14 |     4.11 |     4.23 |     5.21 |     21.58 |     25.50 |
| cam-both-usb_cam+gst_bridge-yuv-rtp   | stream       |     8.67 |     8.49 |     8.84 |     9.69 |     18.38 |     21.50 |
| cam-both-usb_cam+gst_bridge-yuv-rtp   | ros_raw      |     3.29 |     3.26 |     3.40 |     3.85 |     18.38 |     21.50 |
| cam-both-usb_cam+rtsp_fkie-yuv-h264   | stream       |     7.14 |     6.99 |     7.31 |     8.77 |     14.96 |     17.75 |
| cam-both-usb_cam+rtsp_fkie-yuv-h264   | ros_raw      |     3.47 |     3.33 |     3.65 |     4.98 |     14.96 |     17.75 |

### cam-ros — camera → ROS image

`rcs-mjpeg` (compressed topic) is uniquely cheap because no decode
happens. The three raw YUV publishers (`rcs-yuv`, `usb_cam-yuv`,
`gscam-yuv`) are within 0.13 ms of each other.

| Stack                         | recv         |  p50_med |  p50_min |  p50_max |  p95_med | cpu_mean |  cpu_p95 |
| ----------------------------- | ------------ | -------: | -------: | -------: | -------: | -------: | -------: |
| cam-ros-gscam-mjpeg           | ros_raw      |     3.54 |     3.48 |     3.60 |     4.04 |     9.38 |    10.75 |
| cam-ros-gscam-yuv             | ros_raw      |     2.88 |     2.84 |     3.07 |     3.31 |     7.04 |     8.00 |
| **cam-ros-rcs-mjpeg**         | **ros_jpeg** | **1.80** | **1.67** | **1.86** | **2.20** | **0.62** | **2.00** |
| **cam-ros-rcs-mjpeg-decoded** | **ros_raw**  | **4.52** | **4.38** | **4.60** | **5.70** | **9.23** |**10.00** |
| **cam-ros-rcs-yuv**           | **ros_raw**  | **2.93** | **2.87** | **2.99** | **3.37** | **6.54** | **8.00** |
| cam-ros-usb_cam-mjpeg         | ros_raw      |     3.56 |     3.50 |     3.63 |     4.02 |     9.62 |    11.75 |
| cam-ros-usb_cam-yuv           | ros_raw      |     2.80 |     2.74 |     2.84 |     3.16 |     6.96 |     8.00 |

### cam-stream — camera → encoded stream

`rcs-yuv-rtp` and `rcs-yuv-webrtc` lead at ~0.4–0.6 ms above the bare
gst-launch UDP-RTP floor (4.59 ms). `rtsp_fkie` is the closest
non-rcs stack; `gst_bridge`-based stacks add ~3.5 ms over the floor.

| Stack                                         | recv       |  p50_med |  p50_min |  p50_max |  p95_med | cpu_mean |  cpu_p95 |
| --------------------------------------------- | ---------- | -------: | -------: | -------: | -------: | -------: | -------: |
| cam-stream-floor                              | stream     |     4.59 |     4.53 |     4.68 |     4.96 |     3.88 |     6.00 |
| **cam-stream-rcs-yuv-rtp**                    | **stream** | **4.95** | **4.86** | **5.09** | **5.52** | **5.88** | **7.50** |
| **cam-stream-rcs-yuv-webrtc**                 | **stream** | **5.17** | **5.06** | **5.29** | **5.69** | **5.12** | **6.75** |
| cam-stream-usb_cam+rtsp_fkie-yuv-h264         | stream     |     6.53 |     6.43 |     6.57 |     7.68 |    12.69 |    15.50 |
| cam-stream-usb_cam+gst_bridge-yuv-rtp         | stream     |     8.17 |     7.98 |     8.25 |     8.84 |    16.31 |    18.75 |
| cam-stream-gscam+gst_bridge-yuv-rtp           | stream     |     8.24 |     8.16 |     8.31 |     9.18 |    16.42 |    18.00 |
| cam-stream-usb_cam+web_video_server-yuv-mjpeg | stream     |     8.30 |     7.32 |     8.49 |     9.72 |    26.31 |    29.75 |

### ros-stream — ROS image input → encoded stream

`rtsp_fkie` reports the lowest p50 (4.89 ms), but it does so without
the 100 ms jitter buffer a generic RTSP client adds — see the
`rtsp_image_transport` caveat in [METHODOLOGY.md](METHODOLOGY.md#caveats).
The `rcs` and `gst_bridge` paths cluster at 6.5–6.9 ms.

| Stack                                 | recv       |  p50_med |  p50_min |  p50_max |  p95_med |  cpu_mean |   cpu_p95 |
| ------------------------------------- | ---------- | -------: | -------: | -------: | -------: | --------: | --------: |
| ros-stream-gst_bridge-raw-rtp         | stream     |     6.53 |     6.41 |     8.83 |     7.20 |      9.27 |     10.00 |
| **ros-stream-rcs-jpeg-rtp**           | **stream** | **6.57** | **6.49** | **6.82** | **7.46** |  **8.58** | **16.00** |
| **ros-stream-rcs-jpeg-webrtc**        | **stream** | **6.62** | **6.52** | **6.84** | **7.65** |  **7.62** | **10.00** |
| **ros-stream-rcs-raw-rtp**            | **stream** | **6.68** | **6.59** | **8.87** | **7.53** | **10.50** | **14.00** |
| **ros-stream-rcs-raw-webrtc**         | **stream** | **6.83** | **6.71** | **9.14** | **7.69** | **10.00** | **12.00** |
| ros-stream-rtsp_fkie-raw-h264         | stream     |     4.89 |     4.79 |     4.95 |     5.71 |      5.77 |      6.75 |
| ros-stream-web_video_server-raw-mjpeg | stream     |     6.36 |     5.99 |     8.24 |     7.85 |     15.08 |     17.75 |