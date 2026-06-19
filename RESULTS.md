# Results

Latency columns are milliseconds. `p50_med` / `p50_min` / `p50_max`
are the median, min, and max of per-run p50 across 20 repeats;
`p95_med` is the median of per-run p95. `cpu_mean` / `cpu_p95` are
%CPU summed across the stack-under-test processes only (Linux/`top`
convention: 100% per fully-loaded logical core); harness producers
and receivers are excluded. A `—` in the CPU columns means the stack
encodes inside the producer process (`ros-stream-ffmpeg_transport`),
so there is no separate stack-under-test PID to sample. **Bold rows are
`ros_camera_server`** (the subject under test). Definitions, fairness
controls, and caveats are in [METHODOLOGY.md](METHODOLOGY.md).

Operating point: 1920x1080 @ 30 fps, 1 Mbps H.264, 30 s per run,
20 repeats per scenario.

## Notebook

- **CPU**: 13th Gen Intel Core i9-13900H (20 logical cores)
- **iGPU (VA-API)**: Intel Raptor Lake-P Iris Xe Graphics
- **OS**: Ubuntu 24.04.4 LTS, kernel 6.17.0-35-generic
- **GStreamer**: 1.24.2
- **Encoder**: VA-API (`vah264lpenc`) — `BENCH_ENCODER=va`

### cam-both — camera → ROS image + H.264 stream

`ros_camera_server` (RTP and WebRTC) leads on stream latency by ~6 ms
over the next non-rcs stack (`rcs-yuv-webrtc` 9.44 ms vs
`usb_cam+rtsp_fkie` 15.96 ms). On the ROS topic side, the rcs `mjpeg`
passthrough at 2.60 ms is uniquely cheap (no decode), and the rcs raw
paths (~6.5 ms) edge out the camera-driver baselines (7.5–9.8 ms).

| scenario                                      | recv         |   p50_med |   p50_min |   p50_max |   p95_med |  cpu_mean |   cpu_p95 |
| --------------------------------------------- | ------------ | --------: | --------: | --------: | --------: | --------: | --------: |
| cam-both-gscam+gst_bridge-mjpeg-rtp           | stream       |     18.12 |     17.90 |     20.41 |     19.18 |     41.09 |     44.00 |
| cam-both-gscam+gst_bridge-mjpeg-rtp           | ros_raw      |      8.02 |      7.85 |     10.30 |      8.61 |     41.09 |     44.00 |
| cam-both-gscam+gst_bridge-yuv-rtp             | stream       |     19.96 |     17.55 |     20.20 |     20.74 |     46.66 |     49.25 |
| cam-both-gscam+gst_bridge-yuv-rtp             | ros_raw      |      9.81 |      7.68 |     10.02 |     10.28 |     46.66 |     49.25 |
| **cam-both-rcs-mjpeg-decoded-rtp**            | **stream**   |  **8.95** |  **8.53** |  **9.11** |  **9.87** | **21.48** | **24.25** |
| **cam-both-rcs-mjpeg-decoded-rtp**            | **ros_raw**  |  **8.87** |  **8.80** | **11.23** |  **9.55** | **21.48** | **24.25** |
| **cam-both-rcs-mjpeg-rtp**                    | **stream**   |  **8.75** |  **8.58** |  **8.95** |  **9.65** |  **3.64** |  **6.00** |
| **cam-both-rcs-mjpeg-rtp**                    | **ros_jpeg** |  **2.60** |  **2.46** |  **2.70** |  **3.22** |  **3.64** |  **6.00** |
| **cam-both-rcs-yuv-rtp**                      | **stream**   | **10.39** | **10.15** | **10.56** | **11.22** | **24.82** | **29.25** |
| **cam-both-rcs-yuv-rtp**                      | **ros_raw**  |  **6.56** |  **6.43** |  **9.02** |  **6.94** | **24.82** | **29.25** |
| **cam-both-rcs-yuv-webrtc**                   | **stream**   |  **9.44** |  **9.34** | **10.88** | **10.63** | **23.80** | **26.00** |
| **cam-both-rcs-yuv-webrtc**                   | **ros_raw**  |  **6.54** |  **6.42** |  **9.11** |  **6.96** | **23.80** | **26.00** |
| cam-both-usb_cam+ffmpeg_transport-yuv-h264    | stream       |     21.35 |     21.21 |     21.41 |     22.41 |     36.48 |     38.00 |
| cam-both-usb_cam+ffmpeg_transport-yuv-h264    | ros_raw      |     14.65 |     14.52 |     14.81 |     15.43 |     36.48 |     38.00 |
| cam-both-usb_cam+gst_bridge-mjpeg-rtp         | stream       |     17.88 |     17.60 |     18.06 |     18.97 |     40.59 |     42.50 |
| cam-both-usb_cam+gst_bridge-mjpeg-rtp         | ros_raw      |      7.75 |      7.62 |      7.90 |      8.44 |     40.59 |     42.50 |
| cam-both-usb_cam+gst_bridge-yuv-rtp           | stream       |     17.59 |     17.40 |     17.74 |     18.54 |     38.27 |     40.00 |
| cam-both-usb_cam+gst_bridge-yuv-rtp           | ros_raw      |      7.49 |      7.43 |      7.56 |      7.93 |     38.27 |     40.00 |
| cam-both-usb_cam+rtsp_fkie-yuv-h264           | stream       |     15.96 |     15.81 |     16.09 |     16.75 |     30.79 |     34.00 |
| cam-both-usb_cam+rtsp_fkie-yuv-h264           | ros_raw      |      7.54 |      7.40 |      7.59 |      8.87 |     30.79 |     34.00 |

### cam-ros — camera → ROS image

`rcs-mjpeg` (compressed topic) is uniquely cheap (2.44 ms) because no
decode happens. The three raw YUV publishers (`rcs-yuv` 6.41,
`usb_cam-yuv` 6.35, `gscam-yuv` 6.64) are within ~0.3 ms of each other;
`rcs-mjpeg-decoded` (10.55 ms) pays the in-server MJPEG→rgb8 decode.

| scenario                                      | recv         |   p50_med |   p50_min |   p50_max |   p95_med |  cpu_mean |   cpu_p95 |
| --------------------------------------------- | ------------ | --------: | --------: | --------: | --------: | --------: | --------: |
| cam-ros-gscam-mjpeg                           | ros_raw      |      6.84 |      6.65 |      9.10 |      7.40 |     18.09 |     20.00 |
| cam-ros-gscam-yuv                             | ros_raw      |      6.64 |      6.39 |      8.87 |      6.95 |     15.88 |     18.75 |
| **cam-ros-rcs-mjpeg**                         | **ros_jpeg** |  **2.44** |  **2.32** |  **2.53** |  **2.99** |  **0.82** |  **2.00** |
| **cam-ros-rcs-mjpeg-decoded**                 | **ros_raw**  | **10.55** |  **9.27** | **11.76** | **11.14** | **22.55** | **25.00** |
| **cam-ros-rcs-yuv**                           | **ros_raw**  |  **6.41** |  **6.35** |  **8.75** |  **6.77** | **14.45** | **16.50** |
| cam-ros-usb_cam-mjpeg                         | ros_raw      |      6.56 |      6.39 |      6.63 |      7.24 |     17.54 |     20.00 |
| cam-ros-usb_cam-yuv                           | ros_raw      |      6.35 |      6.20 |      6.41 |      6.67 |     15.23 |     16.50 |

### cam-stream — camera → encoded stream

`rcs-yuv-webrtc` (9.29) and `rcs-yuv-rtp` (10.23) lead at ~0.9–1.8 ms
above the bare gst-launch UDP-RTP floor (8.44 ms). `rtsp_fkie` (14.76)
is the closest non-rcs stack; `gst_bridge`-based stacks add ~8 ms over
the floor, and the `ffmpeg_image_transport` arm is the most expensive
(21.0 ms).

| scenario                                      | recv       |   p50_med |   p50_min |   p50_max |   p95_med |  cpu_mean |   cpu_p95 |
| --------------------------------------------- | ---------- | --------: | --------: | --------: | --------: | --------: | --------: |
| cam-stream-floor                              | stream     |      8.44 |      8.31 |      8.48 |      9.41 |      8.11 |     10.00 |
| cam-stream-gscam+gst_bridge-yuv-rtp           | stream     |     16.54 |     16.42 |     16.67 |     17.34 |     34.52 |     36.50 |
| **cam-stream-rcs-yuv-rtp**                    | **stream** | **10.23** | **10.10** | **10.33** | **11.04** |  **9.54** | **12.00** |
| **cam-stream-rcs-yuv-webrtc**                 | **stream** |  **9.29** |  **9.19** |  **9.39** | **10.02** |  **9.21** | **11.25** |
| cam-stream-usb_cam+ffmpeg_transport-yuv-h264  | stream     |     21.00 |     20.82 |     21.23 |     21.95 |     32.29 |     34.00 |
| cam-stream-usb_cam+gst_bridge-yuv-rtp         | stream     |     16.26 |     16.12 |     16.39 |     16.99 |     34.02 |     36.00 |
| cam-stream-usb_cam+rtsp_fkie-yuv-h264         | stream     |     14.76 |     14.62 |     14.88 |     15.42 |     27.14 |     30.00 |
| cam-stream-usb_cam+web_video_server-yuv-mjpeg | stream     |     15.11 |     14.02 |     15.31 |     16.74 |     49.88 |     54.00 |

### ros-stream — ROS image input → encoded stream

`rtsp_fkie` reports the lowest p50 (11.45 ms), but without the ~100 ms
jitter buffer a generic RTSP client adds — see the
`rtsp_image_transport` caveat in [METHODOLOGY.md](METHODOLOGY.md#caveats).
The rcs jpeg paths (12.2–13.2 ms) sit just behind `web_video_server`
(11.76 ms); the rcs raw paths (15.6–16.6 ms) bracket `gst_bridge`
(15.23 ms), and `ffmpeg_image_transport` is highest (22.48 ms).

| scenario                                      | recv       |   p50_med |   p50_min |   p50_max |   p95_med |  cpu_mean |   cpu_p95 |
| --------------------------------------------- | ---------- | --------: | --------: | --------: | --------: | --------: | --------: |
| ros-stream-ffmpeg_transport-raw-h264          | stream     |     22.48 |     17.66 |     22.62 |     23.38 |         — |         — |
| ros-stream-gst_bridge-raw-rtp                 | stream     |     15.23 |     15.11 |     20.16 |     16.23 |     19.07 |     20.00 |
| **ros-stream-rcs-jpeg-rtp**                   | **stream** | **13.16** | **12.79** | **15.96** | **14.52** | **13.38** | **16.00** |
| **ros-stream-rcs-jpeg-webrtc**                | **stream** | **12.18** | **11.84** | **15.25** | **13.46** | **13.30** | **16.00** |
| **ros-stream-rcs-raw-rtp**                    | **stream** | **16.56** | **16.37** | **21.37** | **17.80** | **19.18** | **20.00** |
| **ros-stream-rcs-raw-webrtc**                 | **stream** | **15.63** | **15.48** | **20.44** | **16.61** | **19.14** | **20.00** |
| ros-stream-rtsp_fkie-raw-h264                 | stream     |     11.45 |     11.39 |     11.51 |     12.17 |     12.43 |     14.00 |
| ros-stream-web_video_server-raw-mjpeg         | stream     |     11.76 |     11.67 |     16.50 |     13.25 |     35.50 |     38.00 |
