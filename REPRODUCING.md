# Reproducing

Setup, run, and tunables for the harness. The methodology behind the
numbers lives in [METHODOLOGY.md](METHODOLOGY.md); the headline
results in the [project README](README.md).

## Setup

### System packages (apt)

```bash
sudo apt install v4l2loopback-dkms v4l-utils \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libjpeg-turbo8-dev
```

The RTP payloaders / depayloaders (`rtph264pay`, `rtph264depay`) ship
in `gstreamer1.0-plugins-good`; both VA-API (`vah264lpenc`,
`vah264dec` — Intel) and NVIDIA (`nvh264enc`, `nvh264dec`)
encoders/decoders ship in `gstreamer1.0-plugins-bad`. The Rockchip
hardware encoder (`mpph264enc`, paired with `mpph264dec`) is exposed
through the GStreamer Rockchip MPP plugin, available on
RK3399 / RK3566 / RK3568 / RK3588 boards via `gstreamer1.0-rockchip`
in vendor BSP / Armbian / Debian-Rockchip repos, or built from source
(e.g. `JeffyCN/mpp` + `nyanmisaka/gst-mpp`). Confirm whichever family
you intend to test with `gst-inspect-1.0 vah264lpenc`,
`gst-inspect-1.0 nvh264enc`, and/or `gst-inspect-1.0 mpph264enc`
before running benchmarks. The `ros_camera_server` configs use
`envsubst` to drop `BENCH_RCS_ENC` (set by `env_fairness.sh`) into each
`encoder:` field; `envsubst` is provided by `gettext-base` (almost
always installed already — check `/usr/bin/envsubst`).

### Stacks-under-test

Each scenario depends on a different subset; if a stack isn't
installed, `run_scenario.sh` prints a clear message and skips that
scenario.

| Source           | Packages                                                     | Required for                                               |
| ---------------- | ------------------------------------------------------------ | ---------------------------------------------------------- |
| this workspace   | `ros_camera_server`                                          | all `*-rcs-*` scenarios                                    |
| ROS distro (apt) | `usb_cam` (`ros-<distro>-usb-cam`)                           | `*-usb_cam-*`, `*-web_video_server-*`, `*-rtsp_fkie-*`     |
| ROS distro (apt) | `gscam` (`ros-<distro>-gscam`)                               | `*-gscam-*`                                                |
| source build     | `gst_bridge` (BrettRD/ros-gst-bridge)                        | `*-gst_bridge-*`, `cam-both-usb_cam-*`, `cam-both-gscam-*` |
| ROS distro (apt) | `web_video_server` (`ros-<distro>-web-video-server`)         | `*-web_video_server-*`                                     |
| ROS distro (apt) | `rtsp_image_transport` (`ros-<distro>-rtsp-image-transport`) | `*-rtsp_fkie-*`                                            |

Install rosdep-able packages from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

`gst_bridge` has no rosdep key on most distros — clone into the
workspace and rebuild:

```bash
git clone https://github.com/BrettRD/ros-gst-bridge.git src/ros-gst-bridge
colcon build --packages-up-to gst_bridge
```

Only the `gst_bridge` package itself is needed: the harness invokes
its `rosimagesrc` GStreamer element directly via `gst-launch-1.0`, so
the sister `gst_pipeline` package is not required.

### Build the harness

```bash
# BUILD_TESTING runs the marker roundtrip tests.
colcon build --packages-select ros_camera_server_benchmarks \
  --cmake-args -DBUILD_TESTING=ON
# Optionally, run the tests
colcon test --packages-select ros_camera_server_benchmarks \
  --event-handlers console_direct+

source install/setup.bash
```

### v4l2loopback devices

`/dev/video10` (YUYV) + `/dev/video11` (MJPEG):

```bash
sudo ./setup_loopback.sh
```

### CPU governor

Headline numbers were captured with the CPU governor pinned to
`performance` to remove frequency-scaling jitter from latency tails and
`cpu.csv`:

```bash
sudo cpupower frequency-set -g performance
```

Revert with `-g powersave` (or your distro's default) afterwards. The
runner prints a one-time warning if the governor is anything else;
silence with `BENCH_SKIP_GOVERNOR_CHECK=1` if you accept the noise (e.g.
laptop on battery, dev iteration).

## Run

### Single scenario

```bash
BENCH_DURATION=10 ./run_scenario.sh cam-stream-rcs-yuv-rtp
cat results/cam-stream-rcs-yuv-rtp/summary.json
```

### Full matrix (or one group)

```bash
# Full matrix:
BENCH_REPEATS=5 ./run_all.sh
# Just one group (cam-ros, cam-stream, ros-stream, cam-both):
BENCH_REPEATS=5 BENCH_GROUP=cam-stream ./run_all.sh
column -ts, results/run1/report.csv | less -S
```

### Reproducing the headline tables

```bash
sudo modprobe -r v4l2loopback
sudo bash ros_camera_server_benchmarks/scripts/setup_loopback.sh
source ~/workspaces/claude/setup.bash
BENCH_VIDEO=ros_camera_server_benchmarks/input.robocup_crossing_ramps.mp4 \
  BENCH_VIDEO_CYCLE_S=15 \
  bash ros_camera_server_benchmarks/scripts/run_all.sh
# (BENCH_DURATION=15, BENCH_WARMUP_FRAMES=60, BENCH_MAX_FRAMES=750,
# BENCH_REPEATS=5 are the run_scenario.sh defaults — overrides only
# needed if you want a longer/shorter window.)
# HW Encoder is automatically chosen. If both available, user is asked.
# Use BENCH_ENCODER=va or BENCH_ENCODER=nv to select and skip input.
python3 ros_camera_server_benchmarks/scripts/bench_summary.py \
  --results-root ros_camera_server_benchmarks/results
```

Drop `BENCH_VIDEO` to fall back to the synthetic flat-gray + walking-bar
producer (deflates encoder load — see methodology "Frame content").

### Output layout

Per-run output lands in
`results/runN/<scenario>/{summary.json, stream.csv, ros_raw.csv,
ros_jpeg.csv, frame_gen_stats.csv, cpu.csv, console.log}` (plus a
`*.csv.stats` sidecar per receiver CSV with the percentile breakdown
`bench_aggregate.py` derives), and `results/runN/report.csv` with one
row per `(scenario, receiver)`.

`cpu.csv` records the CPU usage of the stack-under-test (the libraries
being compared, e.g. `camera_server_standalone`, `gst-launch-1.0`,
`usb_cam_node`, `gscam_node`). The harness producers and receivers
(`frame_gen`, `ros_pub`, `gst_recv`, `ros_recv`, `webrtc_recv`) are
deliberately excluded — they are the test instrument, not the subject.
Sampler walks `/proc` at 2 Hz and emits one row per stack process per
tick: `tick_ns,sid,pid,comm,cpu_pct`. `cpu_pct` follows the Linux/`top`
convention: 100% per fully-loaded logical core (>100% on multi-thread
saturation).

`summary.json` carries a `cpu_stack` block aggregated per `comm` —
mean and p95 %CPU after dropping the warmup window
(`warmup_frames / fps` seconds). The synthetic key `_total` is the
per-tick sum across all stack processes (mean and p95 of the totals,
not the sum of per-comm means). `report.csv` exposes `_total` as
`cpu_stack_total_mean_pct` and `cpu_stack_total_p95_pct`. Per-comm
detail stays in `summary.json` only.

## Tunables (env vars)

| Variable                    | Default                        | Purpose                                                                        |
| --------------------------- | ------------------------------ | ------------------------------------------------------------------------------ |
| `BENCH_ENCODER`             | (auto / prompt)                | `va`/`nv`/`mpp`. Selects H.264 encoder family across every stack-under-test.   |
| `BENCH_VIDEO`               | (synthetic flat-gray)          | Path to a video file overlaid behind the marker strip.                         |
| `BENCH_VIDEO_CYCLE_S`       | `5`                            | Seconds of decoded video the loader caches before looping.                     |
| `BENCH_DURATION`            | `15`                           | Per-scenario run duration in seconds.                                          |
| `BENCH_WARMUP_FRAMES`       | `60`                           | Frames discarded at the start of each run.                                     |
| `BENCH_MAX_FRAMES`          | `750`                          | Cap on frames recorded per run after warmup.                                   |
| `BENCH_REPEATS`             | `5`                            | Number of repeats per scenario in `run_all.sh`.                                |
| `BENCH_GROUP`               | (all)                          | Filter for `run_all.sh`: `cam-ros` / `cam-stream` / `ros-stream` / `cam-both`. |
| `BENCH_RESULTS_DIR`         | `${PWD}/results`               | Where per-run output is written.                                               |
| `BENCH_WIDTH`               | `1280`                         | Frame width in px.                                                             |
| `BENCH_HEIGHT`              | `720`                          | Frame height in px.                                                            |
| `BENCH_FPS`                 | `30`                           | Producer frame rate.                                                           |
| `BENCH_BITRATE`             | `1000`                         | H.264 bitrate in kbps.                                                         |
| `BENCH_RTP_PORT`            | `7100`                         | UDP-RTP data port (RTCP lands on port+1).                                      |
| `BENCH_WVS_PORT`            | `8888`                         | `web_video_server` HTTP port.                                                  |
| `BENCH_RTSP_URL`            | `rtsp://127.0.0.1:8554/stream` | RTSP stream URL.                                                               |
| `BENCH_WEBRTC_PORT`         | `8443`                         | WebRTC signaling port.                                                         |
| `BENCH_SKIP_GOVERNOR_CHECK` | (unset)                        | Suppress the runner's CPU-governor warning.                                    |

`run_all.sh` and `run_scenario.sh` also accept `--encoder=va|nv|mpp` as
a CLI flag (equivalent to setting `BENCH_ENCODER`).

## Scenario matrix

Receivers per group: cam-ros = `ros_recv`; cam-stream / ros-stream =
stream receiver; cam-both = both.

### cam-ros — Camera → ROS topic only

| ID                        | Source                                          | ROS topic |
| ------------------------- | ----------------------------------------------- | --------- |
| cam-ros-rcs-yuv           | `ros_camera_server` V4L2 YUYV                   | rgb8      |
| cam-ros-rcs-mjpeg         | `ros_camera_server` V4L2 MJPEG passthrough      | jpeg      |
| cam-ros-rcs-mjpeg-decoded | `ros_camera_server` V4L2 MJPEG → jpegdec → rgb8 | rgb8      |
| cam-ros-usb_cam-yuv       | `usb_cam` YUYV → rgb8                           | rgb8      |
| cam-ros-usb_cam-mjpeg     | `usb_cam` MJPEG → rgb8 (decoded in driver)      | rgb8      |
| cam-ros-gscam-yuv         | `gscam` YUY2 → rgb8                             | rgb8      |
| cam-ros-gscam-mjpeg       | `gscam` jpegdec → rgb8                          | rgb8      |

`cam-ros-rcs-mjpeg` is the only arm where the ROS topic carries
`CompressedImage` instead of raw `Image`. Use
`cam-ros-rcs-mjpeg-decoded` for an apples-to-apples comparison with
`cam-ros-usb_cam-mjpeg` / `cam-ros-gscam-mjpeg`, where the JPEG decode
happens inside the publisher.

### cam-stream — Camera → encoded stream

| ID                                            | Source / Stack                         | Stream        |
| --------------------------------------------- | -------------------------------------- | ------------- |
| cam-stream-floor                              | gst-launch v4l2src (UDP-RTP floor)     | UDP-RTP       |
| cam-stream-rcs-yuv-rtp                        | `ros_camera_server` V4L2 yuv           | UDP-RTP H.264 |
| cam-stream-rcs-yuv-webrtc                     | `ros_camera_server` V4L2 yuv           | WebRTC        |
| cam-stream-usb_cam+gst_bridge-yuv-rtp         | `usb_cam` yuv + `ros-gst-bridge`       | UDP-RTP H.264 |
| cam-stream-gscam+gst_bridge-yuv-rtp           | `gscam` yuv + `ros-gst-bridge`         | UDP-RTP H.264 |
| cam-stream-usb_cam+web_video_server-yuv-mjpeg | `usb_cam` yuv + `web_video_server`     | HTTP MJPEG    |
| cam-stream-usb_cam+rtsp_fkie-yuv-h264         | `usb_cam` yuv + `rtsp_image_transport` | RTSP H.264    |

`cam-stream-usb_cam+gst_bridge-yuv-rtp` / `cam-stream-gscam+gst_bridge-yuv-rtp`
use the same launch files as their cam-both counterparts (the bridge's
`rosimagesrc` requires the camera driver to publish a ROS topic), but
skip the ROS receiver so only the encoded stream is measured.

### ros-stream — ROS image input → encoded stream

| ID                                    | Producer / Stack                       | Stream        |
| ------------------------------------- | -------------------------------------- | ------------- |
| ros-stream-rcs-raw-rtp                | `ros_pub` raw / `ros_camera_server`    | UDP-RTP H.264 |
| ros-stream-rcs-jpeg-rtp               | `ros_pub` jpeg / `ros_camera_server`   | UDP-RTP H.264 |
| ros-stream-rcs-raw-webrtc             | `ros_pub` raw / `ros_camera_server`    | WebRTC        |
| ros-stream-rcs-jpeg-webrtc            | `ros_pub` jpeg / `ros_camera_server`   | WebRTC        |
| ros-stream-gst_bridge-raw-rtp         | `ros_pub` raw / `ros-gst-bridge`       | UDP-RTP H.264 |
| ros-stream-web_video_server-raw-mjpeg | `ros_pub` raw / `web_video_server`     | HTTP MJPEG    |
| ros-stream-rtsp_fkie-raw-h264         | `ros_pub` raw / `rtsp_image_transport` | RTSP H.264    |

### cam-both — Camera → both stream + ROS topic

| ID                                    | Source / Stack                                  | Stream        | ROS topic |
| ------------------------------------- | ----------------------------------------------- | ------------- | --------- |
| cam-both-rcs-yuv-rtp                  | `ros_camera_server` V4L2 yuv                    | UDP-RTP H.264 | rgb8      |
| cam-both-rcs-mjpeg-rtp                | `ros_camera_server` V4L2 mjpeg passthrough      | UDP-RTP H.264 | jpeg      |
| cam-both-rcs-mjpeg-decoded-rtp        | `ros_camera_server` V4L2 mjpeg → jpegdec → rgb8 | UDP-RTP H.264 | rgb8      |
| cam-both-rcs-yuv-webrtc               | `ros_camera_server` V4L2 yuv                    | WebRTC        | rgb8      |
| cam-both-usb_cam+gst_bridge-yuv-rtp   | `usb_cam` yuv + `ros-gst-bridge`                | UDP-RTP H.264 | rgb8      |
| cam-both-gscam+gst_bridge-yuv-rtp     | `gscam` yuv + `ros-gst-bridge`                  | UDP-RTP H.264 | rgb8      |
| cam-both-usb_cam+gst_bridge-mjpeg-rtp | `usb_cam` mjpeg + `ros-gst-bridge`              | UDP-RTP H.264 | rgb8      |
| cam-both-gscam+gst_bridge-mjpeg-rtp   | `gscam` mjpeg + `ros-gst-bridge`                | UDP-RTP H.264 | rgb8      |
| cam-both-usb_cam+rtsp_fkie-yuv-h264   | `usb_cam` yuv + `rtsp_image_transport`          | RTSP H.264    | rgb8      |

Notes on stack-specific behaviour:

- **`*-usb_cam+gst_bridge-mjpeg-rtp` / `*-gscam+gst_bridge-mjpeg-rtp`**:
  `usb_cam` and `gscam` cannot publish `CompressedImage`; they decode
  MJPEG inside the driver and publish rgb8. The camera path includes
  the MJPEG decode but the ROS topic is raw.
- **`ros-stream-rcs-jpeg-rtp` / `-webrtc`**: `ros_camera_server`'s
  pipeline accepts `CompressedImage` via the `ros2` input + jpeg
  format. No bridge counterpart exists because `gst_bridge` lacks
  `roscompressedimagesrc`.
- **`*-web_video_server-*` + `*-rtsp_fkie-*`**: `web_video_server` and
  `rtsp_image_transport` consume ROS Image topics (not Compressed),
  so their ros-stream entries are raw-input only. For `cam-both` the
  same launch file publishes the topic and serves the stream
  simultaneously; the ROS topic flows past the encoder in parallel
  rather than through it.

### Scenario ID convention

`<group>-<stack>[+<bridge_stack>]-[<input>]-[<output>]`

- `+` joins composite stacks (e.g. `usb_cam+gst_bridge`,
  `usb_cam+rtsp_fkie`).
- Input format (`yuv`, `mjpeg`, `raw`, `jpeg`) is included whenever
  the stack accepts a choice.
- Output transport (`rtp`, `webrtc`, `mjpeg`, `h264`) is included for
  every scenario that emits a stream.
- `cam-ros-*` scenarios omit the output slot — the ROS topic is the
  output by definition.

## Full results

See [RESULTS.md](RESULTS.md) for the current notebook numbers.
`scripts/bench_summary.py` rolls per-run `summary.json` files into the
markdown tables there, one row per `(scenario, receiver)`: p50_med /
p50_min / p50_max / p95_med plus per-stack total CPU (cpu_mean /
cpu_p95 from `cpu_stack._total`). Re-run after a new matrix run to
refresh:

```bash
python3 ros_camera_server_benchmarks/scripts/bench_summary.py \
  --results-root ros_camera_server_benchmarks/results
```
