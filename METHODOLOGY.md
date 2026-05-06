# Methodology

How the harness measures latency, how each stack is exercised, the
fairness controls that pin cross-stack comparisons, and the cost
layers the headline numbers decompose into.

## Marker

Every produced frame carries a 64-bit `CLOCK_MONOTONIC` timestamp,
a 32-bit frame counter, a 16-bit width hint, and a CRC, painted as a
grid of 16x16 px black/white cells in the top rows of the Y plane. The
16 px cell aligns with both H.264 macroblocks and libjpeg's MCU
processing bands so the marker survives x264, VA-API, libav H.264, and
libjpeg q=70/85 roundtrips. The contract is pinned by
`test_marker_roundtrip` (`test/test_marker_roundtrip.cpp`,
specifically `MarkerRoundtrip.JpegRecvTimestampOrdering` for the
receiver-side timestamp ordering rule).

The U and V planes are forced to neutral (128) under the marker strip
so RGB receivers that approximate luma as `(R+G+B)/3` can still
recover bits — `marker.cpp` `clear_uv_strip_i420()`.

## What "latency" includes

**Producer side.** The marker `ts_ns` is sampled at the top of the
producer loop, *before* colour conversion + write (`frame_gen`) or
JPEG encode + publish (`ros_pub`). The producer-side conversion cost
varies with format (libjpeg encode is materially heavier than
`i420_to_yuyv`), so cross-format comparisons would otherwise see the
harness's own work as if it were SUT latency. To remove that bias, the
producer measures every per-frame conversion with `now_ns_monotonic`
and writes `encode_count` / `encode_total_ns` to its stats sidecar
(`frame_gen_stats.csv` or `ros_pub_stats.csv`). `bench_aggregate.py`
subtracts `encode_total_ns / encode_count` from each per-frame latency
before computing percentiles; the unadjusted numbers are preserved as
`p50_ms_raw` etc. for sanity checking. Receiver-side decode time is
intentionally left in — it represents real display-prep work a
consumer has to do. The floor (`cam-stream-floor`) still measures the
residual harness baseline after this subtraction.

**Receiver side.** `recv_ts` is sampled after the frame is decoded to
a display-ready I420 / rgb8 buffer, *before* the marker bit-extraction
runs. The reported number is therefore "time from producer to a frame
the consumer could render", which is what most teleop / streaming
stacks actually care about. Marker decoding is excluded as harness
overhead, not consumer work.

All receivers (`gst_recv`, `webrtc_recv`, `ros_recv`) honour this
contract. For the raw-Image ROS path the message arrives in a
display-ready encoding (`rgb8` / `mono8`), so `ros_recv` stamps `recv`
at message arrival and runs `rgb_to_y_plane` only to recover the
marker.

## Scenario groups

| Group          | Measures                          | Receivers              |
|----------------|-----------------------------------|------------------------|
| **cam-ros**    | Camera → ROS topic only           | `ros_recv`             |
| **cam-stream** | Camera → encoded stream (no ROS)  | stream receiver only   |
| **ros-stream** | ROS image input → encoded stream  | stream receiver only   |
| **cam-both**   | Camera → both stream + ROS topic  | stream + ROS receiver  |

This split lets you separate concerns. `cam-stream` vs `cam-both`
isolates whether teeing off a ROS topic affects stream latency.
`cam-ros` isolates producer→ROS without encoder noise. `ros-stream`
isolates ROS→encoder cost.

## Receivers per scenario

Three receiver CSVs may appear in `results/<id>/`, one per concern:

- `stream.csv` — encoded-stream receiver (cam-stream, ros-stream,
  cam-both).
- `ros_raw.csv` — raw ROS topic receiver (cam-ros, cam-both).
- `ros_jpeg.csv` — compressed ROS topic receiver (cam-ros, cam-both).

`bench_aggregate.py` drops empty receivers from `report.csv`. If a
scenario's required receivers (per group) all came back empty,
`bench_aggregate.py` exits non-zero and `run_scenario.sh` fails the
run instead of silently producing nothing.

## Stream framing

Every H.264 stack in the matrix emits RTP over UDP — `ros_camera_server`,
the `start_floor_pipeline` reference, and the `gst_bridge`-based bridges
all use `rtph264pay ! udpsink` with no jitter buffer and no retransmit
window. WebRTC negotiates its own SRTP transport via `webrtcbin`. The
non-H.264 stacks emit what they support natively:

- **`web_video_server`**: HTTP MJPEG (multipart). H.264 path excluded —
  broken upstream (empty `avcC` in the served fMP4).
- **`rtsp_image_transport`**: standard RTSP H.264 via
  `image_transport/rtsp_pub`. Standard RTSP clients (`ffprobe`,
  `gst-launch rtspsrc`) cannot complete SETUP/PLAY against
  `rtsp_pub`, so the bench consumes via FKIE's own `rtsp_sub` plugin
  (also in the `rtsp_image_transport` package) using a second
  `image_transport republish rtsp raw`. `ros_recv` subscribes to the
  republished raw topic and writes to `stream.csv`. The chain
  therefore adds one extra ROS topic hop above what an external RTSP
  client would see, and `rtsp_sub` does not insert a 100 ms jitter
  buffer a generic `rtspsrc latency=100` receiver would; the reported
  latency reflects the FKIE-stack end-to-end path users actually run.
  `rtsp_pub` also gates encoding on having a raw-transport subscriber,
  so `run_scenario.sh` keeps a `ros2 topic hz /out` alive for the
  duration of the scenario.

`gst_recv --mode` switches between UDP-RTP and HTTP-MJPEG transports;
WebRTC has its own receiver (`webrtc_recv`) because it needs websocket
signaling; RTSP arms have no `gst_recv` mode and use
`image_transport republish rtsp raw` + `ros_recv` instead.

No jitter buffer anywhere in the H.264 receive paths: the latency floor
is bare encode + RTP payload + decode, with no application-level buffer
hiding pipeline costs. RTCP lands on `RTP_PORT + 1` (default 7101) but
no peer is bound there, so kernel ICMP-unreachable packets are produced
and discarded — harmless for localhost benches.

## Pipeline commands

The exact pipelines under test, for reference. The encoder line is
the shared `BENCH_ENC_GST` fragment built by `env_fairness.sh` from
the selected family (VA_LP shown below; the NV variant is
`nvh264enc preset=4 bitrate=1000 gop-size=10 strict-gop=true rc-mode=3
min-force-key-unit-interval=1000000000 spatial-aq=true zerolatency=true
aud=true`; the MPP variant is `mpph264enc bps=1000000 gop=10
header-mode=1 profile=66 rc-mode=0 max-pending=1
min-force-key-unit-interval=1000000000 zero-copy-pkt=true` — `bps`
takes bits-per-second on the Rockchip plugin while VA/NV `bitrate`
take kbps, so `BENCH_ENC_GST` multiplies `BITRATE` by 1000 for the
MPP arm to keep the operating point identical). Encoder knobs mirror
`ros_camera_server`'s H.264 codec helper
(`ros_camera_server/src/codecs/h264.hpp`) so cross-stack deltas
measure pipeline overhead, not codec settings. The floor + bridge
gst-launch invocations live in shell helpers in `scripts/run_scenario.sh`
(`start_floor_pipeline` / `start_bridge_pipeline`) to keep the encoder
fragment expansion in bash rather than fighting `ros2 launch` yaml's
tokenization rules for `executable.cmd`.

**Floor (`cam-stream-floor`)** — `start_floor_pipeline`:

```text
v4l2src device=/dev/video40 io-mode=mmap do-timestamp=true
  ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1
  ! videoconvert ! video/x-raw,format=NV12
  ! vah264lpenc bitrate=1000 key-int-max=10 aud=true
                min-force-key-unit-interval=1000000000 num-slices=1
                cabac=true dct8x8=true rate-control=vbr ref-frames=1
  ! rtph264pay aggregate-mode=0 mtu=1300 config-interval=-1
  ! udpsink host=127.0.0.1 port=7100 sync=false async=false
```

**Bridges (`cam-stream-{usb_cam,gscam}+gst_bridge-yuv-rtp`,
`ros-stream-gst_bridge-raw-rtp`, `cam-both-{usb_cam,gscam}+gst_bridge-*-rtp`)** —
`start_bridge_pipeline /bench/<topic>`:

```text
rosimagesrc ros-topic=/bench/<topic>
  ! video/x-raw,framerate=30/1
  ! videoconvert ! video/x-raw,format=NV12
  ! vah264lpenc ...same knobs as above...
  ! rtph264pay aggregate-mode=0 mtu=1300 config-interval=-1
  ! udpsink host=127.0.0.1 port=7100 sync=false async=false
```

The camera-driver scenarios prefix this with `usb_cam_node_exe` or
`gscam_node` reading from `/dev/video1{0,1}` and publishing onto a ROS
topic that `rosimagesrc` then subscribes to (the camera launches via
`usb_cam_only_*.launch.yaml` / `gscam2_only_*.launch.yaml`, the bridge
follows via `start_bridge_pipeline`). `ros-stream-gst_bridge-raw-rtp`
has no driver — `ros_pub` publishes synthetic frames directly.

**Stream receivers** — `gst_recv` constructs an appsink pipeline keyed
on `--mode`:

```text
# udp-rtp (every H.264 stack: floor, ros_camera_server RTP, bridges):
udpsrc port=N caps="application/x-rtp,...,encoding-name=H264,payload=96"
  ! rtph264depay ! h264parse ! decodebin ! videoconvert ! I420 ! appsink

# http-mjpeg (web_video_server MJPEG):
souphttpsrc location=$URI is-live=true ! multipartdemux
  ! image/jpeg ! jpegparse ! decodebin ! videoconvert ! I420 ! appsink
```

The RTSP arms use FKIE's `image_transport republish rtsp raw` instead
of a `gst_recv` mode (the `rtsp_sub` plugin is the only working
consumer for `rtsp_pub`); `ros_recv` subscribes to the republished raw
topic and writes to `stream.csv`.

`webrtc_recv` runs a libsoup websocket signaling client against
`ws://host:port/{camera_id}/{output_index}` (see
`ros_camera_server/src/webrtc/webrtc_peer.cpp` for the wire format)
and plumbs the negotiated H.264 RTP stream through
`rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! I420 ! appsink`.

## Fairness

- **Same operating point** across all stacks: 1280x720 @ 30 fps,
  1 Mbps bitrate, RTP port 7100. Override via `BENCH_*` env vars (see
  [REPRODUCING.md](REPRODUCING.md)).
- **Encoder family pinned at run time.** `BENCH_ENCODER=va` selects
  VA-API `vah264lpenc`, `BENCH_ENCODER=nv` selects NVIDIA `nvh264enc`,
  `BENCH_ENCODER=mpp` selects Rockchip `mpph264enc`. If unset and only
  one is installed, `env_fairness.sh` auto-picks; if more than one is
  installed it prompts on stderr (set `BENCH_ENCODER` for scripted
  runs). The selected family is exported as `BENCH_ENC_GST`
  (gst-launch encoder fragment for floor + bridge helpers) and
  `BENCH_RCS_ENC` (`VA_LP` / `NV` / `MPP` string consumed by
  `ros_camera_server`'s `codecFromString` after `envsubst` renders the
  `encoder:` field in each rcs config). Competing encoders/decoders
  (the *other* families, plus `qsv*`, `v4l2*`, `x264enc`, `avdec_h264`)
  are demoted via `GST_PLUGIN_FEATURE_RANK` so the picked encoder wins
  deterministically and receivers' `decodebin` picks the matching
  hardware decoder. The same demotion list also covers hardware JPEG
  decoders (`vajpegdec` / `nvjpegdec` / `qsvjpegdec` / `mppjpegdec`)
  so the `http-mjpeg` receiver's `decodebin` picks the JPEG decoder
  from the selected family when one is installed, falling back to
  libjpeg-turbo `jpegdec` otherwise.
- **Encoder knobs aligned across H.264 stacks.** `key-int-max` /
  `gop-size` / `gop`, `rate-control` / `rc-mode`, `aud`, `cabac`,
  `dct8x8`, `num-slices`, `ref-frames`, `preset`, `spatial-aq`,
  `zerolatency`, `header-mode`, `profile`, `max-pending`,
  `zero-copy-pkt`, `min-force-key-unit-interval` on bridges + floor
  mirror what `ros_camera_server`'s H.264 codec helper sets on its own
  encoder (`ros_camera_server/src/codecs/h264.hpp`, VA_LP block lines
  135–143, NV block 163–171, MPP block 184–193), so cross-stack deltas
  measure pipeline overhead, not codec settings. The MPP fragment
  passes `bps` in bits-per-second (the Rockchip plugin's unit) computed
  from `BITRATE` × 1000, so the MPP arm runs at the same operating
  point as the kbps-based VA/NV arms.
- **Wire framing is uniform: RTP over UDP, no jitter buffer, no
  retransmit window.** `ros_camera_server`, the floor, and every
  `gst_bridge`-based bridge use the same `rtph264pay ! udpsink` →
  `udpsrc ! rtph264depay` chain. Latency above the floor is therefore
  pipeline overhead inside each stack, not framing cost.
- **WebRTC and FKIE-RTSP negotiate / configure their own encoders
  internally** and are not pinned the same way. They are compared as
  out-of-the-box stacks rather than identical pipelines — that is the
  intended user experience for both.
- **Receiver QoS hardcoded to `BEST_EFFORT, KEEP_LAST(10)`** because
  `usb_cam` publishes with sensor_data QoS (best-effort only). A
  reliable subscriber against `ros_camera_server` (reliable publisher)
  might shave a small amount of jitter, but `usb_cam` would receive
  nothing — best-effort is the lowest common denominator that keeps
  the comparison apples-to-apples.
- **`ros_pub` (the `ros-stream` group's image producer) publishes
  with `BEST_EFFORT, KEEP_LAST(10), VOLATILE`**, matching `usb_cam`'s
  sensor_data default. Each downstream stack (RCS, gst_bridge,
  web_video_server, rtsp_pub) sees the same producer-side QoS so
  internal queueing is compared on equal footing.
- **2 s warmup dropped, up to 750 frames recorded per run, 5 repeats
  per scenario.** Defaults in `run_scenario.sh` are
  `BENCH_WARMUP_FRAMES=60`, `BENCH_DURATION=15`, `BENCH_MAX_FRAMES=750`,
  `BENCH_REPEATS=5`.
- **CPU governor expected at `performance`.** Default Ubuntu governors
  (`schedutil`, `powersave`) ramp cores up/down per-tick, adding wake-up
  jitter to receiver dispatch and noise to `cpu.csv`. The runner warns
  when the governor is not `performance`; the fix is
  `sudo cpupower frequency-set -g performance` (see
  [REPRODUCING.md](REPRODUCING.md)).

## Frame content

By default `frame_gen` emits flat gray + a 1-px walking bar plus the
marker strip. This is low-entropy: H.264 CBR at 1 Mbps will be far
above the rate needed, so encoder work and queueing latency are
deflated relative to real camera content.

For results worth publishing, run with a base video overlaid behind
the marker strip:

```bash
BENCH_VIDEO=/path/to/highmotion.mp4 \
  BENCH_VIDEO_CYCLE_S=5 \
  BENCH_REPEATS=5 \
  ros2 run ros_camera_server_benchmarks run_all.sh
```

`frame_gen` and `ros_pub` use a streaming GStreamer loader (`filesrc !
decodebin ! videoconvert ! videoscale ! I420 @ WxH`) that decodes the
video on a worker thread while the producer's main loop emits frames.
The producer blocks on the *first* decoded frame (~one decode latency,
typically <300 ms with VA-API), then ticks at `BENCH_FPS`, sourcing
each frame from the streaming buffer. Once `BENCH_VIDEO_CYCLE_S * fps`
frames are decoded the loader stops and the producer loops the cycle
(default 5 s = 150 frames at 30 fps, ~200 MB at 720p). The marker
strip is overlaid on top of each frame's Y plane, so the marker is
unaffected by the underlying content.

If decode falls behind the producer's tick rate (a slow-decode host,
or a codec with no hardware support), the loader returns the most
recently decoded frame and increments `base_video_catch_up_reuses` in
`frame_gen_stats.csv`. Watch that counter — non-zero means the host
can't sustain real-time decode and the recorded video content is
partially repeated.


## Caveats

- **gscam is run with `sync_sink: false` (non-default).** Upstream's
  default `sync_sink: true` makes the appsink hold each buffer until
  its presentation time matches the pipeline clock, adding up to one
  frame period (~33 ms at 30 fps) of clock alignment that `usb_cam`
  (no equivalent flag, publishes as soon as a V4L2 frame is captured)
  does not pay. We override the default so cross-stack deltas reflect
  pipeline overhead rather than gscam's clocking choice.
- **`usb_cam` runs with a v4l2loopback `TIMEPERFRAME` `LD_PRELOAD`
  shim** because v4l2loopback does not advertise the
  `V4L2_CAP_TIMEPERFRAME` capability that `usb_cam` requires; without
  the shim `usb_cam_node_exe` aborts. `gscam` does not probe that
  capability, so it does not need the shim. The shim affects only
  capability-flag visibility; it does not move frame data or change
  scheduling, so the asymmetry does not bias measured latency.
- **`web_video_server` H.264 path is excluded from the matrix.**
  Upstream emits an empty `avcC` in the served fragmented MP4's moov
  box, so no standard H.264 consumer (`ffmpeg`, GStreamer's
  `qtdemux` / `decodebin`) can bootstrap a decoder. MJPEG is
  `web_video_server`'s primary streamer and works end-to-end.
- **`rtsp_image_transport`: FKIE's `rtsp_pub` interoperates only with
  FKIE's `rtsp_sub`.** The `image_transport/rtsp_pub` plugin advertises
  a valid SDP via DESCRIBE, but standard RTSP clients (`ffprobe`,
  `ffplay`, GStreamer's `rtspsrc`) cannot complete the SETUP/PLAY
  handshake — the request hangs silently or returns 404. The plugin's
  `rtsp_sub` counterpart (in the same package) does interoperate, so
  the bench consumes RTSP via a second `image_transport republish rtsp
  raw`, which decodes the RTSP stream and republishes raw Image.
  Side-effects: (1) the receive chain adds one extra ROS topic hop;
  (2) `rtsp_sub` does not insert a 100 ms jitter buffer the way the
  old `rtspsrc latency=100 drop-on-latency=true` receiver did, so
  reported latency is correspondingly lower and reflects the
  FKIE-stack end-to-end path users running this stack actually
  measure. `rtsp_pub` also gates encoding on having a ROS subscriber
  on the raw output topic (independent of the RTSP transport);
  `run_scenario.sh` keeps a `ros2 topic hz /out` alive for the
  lifetime of every RTSP scenario to satisfy this. Source
  confirmation: `rtsp_image_transport/src/publisher_plugin.cpp`
  (`getNumSubscribers()` ↔ `onGraphChange()`). The keepalive
  subscribers exchange empty frame-notification messages, not video
  data — they are ROS graph bookkeeping to keep the publisher's
  encoder alive, not part of the measured transport latency.
- **`frame_gen` on underrun resets the deadline to `now`** rather
  than failing the run, so a very slow stack would silently lower its
  effective fps and the survivors would still produce a CSV. Watch
  `frame_gen_stats.csv` for non-zero `underruns` /
  `base_video_catch_up_reuses` counters.
- **Synthetic frames (the default flat-gray + walking bar)** are
  low-entropy; H.264 CBR at 1 Mbps is far above what's needed and
  encoder/queue latency is deflated relative to real camera content.
  Numbers worth publishing should be captured with
  `BENCH_VIDEO=/path/to/real_video.mp4` set — see "Frame content".

## Stacks intentionally excluded

- **`camera_ros` (libcamera)** — libcamera's pipeline handlers do not
  match `v4l2loopback` devices, so the synthetic-marker methodology
  used here cannot drive it without either a USB UVC gadget or a
  patched v4l2loopback. Either is out of scope.
- **`gst_bridge + webrtcbin`** — `gst_bridge` does not expose a webrtc
  element. A combined arm would require patching the bridge upstream.
- **`go2rtc`, `mediamtx`** — standalone RTSP/WebRTC restreamers, not
  ROS packages. Comparing them would step outside the ROS ecosystem
  this harness is scoped to.
- **`gst_bridge` does not support compressed images as input, so
  there is no jpeg-input bridge counterpart for
  `ros-stream-rcs-jpeg-rtp` / `ros-stream-rcs-jpeg-webrtc`. Those
  scenarios exist only on the `ros_camera_server` side.
