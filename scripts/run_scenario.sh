#!/usr/bin/env bash
# Run a single benchmark scenario. Usage: run_scenario.sh [--encoder=va|nv|mpp] <id>
#
# Scenarios are grouped by what they measure (see README + METHODOLOGY.md):
#   cam-ros-*     Camera   -> ROS image    (producer-only latency, ros_recv only)
#   cam-stream-*  Camera   -> H.264 stream (no ROS topic in path; stream receiver only)
#   ros-stream-*  ROS img  -> H.264 stream (ros_pub producer; stream receiver only)
#   cam-both-*    Camera   -> both stream + ROS image (two receivers per scenario)
#
# Produces results/<id>/{stream.csv,ros_raw.csv,ros_jpeg.csv,
# frame_gen_stats.csv,summary.json,console.log}. Receivers that aren't
# applicable to a scenario simply don't get started; bench_aggregate.py
# enforces per-scenario expected-receiver coverage.
#
# Encoder: --encoder=va|nv|mpp (or BENCH_ENCODER env) selects vah264lpenc /
# nvh264enc / mpph264enc. If unset and more than one is installed,
# env_fairness.sh prompts interactively.
set -euo pipefail

SCENARIO=""
for arg in "$@"; do
  case "${arg}" in
    --encoder=va|--encoder=nv|--encoder=mpp) export BENCH_ENCODER="${arg#--encoder=}" ;;
    --encoder=*) echo "Usage: $0 [--encoder=va|nv|mpp] <scenario_id>" >&2; exit 2 ;;
    *) SCENARIO="${arg}" ;;
  esac
done
if [[ -z "${SCENARIO}" ]]; then
  echo "Usage: $0 [--encoder=va|nv|mpp] <scenario_id>" >&2
  echo "  groups: cam-ros-* cam-stream-* ros-stream-* cam-both-*" >&2
  exit 2
fi

HERE="$(cd "$(dirname "$0")" && pwd)"

# `ros2 pkg prefix` prints a generic "Package not found" with no package
# name on failure. Catch that here so the user gets an actionable message
# instead of a cryptic line repeated for every scenario.
if ! PKG_PREFIX="$(ros2 pkg prefix ros_camera_server_benchmarks 2>/dev/null)"; then
  echo "[run_scenario] ros_camera_server_benchmarks not visible to ros2." >&2
  echo "[run_scenario]   did you forget to 'source install/setup.bash'?" >&2
  exit 2
fi
PKG_SHARE="${PKG_PREFIX}/share/ros_camera_server_benchmarks"

# Each scenario routes through a different stack-under-test. If a required
# package isn't visible to ros2, fail with a clear message and a non-zero
# exit code so run_all.sh can record a skip instead of crashing inside
# `ros2 launch`. Exit code 3 is reserved for "missing dependency".
require_pkg() {
  local pkg="$1"
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[run_scenario ${SCENARIO}] missing package: ${pkg}" >&2
    echo "[run_scenario ${SCENARIO}]   see REPRODUCING.md 'Stacks-under-test'." >&2
    exit 3
  fi
}

case "${SCENARIO}" in
  cam-stream-floor)                                                                       : ;; # gst-launch only
  ros-stream-gst_bridge-raw-rtp)                                                          require_pkg gst_bridge ;;
  cam-ros-rcs-*|cam-stream-rcs-*|ros-stream-rcs-*|cam-both-rcs-*)                         require_pkg ros_camera_server ;;
  cam-ros-usb_cam-*)                                                                      require_pkg usb_cam ;;
  cam-stream-usb_cam+gst_bridge-*|cam-both-usb_cam+gst_bridge-*)                          require_pkg usb_cam; require_pkg gst_bridge ;;
  cam-ros-gscam-*)                                                                        require_pkg gscam ;;
  cam-stream-gscam+gst_bridge-*|cam-both-gscam+gst_bridge-*)                              require_pkg gscam; require_pkg gst_bridge ;;
  cam-stream-usb_cam+web_video_server-*|ros-stream-web_video_server-*)                    require_pkg usb_cam; require_pkg web_video_server ;;
  cam-stream-usb_cam+rtsp_fkie-*|ros-stream-rtsp_fkie-*|cam-both-usb_cam+rtsp_fkie-*)     require_pkg usb_cam; require_pkg rtsp_image_transport ;;
  cam-stream-usb_cam+ffmpeg_transport-*|cam-both-usb_cam+ffmpeg_transport-*)              require_pkg usb_cam; require_pkg ffmpeg_image_transport ;;
  ros-stream-ffmpeg_transport-*)                                                          require_pkg ffmpeg_image_transport ;;
  *)                                                                                      echo "unknown scenario: ${SCENARIO}" >&2; exit 2 ;;
esac

# shellcheck disable=SC1091
source "${HERE}/helpers/check_cpu_governor.sh"

RESULTS_ROOT="${BENCH_RESULTS_DIR:-${PWD}/results}"
OUT="${RESULTS_ROOT}/${SCENARIO}"
mkdir -p "${OUT}"

DURATION="${BENCH_DURATION:-30}"
WARMUP="${BENCH_WARMUP_FRAMES:-60}"
MAX_FRAMES="${BENCH_MAX_FRAMES:-900}"
WIDTH="${BENCH_WIDTH:-1920}"
HEIGHT="${BENCH_HEIGHT:-1080}"
# Re-export the resolved resolution so start_rcs's envsubst pass substitutes
# ${BENCH_WIDTH}/${BENCH_HEIGHT} in the rcs configs (same mechanism as
# ${BENCH_RCS_ENC}). Keeps RCS at the harness operating point instead of a
# hardcoded one; a config/harness resolution mismatch silently corrupts the
# latency marker (downscale resamples the cell strip -> CRC fails).
export BENCH_WIDTH="${WIDTH}" BENCH_HEIGHT="${HEIGHT}"
FPS="${BENCH_FPS:-30}"
RTP_PORT="${BENCH_RTP_PORT:-7100}"
BITRATE="${BENCH_BITRATE:-1000}"
WVS_PORT="${BENCH_WVS_PORT:-8888}"
RTSP_URL="${BENCH_RTSP_URL:-rtsp://127.0.0.1:8554/stream}"
WEBRTC_PORT="${BENCH_WEBRTC_PORT:-8443}"
export BITRATE  # env_fairness.sh interpolates this into BENCH_ENC_GST.

# shellcheck disable=SC1091
source "${HERE}/helpers/env_fairness.sh"

PIDS=()
LOG="${OUT}/console.log"
: > "${LOG}"

# Kill the whole process group of each child. `ros2 launch` spawns
# camera_server_standalone / gst-launch-1.0 as grandchildren and ignores
# SIGTERM forwarded to the launch script — killing the group reaches them.
# `setsid` in start_bg ensures each background command is its own session
# leader, so its PID equals its PGID.
cleanup() {
  echo "[run_scenario] cleanup" >> "${LOG}"
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
  # Belt-and-suspenders: kill anything still bound to the RTP port or
  # producing into our v4l2loopback nodes, plus stack-specific daemons.
  # Patterns are anchored to executable paths or arg shapes specific to
  # the spawned processes so they don't match this script's own command
  # line (scenario IDs like `cam-stream-usb_cam+web_video_server-yuv-mjpeg`
  # would otherwise self-kill the dispatcher).
  pkill -KILL -f "camera_server_standalone --ros-args" 2>/dev/null || true
  pkill -KILL -f "ros_camera_server_benchmarks/(ros_pub|gst_recv|webrtc_recv|frame_gen|ros_recv)" 2>/dev/null || true
  pkill -KILL -f "gst-launch-1.0 .* udpsink" 2>/dev/null || true
  pkill -KILL -f "lib/web_video_server/web_video_server" 2>/dev/null || true
  pkill -KILL -f "rtsp_image_transport publish_rtsp_stream" 2>/dev/null || true
  pkill -KILL -f "image_transport/republish" 2>/dev/null || true
  pkill -KILL -f "ros2 topic (hz|echo)" 2>/dev/null || true
}
trap cleanup EXIT

start_bg() {
  echo "+ $*" >> "${LOG}"
  setsid "$@" >>"${LOG}" 2>&1 &
  PIDS+=("$!")
}

# =============================================================================
# Receiver / producer / stack helpers. CSV file names are fixed
# (stream.csv, ros_raw.csv, ros_jpeg.csv) so bench_aggregate.py can locate
# them without per-scenario wiring.
# =============================================================================

start_gst_recv() {  # mode uri
  start_bg ros2 run ros_camera_server_benchmarks gst_recv \
    --mode "$1" --uri "$2" --csv "${OUT}/stream.csv" \
    --warmup-frames "${WARMUP}" --max-frames "${MAX_FRAMES}"
}

start_webrtc_recv() {  # ws_uri
  start_bg ros2 run ros_camera_server_benchmarks webrtc_recv \
    --uri "$1" --csv "${OUT}/stream.csv" \
    --warmup-frames "${WARMUP}" --max-frames "${MAX_FRAMES}"
}

start_ros_recv_raw() {  # topic [csv-basename]
  local csv="${2:-ros_raw}"
  start_bg ros2 run ros_camera_server_benchmarks ros_recv --ros-args \
    -r __node:="ros_recv_${csv}" \
    -p topic:="$1" -p kind:=raw \
    -p csv:="${OUT}/${csv}.csv" \
    -p warmup:="${WARMUP}" -p max_frames:="${MAX_FRAMES}"
}

start_ros_recv_jpeg() {  # topic
  start_bg ros2 run ros_camera_server_benchmarks ros_recv --ros-args \
    -r __node:=ros_recv_jpeg \
    -p topic:="$1" -p kind:=compressed \
    -p csv:="${OUT}/ros_jpeg.csv" \
    -p warmup:="${WARMUP}" -p max_frames:="${MAX_FRAMES}"
}

# RTSP receivers use FKIE's own rtsp_sub plugin via image_transport republish:
# standard RTSP clients (ffprobe, gst-launch rtspsrc) cannot complete
# SETUP/PLAY against rtsp_pub. The republished raw Image topic feeds ros_recv
# writing to stream.csv so bench_aggregate.py treats it as the stream
# receiver. Adds ~1 ROS topic hop above what an external RTSP client would
# see — documented in METHODOLOGY.md "Stream framing".
#
# rtsp_pub also gates encoding on a raw-transport subscriber; without it the
# encoder stops even when rtsp_sub is connected. start_rtsp_keepalive holds
# both transports open with cheap subscribers.
start_rtsp_keepalive() {  # base url_log
  : > "$2"
  start_bg ros2 topic hz "$1"
  start_bg bash -c "ros2 topic echo --qos-durability transient_local \
                                    --qos-reliability reliable \
                                    --qos-history keep_last --qos-depth 1 \
                                    '$1/rtsp' std_msgs/msg/String \
                    > '$2' 2>&1"
}

start_rtsp_camera_proxy_recv() {  # in_base out_topic
  start_bg ros2 launch ros_camera_server_benchmarks rtsp_camera_proxy_recv.launch.yaml \
    "in_base:=$1" "out_topic:=$2"
}

# Combined RTSP "stream receiver": keepalive + republish + ros_recv writing
# to stream.csv. Used by cam-stream-usb_cam+rtsp_fkie-yuv-h264,
# ros-stream-rtsp_fkie-raw-h264, cam-both-usb_cam+rtsp_fkie-yuv-h264.
start_rtsp_stream_recv() {
  start_rtsp_keepalive /out "${OUT}/rtsp_url.log"
  sleep 2
  start_rtsp_camera_proxy_recv /out /bench/rtsp_recv_image
  sleep 1
  start_bg ros2 run ros_camera_server_benchmarks ros_recv --ros-args \
    -r __node:=ros_recv_stream \
    -p topic:=/bench/rtsp_recv_image -p kind:=raw \
    -p csv:="${OUT}/stream.csv" \
    -p warmup:="${WARMUP}" -p max_frames:="${MAX_FRAMES}"
}

# ffmpeg_image_transport "stream receiver": `republish ffmpeg raw` decoder +
# ros_recv writing to stream.csv. Used by cam-stream-usb_cam+ffmpeg_transport-yuv-h264,
# ros-stream-ffmpeg_transport-raw-h264, cam-both-usb_cam+ffmpeg_transport-yuv-h264.
# Simpler than start_rtsp_stream_recv because ffmpeg is pure ROS topics — no URL
# keepalive / discovery handshake. Adds the same +1 ROS-topic hop on the
# receive path as the rtsp_fkie arm; documented in METHODOLOGY.md.
#
# $1 is the encoder-side base topic the ffmpeg subscriber attaches to
# (`<base>/ffmpeg`). For cam-* arms usb_cam owns the image_transport
# publisher directly, so the base is the remapped /image_raw destination
# (`/bench/out_raw`). For ros-stream-ffmpeg_transport-raw-h264 the ros_pub
# producer publishes via image_transport on the configured topic
# (`/bench/image_raw`).
start_ffmpeg_stream_recv() {
  local in_base="${1:-/out}"
  start_bg ros2 launch ros_camera_server_benchmarks ffmpeg_recv.launch.yaml \
    "in_base:=${in_base}" "out_topic:=/bench/ffmpeg_recv_image"
  sleep 1
  start_bg ros2 run ros_camera_server_benchmarks ros_recv --ros-args \
    -r __node:=ros_recv_stream \
    -p topic:=/bench/ffmpeg_recv_image -p kind:=raw \
    -p csv:="${OUT}/stream.csv" \
    -p warmup:="${WARMUP}" -p max_frames:="${MAX_FRAMES}"
}

# rbfimagesink publishes compressed images at "<topic>/compressed", not at
# the base topic name (raw goes to base topic, jpeg gets the suffix appended
# by image_transport convention). ros_camera_server's mjpeg config uses
# topic=/bench/out_compressed, so the actual jpeg stream is at
# /bench/out_compressed/compressed.
RCS_JPEG_TOPIC="/bench/out_compressed/compressed"

start_frame_gen() {  # format device
  # Tail beyond DURATION so the producer survives post-launch sleeps + cleanup.
  # Without it, scenarios with longer setup (web_video_server, RTSP) end up
  # reading from a closed v4l2loopback node and usb_cam crashes with "Select
  # timeout".
  local frame_gen_duration=$((DURATION + 30))
  local args=(
    --device "$2" --width "${WIDTH}" --height "${HEIGHT}"
    --fps "${FPS}" --format "$1"
    --duration "${frame_gen_duration}"
    --underrun-csv "${OUT}/frame_gen_stats.csv"
  )
  if [[ -n "${BENCH_VIDEO:-}" ]]; then
    args+=(--video "${BENCH_VIDEO}")
    [[ -n "${BENCH_VIDEO_CYCLE_S:-}" ]] && args+=(--video-cycle-seconds "${BENCH_VIDEO_CYCLE_S}")
  fi
  start_bg ros2 run ros_camera_server_benchmarks frame_gen "${args[@]}"
}

start_ros_pub() {  # kind topic
  # BENCH_VIDEO support mirrors frame_gen so ros-stream (ros_pub-driven)
  # sees the same encoder workload as cam-stream/cam-both (frame_gen-driven).
  # Without this the encoder runs on flat-gray and ros-stream numbers are
  # encoder-deflated.
  local args=(--ros-args
    -p kind:="$1" -p topic:="$2"
    -p width:="${WIDTH}" -p height:="${HEIGHT}" -p fps:="${FPS}"
    -p encode_stats:="${OUT}/ros_pub_stats.csv")
  if [[ -n "${BENCH_VIDEO:-}" ]]; then
    args+=(-p video:="${BENCH_VIDEO}")
    [[ -n "${BENCH_VIDEO_CYCLE_S:-}" ]] && args+=(-p video_cycle_seconds:="${BENCH_VIDEO_CYCLE_S}")
  fi
  # ros_pub publishes raw via image_transport, so transport plugins (notably
  # ffmpeg_image_transport) load directly into the producer process when the
  # corresponding subtopic has a subscriber. ros-stream-ffmpeg sets the
  # plugin params here on ros_pub itself; with no producer-side republisher
  # in the chain, this matches how a real application would publish ffmpeg
  # frames via image_transport. Param namespace mirrors the rule used for
  # usb_cam (resolved-but-unremapped base topic, with leading `/` stripped
  # and `/` replaced by `.`); ros_pub does not remap anything, so it's just
  # the topic with leading slash removed.
  if [[ -n "${BENCH_ROS_PUB_FFMPEG:-}" ]]; then
    local prefix="${2#/}"; prefix="${prefix//\//.}"
    args+=(
      -p "${prefix}.ffmpeg.encoder:=${BENCH_FFMPEG_ENC}"
      -p "${prefix}.ffmpeg.bit_rate:=$((BITRATE * 1000))"
      -p "${prefix}.ffmpeg.gop_size:=30"
      -p "${prefix}.ffmpeg.max_b_frames:=0"
      -p "${prefix}.ffmpeg.encoder_av_options:=${BENCH_FFMPEG_AV_OPTS}"
    )
  fi
  start_bg ros2 run ros_camera_server_benchmarks ros_pub "${args[@]}"
}

# ros_camera_server launches with envsubst-rendered config so ${BENCH_RCS_ENC}
# (set by env_fairness.sh) lands as a literal "VA_LP" / "NV" string that
# ros_camera_server's codecFromString consumes. Source configs live in
# PKG_SHARE/config and stay unmodified between runs.
start_rcs() {  # config_filename
  local rendered="${OUT}/rcs_config.yaml"
  envsubst < "${PKG_SHARE}/config/$1" > "${rendered}"
  start_bg ros2 launch ros_camera_server_benchmarks rcs.launch.yaml \
    "config_path:=${rendered}"
}

# Floor pipeline: stack-less reference path emitting H.264 over UDP-RTP.
# Encoder fragment is BENCH_ENC_GST (env_fairness.sh constructs it with
# bitrate baked in). No jitter buffer, no retransmit window — the latency
# floor is bare encode + RTP payload + decode.
start_floor_pipeline() {
  start_bg gst-launch-1.0 -q \
    v4l2src device=/dev/video40 io-mode=mmap do-timestamp=true ! \
    "video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
    videoconvert ! video/x-raw,format=NV12 ! \
    ${BENCH_ENC_GST} ! \
    rtph264pay aggregate-mode=0 mtu=1300 config-interval=-1 ! \
    udpsink host=127.0.0.1 port=${RTP_PORT} sync=false async=false
}

# Bridge pipeline: rosimagesrc -> encoder -> UDP-RTP, taking the same encoder
# fragment as the floor. Encoder knobs match ros_camera_server's so
# cross-stack deltas measure pipeline overhead, not codec settings. The
# framerate cap after rosimagesrc is required: rosimagesrc reports
# framerate=0/1 (variable) until it has seen enough messages to derive a
# rate, and vah264enc / nvh264enc with rate-control=vbr stalls silently
# until the framerate is fixed.
start_bridge_pipeline() {  # topic
  local topic="$1"
  start_bg gst-launch-1.0 -q \
    rosimagesrc name=ros_src ros-topic="${topic}" ! \
    "video/x-raw,framerate=${FPS}/1" ! \
    videoconvert ! video/x-raw,format=NV12 ! \
    ${BENCH_ENC_GST} ! \
    rtph264pay aggregate-mode=0 mtu=1300 config-interval=-1 ! \
    udpsink host=127.0.0.1 port=${RTP_PORT} sync=false async=false
}

# WebRTC ws URIs: with a single output config the index is 0, with both_webrtc
# configs (ros2 + webrtc) the webrtc output is index 1.
WS_BENCH0="ws://127.0.0.1:${WEBRTC_PORT}/bench/0"
WS_BENCH1="ws://127.0.0.1:${WEBRTC_PORT}/bench/1"
WVS_MJPEG_URL="http://127.0.0.1:${WVS_PORT}/stream?topic=/bench/out_raw&type=mjpeg"
WVS_MJPEG_RAW_URL="http://127.0.0.1:${WVS_PORT}/stream?topic=/bench/image_raw&type=mjpeg"

# =============================================================================
# Scenario table and dispatcher.
#
# Each row: ID|PRE_RECVS|PRODUCER|STACK|POST_RECVS
#   PRE_RECVS  semicolon-separated receiver invocations (started before producer)
#   PRODUCER   one of: framegen-yuyv | framegen-mjpeg | rospub-raw | rospub-jpeg
#   STACK      shell command (eval'd; access to ${RTP_PORT} etc.)
#   POST_RECVS semicolon-separated receiver invocations started AFTER the stack,
#              with `sleep N;` items separating timing-sensitive setup
#
# Receiver invocations are full bash commands, most pointing at the start_*
# helpers above. This keeps the table flat and grep-able without a custom DSL.
#
# Common runtime template implemented by run_template():
#   1. Run PRE_RECVS in order with brief settling sleeps.
#   2. Start PRODUCER, sleep 2s for v4l2loopback / topic to settle.
#   3. Eval STACK.
#   4. Run POST_RECVS in order (these include their own sleeps).
# Producer ordering matters for V4L2-loopback scenarios: v4l2loopback
# exclusive_caps=1 leaves devices direction-less until a producer calls
# VIDIOC_S_FMT and writes a frame. frame_gen sets the format and starts
# emitting frames immediately (streaming base-video loader, see
# frame_render.cpp); the 2 s settling window is plenty.
# =============================================================================

SCENARIO_TABLE=(
  # --- cam-ros: Camera -> ROS topic ---
  "cam-ros-rcs-yuv|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_rcs rcs_v4l2_yuv_rosonly.yaml|"
  "cam-ros-rcs-mjpeg|start_ros_recv_jpeg ${RCS_JPEG_TOPIC}|framegen-mjpeg|start_rcs rcs_v4l2_mjpeg_rosonly.yaml|"
  "cam-ros-rcs-mjpeg-decoded|start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_rcs rcs_v4l2_mjpeg_decoded.yaml|"
  "cam-ros-usb_cam-yuv|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks usb_cam_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}|"
  "cam-ros-usb_cam-mjpeg|start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_bg ros2 launch ros_camera_server_benchmarks usb_cam_only_mjpeg.launch.yaml device:=/dev/video41 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}|"
  "cam-ros-gscam-yuv|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks gscam2_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}|"
  "cam-ros-gscam-mjpeg|start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_bg ros2 launch ros_camera_server_benchmarks gscam2_only_mjpeg.launch.yaml device:=/dev/video41 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}|"

  # --- cam-stream: Camera -> stream only ---
  # cam-stream-floor: stack-less encode -> UDP-RTP -> decode reference. No
  # jitter buffer, no retransmit window — latency floor is bare encode + RTP
  # payload + decode. Receiver opens udpsrc before the sender to avoid
  # losing the first packets.
  "cam-stream-floor|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|framegen-yuyv|start_floor_pipeline|"
  "cam-stream-rcs-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|framegen-yuyv|start_rcs rcs_v4l2_yuv_rtponly.yaml|"
  # webrtc_recv is started AFTER ros_camera_server binds the signaling
  # socket; ros-stream webrtc scenarios use the same post-stack ordering.
  # Receiver-first ordering races the WS server bring-up and exhausts retries.
  "cam-stream-rcs-yuv-webrtc||framegen-yuyv|start_rcs rcs_v4l2_yuv_webrtc.yaml|sleep 3;start_webrtc_recv \${WS_BENCH0}"
  "cam-stream-usb_cam+gst_bridge-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks usb_cam_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  "cam-stream-gscam+gst_bridge-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks gscam2_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  # web_video_server boots an HTTP server; gst_recv is started after a 5 s
  # settling window so the first request hits a server already serving frames.
  "cam-stream-usb_cam+web_video_server-yuv-mjpeg||framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks web_video_server_mjpeg_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS} wvs_port:=\${WVS_PORT}|sleep 5;start_gst_recv http-mjpeg \${WVS_MJPEG_URL}"
  "cam-stream-usb_cam+rtsp_fkie-yuv-h264||framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks rtsp_h264_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS} rtsp_bitrate:=\$((BITRATE * 1000))|sleep 3;start_rtsp_stream_recv"
  # ffmpeg_image_transport arm: usb_cam YUYV -> rgb8 with the ffmpeg
  # publisher plugin loaded directly on usb_cam's image_transport publisher
  # (libav encode in-process; no producer-side `republish raw ffmpeg` hop).
  # Receiver decodes via `republish ffmpeg raw`; +1 ROS topic hop only on
  # the receive side. ffmpeg_encoder picked from BENCH_FFMPEG_ENC
  # (env_fairness.sh) so the libav backend tracks --encoder=va|nv|mpp
  # identically to RCS / gst_bridge / rtsp_fkie.
  "cam-stream-usb_cam+ffmpeg_transport-yuv-h264||framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks ffmpeg_h264_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS} ffmpeg_encoder:=\${BENCH_FFMPEG_ENC} ffmpeg_bitrate:=\$((BITRATE * 1000)) ffmpeg_av_options:=\${BENCH_FFMPEG_AV_OPTS}|sleep 3;start_ffmpeg_stream_recv /bench/out_raw"

  # --- ros-stream: ROS img -> stream only ---
  "ros-stream-rcs-raw-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|rospub-raw|start_rcs rcs_ros_raw.yaml|"
  "ros-stream-rcs-jpeg-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|rospub-jpeg|start_rcs rcs_ros_jpeg.yaml|"
  "ros-stream-rcs-raw-webrtc||rospub-raw|start_rcs rcs_ros_raw_webrtc.yaml|sleep 3;start_webrtc_recv \${WS_BENCH0}"
  "ros-stream-rcs-jpeg-webrtc||rospub-jpeg|start_rcs rcs_ros_jpeg_webrtc.yaml|sleep 3;start_webrtc_recv \${WS_BENCH0}"
  "ros-stream-gst_bridge-raw-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT}|rospub-raw|start_bridge_pipeline /bench/image_raw|"
  "ros-stream-web_video_server-raw-mjpeg||rospub-raw|start_bg ros2 launch ros_camera_server_benchmarks web_video_server_mjpeg_ros_raw.launch.yaml wvs_port:=\${WVS_PORT}|sleep 5;start_gst_recv http-mjpeg \${WVS_MJPEG_RAW_URL}"
  "ros-stream-rtsp_fkie-raw-h264||rospub-raw|start_bg ros2 launch ros_camera_server_benchmarks rtsp_h264_ros_raw.launch.yaml image_topic:=/bench/image_raw rtsp_bitrate:=\$((BITRATE * 1000))|sleep 3;start_rtsp_stream_recv"
  # ros-stream-ffmpeg_transport-raw-h264: ros_pub publishes raw via
  # image_transport, so ffmpeg_image_transport's publisher plugin loads
  # in-process and encoded packets land on /bench/image_raw/ffmpeg without
  # a separate `republish raw ffmpeg` hop. Mirrors how a real application
  # using image_transport would publish ffmpeg frames.
  "ros-stream-ffmpeg_transport-raw-h264||rospub-raw-ffmpeg_transport||sleep 3;start_ffmpeg_stream_recv /bench/image_raw"

  # --- cam-both: Camera -> stream + ROS topic ---
  "cam-both-rcs-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_rcs rcs_v4l2_yuv.yaml|"
  "cam-both-rcs-mjpeg-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_jpeg ${RCS_JPEG_TOPIC}|framegen-mjpeg|start_rcs rcs_v4l2_mjpeg.yaml|"
  "cam-both-rcs-mjpeg-decoded-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_rcs rcs_v4l2_mjpeg_decoded_rtp.yaml|"
  # webrtc index is 1 in rcs_v4l2_yuv_both_webrtc.yaml (ros2 is index 0).
  "cam-both-rcs-yuv-webrtc|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_rcs rcs_v4l2_yuv_both_webrtc.yaml|sleep 3;start_webrtc_recv \${WS_BENCH1}"
  "cam-both-usb_cam+gst_bridge-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks usb_cam_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  "cam-both-gscam+gst_bridge-yuv-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks gscam2_only_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  "cam-both-usb_cam+gst_bridge-mjpeg-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_bg ros2 launch ros_camera_server_benchmarks usb_cam_only_mjpeg.launch.yaml device:=/dev/video41 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  "cam-both-gscam+gst_bridge-mjpeg-rtp|start_gst_recv udp-rtp udp://127.0.0.1:\${RTP_PORT};start_ros_recv_raw /bench/out_raw|framegen-mjpeg|start_bg ros2 launch ros_camera_server_benchmarks gscam2_only_mjpeg.launch.yaml device:=/dev/video41 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS}; sleep 1; start_bridge_pipeline /bench/out_raw|"
  "cam-both-usb_cam+rtsp_fkie-yuv-h264|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks rtsp_h264_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS} rtsp_bitrate:=\$((BITRATE * 1000))|sleep 3;start_rtsp_stream_recv"
  "cam-both-usb_cam+ffmpeg_transport-yuv-h264|start_ros_recv_raw /bench/out_raw|framegen-yuyv|start_bg ros2 launch ros_camera_server_benchmarks ffmpeg_h264_yuv.launch.yaml device:=/dev/video40 image_topic:=/bench/out_raw width:=\${WIDTH} height:=\${HEIGHT} fps:=\${FPS} ffmpeg_encoder:=\${BENCH_FFMPEG_ENC} ffmpeg_bitrate:=\$((BITRATE * 1000)) ffmpeg_av_options:=\${BENCH_FFMPEG_AV_OPTS}|sleep 3;start_ffmpeg_stream_recv /bench/out_raw"
)

# Find the row for the requested scenario.
ROW=""
for r in "${SCENARIO_TABLE[@]}"; do
  if [[ "${r%%|*}" == "${SCENARIO}" ]]; then ROW="$r"; break; fi
done
if [[ -z "${ROW}" ]]; then
  echo "unknown scenario: ${SCENARIO}" >&2
  echo "  ids follow {cam-ros|cam-stream|ros-stream|cam-both}-{stack}-{variant}, see scripts/run_all.sh" >&2
  exit 2
fi

IFS='|' read -r _ PRE_RECVS PRODUCER STACK POST_RECVS <<< "${ROW}"

run_actions() {  # actions-string  (semicolon-separated)
  [[ -z "$1" ]] && return
  # Restrict IFS=';' to the split itself. The eval'd actions call helpers
  # (start_floor_pipeline, start_bridge_pipeline, start_bg ...) that rely on
  # default whitespace word-splitting when expanding ${BENCH_ENC_GST} and
  # other multi-token shell variables; leaking IFS=';' into them collapses
  # those into a single argv token and gst-launch errors with "syntax error".
  local -a ACTS
  local IFS_SAVED="${IFS}"
  IFS=';' read -ra ACTS <<< "$1"
  IFS="${IFS_SAVED}"
  for a in "${ACTS[@]}"; do
    a="${a#"${a%%[![:space:]]*}"}"   # ltrim
    a="${a%"${a##*[![:space:]]}"}"   # rtrim
    [[ -z "$a" ]] && continue
    eval "$a"
  done
}

# ---- 1. pre-stack receivers ----
run_actions "${PRE_RECVS}"
[[ -n "${PRE_RECVS}" ]] && sleep 1

# ---- 2. producer ----
case "${PRODUCER}" in
  framegen-yuyv)   start_frame_gen yuyv /dev/video40 ;;
  framegen-mjpeg)  start_frame_gen mjpeg /dev/video41 ;;
  rospub-raw)      start_ros_pub raw /bench/image_raw ;;
  # ros_pub publishes raw via image_transport, so the ffmpeg plugin loads
  # in-process when there is a subscriber on `<topic>/ffmpeg`. This variant
  # injects the ffmpeg.* plugin parameters so
  # ros-stream-ffmpeg_transport-raw-h264 encodes inside ros_pub itself rather
  # than via a separate `republish raw ffmpeg` hop — matching how a real
  # application using image_transport would publish ffmpeg frames. Requires
  # BENCH_FFMPEG_ENC and BENCH_FFMPEG_AV_OPTS in the environment (set by
  # env_fairness.sh).
  rospub-raw-ffmpeg_transport)
    BENCH_ROS_PUB_FFMPEG=1 \
      LD_PRELOAD="$(ros2 pkg prefix ros_camera_server_benchmarks)/lib/ros_camera_server_benchmarks/libvaapi_drm_redirect_shim.so${LD_PRELOAD:+:${LD_PRELOAD}}" \
      start_ros_pub raw /bench/image_raw ;;
  # ros_camera_server's ros2 input auto-appends /compressed for jpeg/png
  # (image_transport convention), so the producer must publish to the same
  # suffixed name it ends up subscribing to. The base topic (/bench/image_compressed)
  # stays unchanged in rcs_ros_jpeg*.yaml.
  rospub-jpeg)     start_ros_pub compressed /bench/image_compressed/compressed ;;
  "")              ;;  # rare; no producer (would also imply no stack — unused)
  *)               echo "bad producer in table: ${PRODUCER}" >&2; exit 2 ;;
esac
sleep 2

# ---- 3. stack-under-test ----
# Allow ';'-separated stack steps so a scenario can chain a ros2 launch + a
# bridge_pipeline helper without forcing a separate POST_RECVS slot.
PRE_STACK_PID_COUNT=${#PIDS[@]}
run_actions "${STACK}"
# PIDS appended during STACK are session leaders for the libraries under test;
# slicing here isolates them from harness procs (frame_gen / ros_pub /
# *_recv) and from later POST_RECVS additions. Non-interactive bash means
# `setsid` in start_bg execs (no fork), so each $! is its own session leader
# with PID == SID.
STACK_SIDS=("${PIDS[@]:${PRE_STACK_PID_COUNT}}")

# ---- 4. post-stack receivers (with their own sleeps) ----
run_actions "${POST_RECVS}"

# ---- 5. CPU sampler over stack-under-test sessions ----
# Started after POST_RECVS so any sleep/settle in those steps doesn't eat
# into the sampler's run window. Sampler self-exits at --duration; the
# cleanup() trap is a belt-and-suspenders backstop (the sampler's pid is in
# PIDS via start_bg).
if (( ${#STACK_SIDS[@]} > 0 )); then
  start_bg python3 "${HERE}/helpers/cpu_sample.py" \
    --sids "$(IFS=,; echo "${STACK_SIDS[*]}")" \
    --csv "${OUT}/cpu.csv" \
    --interval-hz 2 \
    --duration "${DURATION}"
fi

echo "[run_scenario ${SCENARIO}] running for ${DURATION}s"
sleep "${DURATION}"

cleanup
trap - EXIT

python3 "${HERE}/helpers/bench_aggregate.py" \
  --scenario "${SCENARIO}" \
  --output-dir "${OUT}" \
  --duration "${DURATION}" --warmup-frames "${WARMUP}" --fps "${FPS}" \
  --report-csv "${RESULTS_ROOT}/report.csv"

echo "[run_scenario ${SCENARIO}] done -> ${OUT}/summary.json"
