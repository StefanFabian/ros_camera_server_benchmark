#!/usr/bin/env bash
# Run the full benchmark matrix N times per scenario.
#
# Encoder: --encoder=va|nv|mpp (or BENCH_ENCODER env) selects vah264lpenc /
# nvh264enc / mpph264enc. If unset and more than one is installed,
# resolve_encoder.sh prompts interactively once up front (before the
# scenario loop) and exports BENCH_ENCODER so each per-scenario
# run_scenario.sh subprocess inherits it and runs without further
# interaction.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPEATS="${BENCH_REPEATS:-5}"

# Pick out --encoder=... before the positional scenario list.
POS_ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --encoder=va|--encoder=nv|--encoder=mpp) export BENCH_ENCODER="${arg#--encoder=}" ;;
    --encoder=*) echo "Usage: $0 [--encoder=va|nv|mpp] [scenarios...]" >&2; exit 2 ;;
    *) POS_ARGS+=("${arg}") ;;
  esac
done

# `"${@:-default}"` quoted collapses the default into a single element, so
# branch explicitly on $# to pick either user-supplied scenarios or the
# full matrix.
ALL_SCENARIOS=(
  # Scenario ID convention: <group>-<stack>[+<bridge_stack>]-[<input>]-[<output>]
  # `+` joins composite stacks (usb_cam+gst_bridge, usb_cam+rtsp_fkie, ...).
  # Input format always present where the stack has one (yuv, mjpeg, raw, jpeg).
  # Output transport always present where the stack outputs a stream
  # (rtp, webrtc, mjpeg, h264-for-RTSP).
  #
  # cam-ros — Camera -> ROS topic only. cam-ros-rcs-mjpeg publishes
  # CompressedImage (passthrough); cam-ros-rcs-mjpeg-decoded decodes MJPEG
  # to rgb8 inside ros_camera_server for an apples-to-apples compare with
  # usb_cam / gscam (which always decode).
  cam-ros-rcs-yuv cam-ros-rcs-mjpeg cam-ros-rcs-mjpeg-decoded
  cam-ros-usb_cam-yuv cam-ros-usb_cam-mjpeg
  cam-ros-gscam-yuv cam-ros-gscam-mjpeg
  # cam-stream — Camera -> H.264 stream only.
  # web_video_server H.264 arms excluded: upstream emits an empty avcC in
  # the served fragmented MP4, so no standard H.264 consumer can bootstrap.
  # MJPEG remains the working web_video_server arm.
  # *-rtsp_fkie-* use FKIE's own rtsp_sub plugin (via image_transport
  # republish rtsp raw) since standard RTSP clients fail SETUP/PLAY against
  # rtsp_pub.
  # cam-stream-{usb_cam,gscam}+gst_bridge-yuv-rtp cover the bridge stacks so
  # the cam-stream headline table has a same-group bridge entry instead of
  # forcing the reader to look up the cam-both stream column.
  cam-stream-floor
  cam-stream-rcs-yuv-rtp cam-stream-rcs-yuv-webrtc
  cam-stream-usb_cam+gst_bridge-yuv-rtp cam-stream-gscam+gst_bridge-yuv-rtp
  cam-stream-usb_cam+web_video_server-yuv-mjpeg cam-stream-usb_cam+rtsp_fkie-yuv-h264
  cam-stream-usb_cam+ffmpeg_transport-yuv-h264
  # ros-stream — ROS image input -> stream.
  ros-stream-rcs-raw-rtp ros-stream-rcs-jpeg-rtp
  ros-stream-rcs-raw-webrtc ros-stream-rcs-jpeg-webrtc
  ros-stream-gst_bridge-raw-rtp
  ros-stream-web_video_server-raw-mjpeg ros-stream-rtsp_fkie-raw-h264
  ros-stream-ffmpeg_transport-raw-h264
  # cam-both — Camera -> both stream + ROS topic. cam-both-rcs-mjpeg-decoded-rtp
  # mirrors cam-ros-rcs-mjpeg-decoded for the dual-receiver direction,
  # comparable to the usb_cam / gscam + gst_bridge mjpeg arms which always
  # decode to rgb8.
  cam-both-rcs-yuv-rtp cam-both-rcs-mjpeg-rtp cam-both-rcs-mjpeg-decoded-rtp
  cam-both-rcs-yuv-webrtc
  cam-both-usb_cam+gst_bridge-yuv-rtp cam-both-gscam+gst_bridge-yuv-rtp
  cam-both-usb_cam+gst_bridge-mjpeg-rtp cam-both-gscam+gst_bridge-mjpeg-rtp
  cam-both-usb_cam+rtsp_fkie-yuv-h264
  cam-both-usb_cam+ffmpeg_transport-yuv-h264
)

# `"${@:-default}"` quoted collapses the default into a single element, so
# branch explicitly on $# to pick either user-supplied scenarios or the
# full matrix. BENCH_GROUP=cam-ros / cam-stream / ros-stream / cam-both
# narrows the matrix to one group without listing scenarios manually.
if (( ${#POS_ARGS[@]} > 0 )); then
  SCENARIOS=("${POS_ARGS[@]}")
elif [[ -n "${BENCH_GROUP:-}" ]]; then
  SCENARIOS=()
  for s in "${ALL_SCENARIOS[@]}"; do
    [[ "$s" == "${BENCH_GROUP}-"* ]] && SCENARIOS+=("$s")
  done
  if (( ${#SCENARIOS[@]} == 0 )); then
    echo "BENCH_GROUP=${BENCH_GROUP} matched no scenarios" >&2
    exit 2
  fi
else
  SCENARIOS=("${ALL_SCENARIOS[@]}")
fi

# Up-front sanity check so we don't loop N*M times printing the same setup
# error. run_scenario.sh repeats this with a more specific message, but
# bailing here saves a lot of noise.
if ! ros2 pkg prefix ros_camera_server_benchmarks >/dev/null 2>&1; then
  echo "ros_camera_server_benchmarks is not visible to ros2." >&2
  echo "  source the workspace first, e.g. 'source install/setup.bash'." >&2
  exit 2
fi

# shellcheck disable=SC1091
source "${HERE}/helpers/check_cpu_governor.sh"

# Resolve the encoder family once, before the loop, so the interactive
# prompt fires at most one time per invocation. Each per-scenario
# run_scenario.sh inherits BENCH_ENCODER and skips re-prompting.
# shellcheck disable=SC1091
source "${HERE}/helpers/resolve_encoder.sh"

fail=0
skipped=0
declare -A skipped_scenarios=()

# Fisher-Yates shuffle of a named array, using $RANDOM. Set BENCH_SEED for a
# reproducible matrix (each repeat reseeded as BENCH_SEED+r so repeats differ
# but the whole run replays); leave it unset for true randomness per run.
shuffle_in_place() {
  local -n _arr="$1"
  local i j tmp
  for (( i = ${#_arr[@]} - 1; i > 0; i-- )); do
    j=$(( RANDOM % (i + 1) ))
    tmp="${_arr[i]}"; _arr[i]="${_arr[j]}"; _arr[j]="${tmp}"
  done
}

# Round-robin repeats so each scenario's reps are spread across the whole
# matrix run window: every scenario runs once per repeat, then again the next
# repeat. The per-repeat order is shuffled so no scenario is pinned to the
# same slot every time. Without round-robin, a 29*5 matrix would run scenario
# A's 5 reps back-to-back on a cool host and scenario Z's 5 reps last on a hot
# host. Round-robin alone still leaves each scenario in a fixed position
# within every repeat, so its position bias (early=cool, late=hot) survives
# averaging; shuffling each repeat's order spreads every scenario's slots
# across the run so cross-scenario thermal drift averages out.
for r in $(seq 1 "${REPEATS}"); do
  order=("${SCENARIOS[@]}")
  [[ -n "${BENCH_SEED:-}" ]] && RANDOM=$((BENCH_SEED + r))
  shuffle_in_place order
  echo "==== repeat ${r}/${REPEATS} order: ${order[*]} ===="
  for s in "${order[@]}"; do
    # Skip subsequent repeats once we know a scenario's dependency is
    # missing; the second run won't magically install the package.
    if [[ -n "${skipped_scenarios[$s]:-}" ]]; then
      continue
    fi
    echo "==== ${s} run ${r}/${REPEATS} ===="
    rc=0
    BENCH_RESULTS_DIR="${PWD}/results/run${r}" \
      "${HERE}/run_scenario.sh" "${s}" || rc=$?
    if (( rc == 3 )); then
      echo "scenario ${s}: skipped (missing dependency)"
      skipped_scenarios[$s]=1
      skipped=$((skipped + 1))
    elif (( rc != 0 )); then
      echo "scenario ${s} run ${r} failed (rc=${rc})"
      fail=$((fail + 1))
    fi
  done
done

if (( skipped > 0 )); then
  echo "${skipped} scenario(s) skipped due to missing dependencies: ${!skipped_scenarios[*]}" >&2
fi
if (( fail > 0 )); then
  echo "${fail} scenario run(s) failed" >&2
  exit 1
fi
