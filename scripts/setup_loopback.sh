#!/usr/bin/env bash
# Idempotent setup of v4l2loopback devices /dev/video40 (YUYV) and /dev/video41 (MJPEG).
# Run once per host before running the benchmark suite. Requires sudo.
set -euo pipefail

if ! modinfo v4l2loopback >/dev/null 2>&1; then
  echo "v4l2loopback kernel module not installed. Install: sudo apt install v4l2loopback-dkms" >&2
  exit 1
fi

echo "Reloading v4l2loopback (sudo required)..."
# Force reload so each benchmark session starts with no locked-in format.
# v4l2loopback latches the pixel format on first VIDIOC_S_FMT and will then
# silently substitute the latched format for any later request. Without this
# reset, switching scenarios (or moving from YUYV to RGB / RGB to MJPEG)
# fails: frame_gen pushes one byte layout while consumers see the previous
# layout's caps.
sudo modprobe -r v4l2loopback 2>/dev/null || true
# exclusive_caps=0: device advertises both VIDEO_CAPTURE and VIDEO_OUTPUT
# capabilities at all times, regardless of active streamers. With
# exclusive_caps=1 the kernel hides one direction whenever the device is
# idle, which makes the first VIDIOC_S_FMT after a producer-close fail with
# EINVAL ("Device is not a capture/output device"). The bench launches one
# new producer per scenario, so we'd need a module reload between every
# scenario; exclusive_caps=0 sidesteps that. usb_cam / gscam /
# ros_camera_server all accept the resulting bidirectional caps without
# complaint.
sudo modprobe v4l2loopback video_nr=40,41 \
     exclusive_caps=0,0 \
     max_buffers=8 \
     card_label="bench_yuyv,bench_mjpeg"

v4l2-ctl --device=/dev/video40 --info | head -3
v4l2-ctl --device=/dev/video41 --info | head -3
echo "OK"
