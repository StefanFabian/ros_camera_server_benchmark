/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/frame_render.hpp"
#include "ros_camera_server_benchmarks/mjpeg_encode.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <gst/gst.h>

using ros_camera_server_benchmarks::now_ns_monotonic;
using ros_camera_server_benchmarks::frame_render::BaseVideoLoader;
using ros_camera_server_benchmarks::frame_render::VideoFrame;
using ros_camera_server_benchmarks::frame_render::i420_to_rgb24;
using ros_camera_server_benchmarks::frame_render::i420_to_yuyv;
using ros_camera_server_benchmarks::frame_render::render_i420;

namespace {

std::atomic<bool> g_stop{ false };

void on_signal(int) { g_stop = true; }

bool set_v4l2_format(int fd, int width, int height, uint32_t pixfmt)
{
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixfmt;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (pixfmt == V4L2_PIX_FMT_YUYV) {
    fmt.fmt.pix.bytesperline = width * 2;
    fmt.fmt.pix.sizeimage = width * height * 2;
  } else if (pixfmt == V4L2_PIX_FMT_RGB24) {
    fmt.fmt.pix.bytesperline = width * 3;
    fmt.fmt.pix.sizeimage = width * height * 3;
  } else {
    fmt.fmt.pix.bytesperline = 0;
    fmt.fmt.pix.sizeimage = width * height * 2;
  }
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    std::fprintf(stderr, "VIDIOC_S_FMT failed: %s\n", std::strerror(errno));
    return false;
  }
  // V4L2 may silently substitute a different format. v4l2loopback in
  // particular locks the format after first negotiation and rejects mismatched
  // requests by returning the existing format. Detect that here so callers
  // don't push e.g. RGB24 bytes into a YUYV buffer.
  if (fmt.fmt.pix.pixelformat != pixfmt) {
    char want[5] = { static_cast<char>(pixfmt & 0xff),
                     static_cast<char>((pixfmt >> 8) & 0xff),
                     static_cast<char>((pixfmt >> 16) & 0xff),
                     static_cast<char>((pixfmt >> 24) & 0xff), 0 };
    char got[5] = { static_cast<char>(fmt.fmt.pix.pixelformat & 0xff),
                    static_cast<char>((fmt.fmt.pix.pixelformat >> 8) & 0xff),
                    static_cast<char>((fmt.fmt.pix.pixelformat >> 16) & 0xff),
                    static_cast<char>((fmt.fmt.pix.pixelformat >> 24) & 0xff), 0 };
    std::fprintf(stderr,
                 "VIDIOC_S_FMT: driver substituted format '%s' for requested '%s'.\n"
                 "  On v4l2loopback this means a previous opener locked the format.\n"
                 "  Reload the module: sudo modprobe -r v4l2loopback && "
                 "ros2 run ros_camera_server_benchmarks setup_loopback.sh\n",
                 got, want);
    return false;
  }
  return true;
}

void usage()
{
  std::cerr << "Usage: frame_gen --device /dev/videoN --width W --height H "
               "--fps F --format yuyv|rgb|mjpeg [--cell N] [--duration S] "
               "[--video PATH] [--video-cycle-seconds N] "
               "[--underrun-csv PATH]\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string device = "/dev/video10";
  int width = 1280, height = 720, fps = 30, cell = 16, duration_s = 0;
  int video_cycle_s = 5;
  std::string format = "yuyv";
  std::string video_path;
  std::string underrun_csv_path;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) { std::cerr << name << " requires value\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--device") device = next("--device");
    else if (a == "--width") width = std::atoi(next("--width"));
    else if (a == "--height") height = std::atoi(next("--height"));
    else if (a == "--fps") fps = std::atoi(next("--fps"));
    else if (a == "--format") format = next("--format");
    else if (a == "--cell") cell = std::atoi(next("--cell"));
    else if (a == "--duration") duration_s = std::atoi(next("--duration"));
    else if (a == "--video") video_path = next("--video");
    else if (a == "--video-cycle-seconds")
      video_cycle_s = std::atoi(next("--video-cycle-seconds"));
    else if (a == "--underrun-csv") underrun_csv_path = next("--underrun-csv");
    else if (a == "--help" || a == "-h") { usage(); return 0; }
    else { std::cerr << "unknown arg: " << a << "\n"; usage(); return 2; }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // Set the v4l2loopback format BEFORE any frames flow. v4l2loopback consumers
  // (usb_cam, gscam, gst-launch v4l2src) probe via VIDIOC_ENUM_FMT and need
  // both the format set AND a written frame to negotiate; the kick-off
  // happens once the producer loop writes its first buffer below.
  int fd = open(device.c_str(), O_WRONLY);
  if (fd < 0) {
    std::fprintf(stderr, "open(%s) failed: %s\n", device.c_str(),
                 std::strerror(errno));
    return 1;
  }

  uint32_t pixfmt;
  if (format == "mjpeg") {
    pixfmt = V4L2_PIX_FMT_MJPEG;
  } else if (format == "rgb") {
    pixfmt = V4L2_PIX_FMT_RGB24;
  } else if (format == "yuyv") {
    pixfmt = V4L2_PIX_FMT_YUYV;
  } else {
    std::fprintf(stderr, "unknown --format: %s\n", format.c_str());
    close(fd);
    return 2;
  }
  if (!set_v4l2_format(fd, width, height, pixfmt)) {
    close(fd);
    return 1;
  }

  // Streaming base-video loader: pulls samples on a worker thread and lets
  // the producer start writing real video frames within one decode latency.
  // Earlier versions blocked here for the entire cycle (~30 s on software
  // decode), keeping v4l2loopback un-negotiated long past consumer probe and
  // crashing usb_cam / gscam / gst-launch downstream.
  BaseVideoLoader loader(video_path, width, height, fps, video_cycle_s);
  if (!video_path.empty()) {
    if (loader.failed()) {
      std::fprintf(stderr,
                   "frame_gen: --video given but loader failed; "
                   "falling back to flat gray.\n");
    } else {
      // Block until the first decoded frame is in. Without this the producer
      // would write its first synthetic-fallback frame while the loader is
      // still spinning up; the requirement is "always real video" once a
      // --video file is supplied.
      uint64_t t0 = now_ns_monotonic();
      if (!loader.wait_first_frame(std::chrono::seconds(10))) {
        std::fprintf(stderr,
                     "frame_gen: timed out waiting for first decoded frame; "
                     "falling back to flat gray.\n");
      } else {
        double dt_ms = (now_ns_monotonic() - t0) / 1e6;
        std::fprintf(stderr,
                     "frame_gen: first base-video frame ready after %.1f ms "
                     "(cycle capacity = %zu frames)\n",
                     dt_ms, loader.cycle_capacity());
      }
    }
  }

  std::vector<uint8_t> i420;
  std::vector<uint8_t> outbuf;
  uint32_t frame_id = 0;
  const uint64_t frame_period_ns = 1000000000ULL / static_cast<uint64_t>(fps);
  const uint64_t start_ns = now_ns_monotonic();
  uint64_t next_deadline = start_ns;

  uint64_t underruns = 0;          // count of missed frame deadlines
  uint64_t underrun_total_late_ns = 0;
  bool last_was_underrun = false;  // throttle warning rate

  // Producer-side I420->wire-format conversion is bracketed by the per-frame
  // ts stamp, so its cost is included in measured end-to-end latency. Track
  // it so bench_aggregate.py can subtract harness overhead from the SUT
  // number; without this, JPEG runs look slower than YUV runs purely because
  // libjpeg is more work than i420_to_yuyv.
  uint64_t encode_count = 0;
  uint64_t encode_total_ns = 0;
  uint64_t encode_max_ns = 0;

  while (!g_stop) {
    uint64_t ts = now_ns_monotonic();
    const VideoFrame* base = loader.failed() ? nullptr : loader.frame(frame_id);
    render_i420(i420, width, height, frame_id, ts, cell, base);

    uint64_t enc_t0 = now_ns_monotonic();
    if (pixfmt == V4L2_PIX_FMT_YUYV) {
      i420_to_yuyv(i420.data(), outbuf, width, height);
    } else if (pixfmt == V4L2_PIX_FMT_RGB24) {
      i420_to_rgb24(i420.data(), outbuf, width, height);
    } else {
      if (!ros_camera_server_benchmarks::i420_to_jpeg(i420.data(), width,
                                                      height, outbuf)) {
        std::fprintf(stderr, "jpeg encode failed\n");
        break;
      }
    }
    uint64_t enc_dt = now_ns_monotonic() - enc_t0;
    encode_total_ns += enc_dt;
    if (enc_dt > encode_max_ns) encode_max_ns = enc_dt;
    ++encode_count;

    ssize_t n = write(fd, outbuf.data(), outbuf.size());
    if (n < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "write failed: %s\n", std::strerror(errno));
      break;
    }

    ++frame_id;
    if (duration_s > 0 &&
        (now_ns_monotonic() - start_ns) >= static_cast<uint64_t>(duration_s) * 1000000000ULL) {
      break;
    }

    next_deadline += frame_period_ns;
    uint64_t now = now_ns_monotonic();
    if (next_deadline > now) {
      struct timespec ts_sleep;
      uint64_t delta = next_deadline - now;
      ts_sleep.tv_sec = static_cast<time_t>(delta / 1000000000ULL);
      ts_sleep.tv_nsec = static_cast<long>(delta % 1000000000ULL);
      nanosleep(&ts_sleep, nullptr);
      last_was_underrun = false;
    } else {
      // Fell behind. Resetting the deadline to "now" caps drift, but it also
      // silently lowers effective fps. Track it so the run isn't reported as
      // a clean 30 fps when it wasn't.
      uint64_t late_ns = now - next_deadline;
      ++underruns;
      underrun_total_late_ns += late_ns;
      if (!last_was_underrun) {
        std::fprintf(stderr,
                     "frame_gen: deadline missed at frame %u (late %.2f ms)\n",
                     frame_id, late_ns / 1e6);
      }
      last_was_underrun = true;
      next_deadline = now;
    }
  }

  close(fd);

  if (!underrun_csv_path.empty()) {
    std::ofstream f(underrun_csv_path);
    if (f) {
      uint64_t total_frames = frame_id;
      f << "# frame_gen producer counters\n";
      f << "frames_produced=" << total_frames << "\n";
      f << "underruns=" << underruns << "\n";
      f << "underrun_total_late_ms="
        << (underrun_total_late_ns / 1e6) << "\n";
      f << "base_video_frames=" << loader.cycle_size() << "\n";
      f << "base_video_catch_up_reuses=" << loader.catch_up_reuses() << "\n";
      f << "encode_count=" << encode_count << "\n";
      f << "encode_total_ns=" << encode_total_ns << "\n";
      f << "encode_max_ns=" << encode_max_ns << "\n";
    } else {
      std::fprintf(stderr, "frame_gen: cannot write %s\n",
                   underrun_csv_path.c_str());
    }
  }

  return 0;
}
