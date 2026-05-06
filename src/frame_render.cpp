/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/frame_render.hpp"
#include "ros_camera_server_benchmarks/marker.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

namespace ros_camera_server_benchmarks::frame_render
{

using namespace ros_camera_server_benchmarks::marker;

BaseVideoLoader::BaseVideoLoader(std::string path, int width, int height,
                                 int fps, int cycle_seconds)
  : path_(std::move(path)),
    width_(width),
    height_(height),
    cycle_capacity_(static_cast<size_t>(std::max(1, cycle_seconds)) *
                    static_cast<size_t>(std::max(1, fps)))
{
  if (path_.empty()) {
    failed_.store(true, std::memory_order_release);
    return;
  }

  gst_init(nullptr, nullptr);

  // max-buffers=2: small backlog so the worker thread doesn't get stalled by
  // a slow append, but doesn't grow unbounded either.
  // drop=false: we want every frame, in order.
  gchar* desc = g_strdup_printf(
      "filesrc location=\"%s\" ! decodebin ! videoconvert ! videoscale ! "
      "video/x-raw,format=I420,width=%d,height=%d ! "
      "appsink name=sink sync=false max-buffers=2 drop=false",
      path_.c_str(), width_, height_);
  GError* err = nullptr;
  pipeline_ = gst_parse_launch(desc, &err);
  g_free(desc);
  if (!pipeline_) {
    std::fprintf(stderr, "BaseVideoLoader: gst_parse_launch failed: %s\n",
                 err ? err->message : "unknown");
    if (err) g_error_free(err);
    failed_.store(true, std::memory_order_release);
    return;
  }
  if (err) g_error_free(err);

  sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (!sink_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    failed_.store(true, std::memory_order_release);
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    std::fprintf(stderr, "BaseVideoLoader: cannot start pipeline\n");
    gst_object_unref(sink_);
    gst_object_unref(pipeline_);
    sink_ = nullptr;
    pipeline_ = nullptr;
    failed_.store(true, std::memory_order_release);
    return;
  }

  // Reserve so emplace_back never reallocates: producer-side reads are then
  // safe against the worker's concurrent append (release-store on
  // cycle_size_ paired with acquire-load on the reader side).
  cycle_frames_.reserve(cycle_capacity_);

  worker_ = std::thread(&BaseVideoLoader::worker, this);
}

BaseVideoLoader::~BaseVideoLoader()
{
  stop_.store(true, std::memory_order_release);
  if (worker_.joinable()) worker_.join();
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  if (sink_) gst_object_unref(sink_);
}

void BaseVideoLoader::worker()
{
  const size_t y_size = static_cast<size_t>(width_) * height_;
  const size_t uv_size = static_cast<size_t>(width_ / 2) * (height_ / 2);
  const size_t expected = y_size + 2 * uv_size;

  while (!stop_.load(std::memory_order_acquire) &&
         cycle_frames_.size() < cycle_capacity_) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_),
                                                     100 * GST_MSECOND);
    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(sink_))) break;
      continue;
    }
    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
      if (map.size >= expected) {
        VideoFrame vf;
        vf.i420.assign(map.data, map.data + expected);
        cycle_frames_.push_back(std::move(vf));
        // Release-store: any reader doing acquire-load on cycle_size_ and
        // reading entries with index < cycle_size_ sees a fully-written
        // VideoFrame.
        cycle_size_.store(cycle_frames_.size(), std::memory_order_release);

        // Wake first-frame waiter on the very first append.
        if (cycle_frames_.size() == 1) {
          std::lock_guard<std::mutex> lk(first_frame_mtx_);
          first_frame_cv_.notify_all();
        }
      }
      gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
  }

  cycle_complete_.store(true, std::memory_order_release);
  // Wake first-frame waiter even on failure so wait_first_frame doesn't hang.
  std::lock_guard<std::mutex> lk(first_frame_mtx_);
  first_frame_cv_.notify_all();
}

bool BaseVideoLoader::wait_first_frame(std::chrono::milliseconds timeout)
{
  if (failed()) return false;
  std::unique_lock<std::mutex> lk(first_frame_mtx_);
  bool got = first_frame_cv_.wait_for(lk, timeout, [this] {
    return cycle_size_.load(std::memory_order_acquire) > 0 ||
           cycle_complete_.load(std::memory_order_acquire);
  });
  return got && cycle_size_.load(std::memory_order_acquire) > 0;
}

const VideoFrame* BaseVideoLoader::frame(uint32_t frame_id)
{
  size_t size = cycle_size_.load(std::memory_order_acquire);
  if (size == 0) return nullptr;
  bool complete = cycle_complete_.load(std::memory_order_acquire);
  if (complete) {
    return &cycle_frames_[frame_id % size];
  }
  if (frame_id < size) {
    return &cycle_frames_[frame_id];
  }
  // Decode lagged behind the producer's tick; reuse the latest decoded frame
  // and surface the count so a slow-decode host is observable in stats.
  catch_up_reuses_.fetch_add(1, std::memory_order_relaxed);
  return &cycle_frames_[size - 1];
}

void render_i420(std::vector<uint8_t>& buf, int width, int height,
                 uint32_t frame_id, uint64_t ts_ns, int cell,
                 const VideoFrame* base)
{
  const int y_size = width * height;
  const int uv_size = (width / 2) * (height / 2);
  buf.resize(y_size + 2 * uv_size);

  if (base) {
    std::memcpy(buf.data(), base->i420.data(), buf.size());
  } else {
    std::memset(buf.data(), 128, y_size);
    std::memset(buf.data() + y_size, 128, uv_size);
    std::memset(buf.data() + y_size + uv_size, 128, uv_size);

    int bar_x = static_cast<int>(frame_id) % width;
    int bar_y0 = std::max(strip_height(width, cell) + 8, height / 2);
    for (int y = bar_y0; y < height; ++y) {
      buf[y * width + bar_x] = 220;
    }
  }

  Marker m;
  m.ts_ns = ts_ns;
  m.frame_id = frame_id;
  m.width_hint = static_cast<uint16_t>(width);
  encode_y(buf.data(), width, width, height, m, cell);

  uint8_t* u_plane = buf.data() + y_size;
  uint8_t* v_plane = buf.data() + y_size + uv_size;
  clear_uv_strip_i420(u_plane, v_plane, width / 2, width, height, cell);
}

void i420_to_rgb24(const uint8_t* i420, std::vector<uint8_t>& rgb,
                   int width, int height)
{
  rgb.resize(static_cast<size_t>(width) * height * 3);
  const uint8_t* y_plane = i420;
  const uint8_t* u_plane = i420 + width * height;
  const uint8_t* v_plane = i420 + width * height + (width / 2) * (height / 2);
  for (int y = 0; y < height; ++y) {
    const uint8_t* yrow = y_plane + y * width;
    const uint8_t* urow = u_plane + (y / 2) * (width / 2);
    const uint8_t* vrow = v_plane + (y / 2) * (width / 2);
    uint8_t* out = rgb.data() + y * width * 3;
    for (int x = 0; x < width; ++x) {
      int Y = yrow[x];
      int U = urow[x / 2] - 128;
      int V = vrow[x / 2] - 128;
      int r = Y + (V * 1436 + 512) / 1024;
      int g = Y - (U * 352 + V * 731 + 512) / 1024;
      int b = Y + (U * 1814 + 512) / 1024;
      out[3 * x + 0] = static_cast<uint8_t>(std::clamp(r, 0, 255));
      out[3 * x + 1] = static_cast<uint8_t>(std::clamp(g, 0, 255));
      out[3 * x + 2] = static_cast<uint8_t>(std::clamp(b, 0, 255));
    }
  }
}

void i420_to_yuyv(const uint8_t* i420, std::vector<uint8_t>& yuyv,
                  int width, int height)
{
  yuyv.resize(width * height * 2);
  const uint8_t* y_plane = i420;
  const uint8_t* u_plane = i420 + width * height;
  const uint8_t* v_plane = i420 + width * height + (width / 2) * (height / 2);
  for (int y = 0; y < height; ++y) {
    const uint8_t* yrow = y_plane + y * width;
    const uint8_t* urow = u_plane + (y / 2) * (width / 2);
    const uint8_t* vrow = v_plane + (y / 2) * (width / 2);
    uint8_t* out = yuyv.data() + y * width * 2;
    for (int x = 0; x < width; x += 2) {
      out[2 * x + 0] = yrow[x + 0];
      out[2 * x + 1] = urow[x / 2];
      out[2 * x + 2] = yrow[x + 1];
      out[2 * x + 3] = vrow[x / 2];
    }
  }
}

} // namespace ros_camera_server_benchmarks::frame_render
