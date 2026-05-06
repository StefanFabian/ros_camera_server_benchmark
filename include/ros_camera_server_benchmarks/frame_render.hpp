/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS_FRAME_RENDER_HPP
#define ROS_CAMERA_SERVER_BENCHMARKS_FRAME_RENDER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

typedef struct _GstElement GstElement;

namespace ros_camera_server_benchmarks::frame_render
{

// One pre-decoded I420 frame from a --video file.
struct VideoFrame {
  std::vector<uint8_t> i420;  // y_size + 2 * uv_size bytes
};

// Streams video into memory frame-by-frame so the producer can begin writing
// to v4l2loopback within one decode latency rather than waiting for the
// entire cycle (which can take tens of seconds at long cycle lengths).
//
// Construction starts a GStreamer pipeline + a worker thread that decodes
// frames and appends them to an internally-reserved ring of `cycle_seconds *
// fps` slots. The producer calls `frame(frame_id)` once per tick:
//
//   - Before the ring is full: returns the freshest decoded frame. If the
//     ring isn't yet at frame_id, the most recently decoded frame is returned
//     and `catch_up_reuses()` is incremented so a slow-decode host is
//     observable in the run summary.
//   - Once the ring is full: returns `cycle_frames_[frame_id % cycle_size()]`,
//     looping the cycle.
//
// Memory is bounded: the ring is `reserve()`d up-front so the worker never
// reallocates, and the producer's reads are lock-free against the writer.
class BaseVideoLoader
{
public:
  // Empty `path` constructs a no-op loader (failed() == true). Callers can
  // treat this as "no base video" and fall back to synthetic content.
  BaseVideoLoader(std::string path, int width, int height, int fps,
                  int cycle_seconds);
  ~BaseVideoLoader();

  BaseVideoLoader(const BaseVideoLoader&) = delete;
  BaseVideoLoader& operator=(const BaseVideoLoader&) = delete;

  // Block (up to `timeout`) until at least one frame is decoded. Returns
  // true if a frame is available, false on timeout / decode failure.
  bool wait_first_frame(std::chrono::milliseconds timeout);

  // Returns nullptr only if no frames have been decoded yet (and
  // wait_first_frame wasn't used). Otherwise always returns a pointer to a
  // VideoFrame the caller may read until the next call to frame().
  const VideoFrame* frame(uint32_t frame_id);

  bool failed() const noexcept { return failed_.load(std::memory_order_acquire); }
  size_t cycle_size() const noexcept { return cycle_size_.load(std::memory_order_acquire); }
  bool cycle_complete() const noexcept { return cycle_complete_.load(std::memory_order_acquire); }
  uint64_t catch_up_reuses() const noexcept { return catch_up_reuses_.load(std::memory_order_relaxed); }
  size_t cycle_capacity() const noexcept { return cycle_capacity_; }

private:
  void worker();

  std::string path_;
  int width_;
  int height_;
  size_t cycle_capacity_;

  GstElement* pipeline_ = nullptr;  // owned by worker; tear down in dtor
  GstElement* sink_ = nullptr;

  std::vector<VideoFrame> cycle_frames_;  // reserved up-front, no realloc
  std::atomic<size_t> cycle_size_{ 0 };
  std::atomic<bool> cycle_complete_{ false };
  std::atomic<bool> failed_{ false };
  std::atomic<bool> stop_{ false };
  std::atomic<uint64_t> catch_up_reuses_{ 0 };

  std::mutex first_frame_mtx_;
  std::condition_variable first_frame_cv_;

  std::thread worker_;
};

// Render an I420 frame:
//   * if `base` is non-null, copy its pixels (Y + UV) into `buf`.
//   * else, render the synthetic flat-gray + walking-bar pattern.
// Then overlay the marker strip on the Y plane (top rows) and clear the
// chroma over the strip so YUV→RGB consumers reading luma as (R+G+B)/3 don't
// see colour-shifted cell bits.
//
// `buf` is resized to y_size + 2 * uv_size.
void render_i420(std::vector<uint8_t>& buf, int width, int height,
                 uint32_t frame_id, uint64_t ts_ns, int cell,
                 const VideoFrame* base);

// Convert I420 to packed RGB24 (R first), BT.601 limited-range Q10. Marker
// strip rows have U=V=128 by construction so they decode to R=G=B=Y exactly.
void i420_to_rgb24(const uint8_t* i420, std::vector<uint8_t>& rgb,
                   int width, int height);

// Convert I420 to packed YUYV (4:2:2).
void i420_to_yuyv(const uint8_t* i420, std::vector<uint8_t>& yuyv,
                  int width, int height);

} // namespace ros_camera_server_benchmarks::frame_render

#endif
