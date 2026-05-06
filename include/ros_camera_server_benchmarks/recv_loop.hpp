/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS_RECV_LOOP_HPP
#define ROS_CAMERA_SERVER_BENCHMARKS_RECV_LOOP_HPP

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace ros_camera_server_benchmarks
{

struct RecvLoopConfig {
  std::string csv_path;     // Per-frame latency CSV. Header is written here.
  std::string label;        // Prefix for the final "<label>: seen=N decoded=M" line.
  uint32_t warmup_frames;   // Frames discarded before recording starts.
  uint32_t max_frames;      // Stop after this many recorded frames (0 = unlimited).
  int cell;                 // Marker cell size in pixels.
};

// Shared per-frame bookkeeping for every receiver in the harness. Owns the
// CSV file, marker decode call, warmup-skip + max-frames stop, and the final
// "seen=N decoded=M" stderr summary.
//
// Receivers (gst_recv / webrtc_recv / ros_recv) construct one instance and
// invoke `on_frame()` for each decoded frame. The receiver-side recv_ts must
// already be sampled by the caller at the post-decode / pre-marker-extract
// point (the contract pinned by `MarkerRoundtrip.JpegRecvTimestampOrdering`).
class RecvLoop
{
public:
  explicit RecvLoop(const RecvLoopConfig& cfg);
  ~RecvLoop();

  RecvLoop(const RecvLoop&) = delete;
  RecvLoop& operator=(const RecvLoop&) = delete;

  // Returns false once max_frames has been reached, signalling the caller to
  // tear down its pipeline / shutdown rclcpp / quit its main loop. Returns
  // true otherwise (including when the marker decode failed).
  // `y_plane` must point to a Y plane with `stride` bytes per row.
  bool on_frame(const uint8_t* y_plane, int stride, int width, int height,
                uint64_t recv_ts_ns);

  // True after on_frame() has returned false at least once.
  bool done() const noexcept;

  uint64_t seen() const noexcept;
  uint64_t decoded() const noexcept;

  // Returns true if the CSV file failed to open in the constructor. Caller
  // should bail with a non-zero exit code.
  bool failed() const noexcept;

private:
  RecvLoopConfig cfg_;
  std::ofstream csv_;
  mutable std::mutex mtx_;
  uint64_t seen_ = 0;
  uint64_t decoded_ok_ = 0;
  bool done_ = false;
  bool csv_open_ = false;
};

} // namespace ros_camera_server_benchmarks

#endif
