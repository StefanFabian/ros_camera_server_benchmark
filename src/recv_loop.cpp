/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/recv_loop.hpp"

#include "ros_camera_server_benchmarks/marker.hpp"

#include <cstdio>

namespace ros_camera_server_benchmarks
{

using namespace ros_camera_server_benchmarks::marker;

RecvLoop::RecvLoop(const RecvLoopConfig& cfg) : cfg_(cfg)
{
  csv_.open(cfg_.csv_path);
  if (!csv_) {
    std::fprintf(stderr, "%s: cannot open csv: %s\n",
                 cfg_.label.c_str(), cfg_.csv_path.c_str());
    return;
  }
  csv_ << "frame_id,send_ts_ns,recv_ts_ns,latency_ns\n";
  csv_open_ = true;
}

RecvLoop::~RecvLoop()
{
  std::fprintf(stderr, "%s: seen=%lu decoded=%lu csv=%s\n",
               cfg_.label.c_str(),
               static_cast<unsigned long>(seen_),
               static_cast<unsigned long>(decoded_ok_),
               cfg_.csv_path.c_str());
  // Sidecar stats file next to the CSV (e.g. stream.csv -> stream.stats).
  // bench_aggregate.py reads it to surface the decoded/seen ratio in
  // summary.json — silent marker-decode failures otherwise vanish into
  // the latency percentiles.
  std::string stats_path = cfg_.csv_path + ".stats";
  std::ofstream stats(stats_path);
  if (stats) {
    stats << "seen=" << seen_ << "\n";
    stats << "decoded=" << decoded_ok_ << "\n";
  }
}

bool RecvLoop::on_frame(const uint8_t* y_plane, int stride, int width,
                        int height, uint64_t recv_ts_ns)
{
  Marker m;
  bool ok = decode_y(y_plane, stride, width, height, m, cfg_.cell);

  std::lock_guard<std::mutex> lock(mtx_);
  ++seen_;
  if (ok) {
    ++decoded_ok_;
    if (seen_ > cfg_.warmup_frames && csv_open_) {
      int64_t latency_ns = static_cast<int64_t>(recv_ts_ns) -
                           static_cast<int64_t>(m.ts_ns);
      csv_ << m.frame_id << ',' << m.ts_ns << ',' << recv_ts_ns << ','
           << latency_ns << '\n';
    }
  }
  if (cfg_.max_frames > 0 &&
      seen_ >= static_cast<uint64_t>(cfg_.max_frames) + cfg_.warmup_frames) {
    done_ = true;
  }
  return !done_;
}

bool RecvLoop::done() const noexcept
{
  std::lock_guard<std::mutex> lock(mtx_);
  return done_;
}

uint64_t RecvLoop::seen() const noexcept
{
  std::lock_guard<std::mutex> lock(mtx_);
  return seen_;
}

uint64_t RecvLoop::decoded() const noexcept
{
  std::lock_guard<std::mutex> lock(mtx_);
  return decoded_ok_;
}

bool RecvLoop::failed() const noexcept { return !csv_open_; }

} // namespace ros_camera_server_benchmarks
