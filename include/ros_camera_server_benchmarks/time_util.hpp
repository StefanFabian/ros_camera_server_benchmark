/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS_TIME_UTIL_HPP
#define ROS_CAMERA_SERVER_BENCHMARKS_TIME_UTIL_HPP

#include <cstdint>
#include <ctime>

namespace ros_camera_server_benchmarks
{

inline uint64_t now_ns_monotonic() noexcept
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

} // namespace ros_camera_server_benchmarks

#endif
