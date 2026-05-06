/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS_ROS_RECV_INTERNALS_HPP
#define ROS_CAMERA_SERVER_BENCHMARKS_ROS_RECV_INTERNALS_HPP

#include "ros_camera_server_benchmarks/time_util.hpp"

#include <cstdint>
#include <vector>

namespace ros_camera_server_benchmarks::ros_recv
{

// Decode a JPEG buffer to a Y plane (display-prep cost). Stamps `recv_out`
// immediately after jpeg_finish_decompress so the JPEG decode itself counts
// as part of measured latency, while subsequent marker-extraction overhead
// (decode_y on the returned Y plane) stays excluded. The recv_ts ordering
// is exercised by `MarkerRoundtrip.JpegRecvTimestampOrdering`; do not move
// the now_ns_monotonic call without updating that test.
bool jpeg_to_y_plane(const uint8_t* jpeg, std::size_t size,
                     std::vector<uint8_t>& y, int& width, int& height,
                     uint64_t& recv_out);

} // namespace ros_camera_server_benchmarks::ros_recv

#endif
