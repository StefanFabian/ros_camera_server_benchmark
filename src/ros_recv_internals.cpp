/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/ros_recv_internals.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

// jpeglib.h declares jpeg_stdio_{src,dest} that take FILE*; pulling in
// <cstdio> first satisfies that even if we never call those entry points.
#include <cstdio>
#include <jpeglib.h>

namespace ros_camera_server_benchmarks::ros_recv
{

bool jpeg_to_y_plane(const uint8_t* jpeg, std::size_t size,
                     std::vector<uint8_t>& y, int& width, int& height,
                     uint64_t& recv_out)
{
  jpeg_decompress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpeg, size);
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }
  cinfo.out_color_space = JCS_GRAYSCALE;
  jpeg_start_decompress(&cinfo);
  width = cinfo.output_width;
  height = cinfo.output_height;
  y.resize(static_cast<std::size_t>(width) * height);
  std::vector<JSAMPROW> rows(height);
  for (int i = 0; i < height; ++i) rows[i] = y.data() + i * width;
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, &rows[cinfo.output_scanline],
                        cinfo.output_height - cinfo.output_scanline);
  }
  jpeg_finish_decompress(&cinfo);
  recv_out = now_ns_monotonic();
  jpeg_destroy_decompress(&cinfo);
  return true;
}

} // namespace ros_camera_server_benchmarks::ros_recv
