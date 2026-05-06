/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 *
 *  Regression test for A6 (usb_cam mjpeg2rgb on /dev/video11).
 *
 *  usb_cam's `MJPEG2RGB` decoder defaults `av_device_format=YUV422P` and
 *  configures its swscale context for 4:2:2 chroma plane layout. If
 *  frame_gen produces 4:2:0 JPEG, the decoded frame's chroma planes are
 *  half the height swscale expects, and `sws_scale` reads past the end of
 *  the chroma planes — SIGSEGV at the first frame.
 *
 *  This test asserts that the JPEG bytes produced by `i420_to_jpeg` carry
 *  4:2:2 sampling factors in the SOF0 marker, so a future change to the
 *  encoder that drops back to 4:2:0 will be caught here instead of
 *  manifesting as a crash inside usb_cam during a benchmark run.
 */

#include "ros_camera_server_benchmarks/mjpeg_encode.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <jpeglib.h>

namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 48;

// Build a flat-gray I420 buffer just large enough to encode.
std::vector<uint8_t> make_i420_gray()
{
  const int y_size = kWidth * kHeight;
  const int uv_size = (kWidth / 2) * (kHeight / 2);
  std::vector<uint8_t> buf(y_size + 2 * uv_size, 128);
  return buf;
}

}  // namespace

TEST(MjpegEncode, ProducesYuv422Subsampling)
{
  std::vector<uint8_t> i420 = make_i420_gray();
  std::vector<uint8_t> jpeg;
  ASSERT_TRUE(ros_camera_server_benchmarks::i420_to_jpeg(
      i420.data(), kWidth, kHeight, jpeg));
  ASSERT_GT(jpeg.size(), 100u);

  jpeg_decompress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpeg.data(), jpeg.size());
  ASSERT_EQ(jpeg_read_header(&cinfo, TRUE), JPEG_HEADER_OK);

  EXPECT_EQ(cinfo.image_width, static_cast<JDIMENSION>(kWidth));
  EXPECT_EQ(cinfo.image_height, static_cast<JDIMENSION>(kHeight));
  ASSERT_EQ(cinfo.num_components, 3);

  // 4:2:2: Y has h_samp=2, v_samp=1; Cb/Cr have h_samp=1, v_samp=1.
  // 4:2:0 (the broken setting) would have Y v_samp=2 — would crash usb_cam.
  EXPECT_EQ(cinfo.comp_info[0].h_samp_factor, 2);
  EXPECT_EQ(cinfo.comp_info[0].v_samp_factor, 1)
      << "v_samp_factor=2 means 4:2:0; usb_cam mjpeg2rgb assumes 4:2:2 and "
         "will SIGSEGV on the first decoded frame (A6 regression).";
  EXPECT_EQ(cinfo.comp_info[1].h_samp_factor, 1);
  EXPECT_EQ(cinfo.comp_info[1].v_samp_factor, 1);
  EXPECT_EQ(cinfo.comp_info[2].h_samp_factor, 1);
  EXPECT_EQ(cinfo.comp_info[2].v_samp_factor, 1);

  jpeg_destroy_decompress(&cinfo);
}

TEST(MjpegEncode, RoundTripDecodes)
{
  std::vector<uint8_t> i420 = make_i420_gray();
  std::vector<uint8_t> jpeg;
  ASSERT_TRUE(ros_camera_server_benchmarks::i420_to_jpeg(
      i420.data(), kWidth, kHeight, jpeg));

  jpeg_decompress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpeg.data(), jpeg.size());
  ASSERT_EQ(jpeg_read_header(&cinfo, TRUE), JPEG_HEADER_OK);
  cinfo.out_color_space = JCS_RGB;
  jpeg_start_decompress(&cinfo);

  std::vector<uint8_t> rgb_row(static_cast<size_t>(cinfo.output_width) * 3);
  uint8_t* rows[1] = { rgb_row.data() };
  int rows_decoded = 0;
  while (cinfo.output_scanline < cinfo.output_height) {
    int n = jpeg_read_scanlines(&cinfo, rows, 1);
    ASSERT_EQ(n, 1);
    rows_decoded += n;
  }
  EXPECT_EQ(rows_decoded, kHeight);

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
}
