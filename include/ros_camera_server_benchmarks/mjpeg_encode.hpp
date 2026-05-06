/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS__MJPEG_ENCODE_HPP_
#define ROS_CAMERA_SERVER_BENCHMARKS__MJPEG_ENCODE_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <jpeglib.h>

namespace ros_camera_server_benchmarks {

// Convert I420 -> JPEG (raw planar input, no extra colour conversion).
//
// Output is 4:2:2 (h_samp=2, v_samp=1 on Y; 1,1 on Cb/Cr) so that downstream
// consumers like usb_cam (`pixel_format: mjpeg2rgb`) — which hardcode
// `av_device_format=YUV422P` and configure their swscale context for 4:2:2
// chroma — can decode the frames without reading past chroma plane bounds
// (decoding 4:2:0 with a 4:2:2-configured swscale segfaults at first frame).
//
// The source frame is I420 (4:2:0), so each chroma row covers two Y rows.
// Feeding libjpeg the duplicated chroma rows produces 4:2:2 output that
// retains the original chroma values without resampling here. Quality is
// fixed at 85 for determinism.
inline bool i420_to_jpeg(const uint8_t* i420, int width, int height,
                         std::vector<uint8_t>& jpeg)
{
  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  unsigned char* outbuf = nullptr;
  unsigned long outsize = 0;
  jpeg_mem_dest(&cinfo, &outbuf, &outsize);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_YCbCr;
  jpeg_set_defaults(&cinfo);
  jpeg_set_colorspace(&cinfo, JCS_YCbCr);
  cinfo.raw_data_in = TRUE;
  jpeg_set_quality(&cinfo, 85, TRUE);
  // 4:2:2 subsampling — chroma half horizontal, full vertical resolution.
  cinfo.comp_info[0].h_samp_factor = 2;
  cinfo.comp_info[0].v_samp_factor = 1;
  cinfo.comp_info[1].h_samp_factor = 1;
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;
  cinfo.comp_info[2].v_samp_factor = 1;

  jpeg_start_compress(&cinfo, TRUE);

  // 4:2:2 MCU height is 8 for both Y and Cb/Cr (since v_samp_factor=1
  // across the board). Process 8 scanlines per call.
  std::vector<JSAMPROW> y_rows(8);
  std::vector<JSAMPROW> cb_rows(8);
  std::vector<JSAMPROW> cr_rows(8);
  JSAMPARRAY planes[3] = { y_rows.data(), cb_rows.data(), cr_rows.data() };

  const uint8_t* y_plane = i420;
  const uint8_t* u_plane = i420 + width * height;
  const uint8_t* v_plane = i420 + width * height + (width / 2) * (height / 2);

  for (int row = 0; row < height; row += 8) {
    for (int i = 0; i < 8; ++i) {
      int yy = std::min(row + i, height - 1);
      y_rows[i] = const_cast<JSAMPROW>(y_plane + yy * width);
      // I420 chroma row index is yy / 2 (each chroma row covers two Y rows).
      int cy = std::min(yy / 2, height / 2 - 1);
      cb_rows[i] = const_cast<JSAMPROW>(u_plane + cy * (width / 2));
      cr_rows[i] = const_cast<JSAMPROW>(v_plane + cy * (width / 2));
    }
    jpeg_write_raw_data(&cinfo, planes, 8);
  }

  jpeg_finish_compress(&cinfo);
  jpeg.assign(outbuf, outbuf + outsize);
  std::free(outbuf);
  jpeg_destroy_compress(&cinfo);
  return true;
}

}  // namespace ros_camera_server_benchmarks

#endif  // ROS_CAMERA_SERVER_BENCHMARKS__MJPEG_ENCODE_HPP_
