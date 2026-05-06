/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#ifndef ROS_CAMERA_SERVER_BENCHMARKS_MARKER_HPP
#define ROS_CAMERA_SERVER_BENCHMARKS_MARKER_HPP

#include <cstdint>

namespace ros_camera_server_benchmarks::marker
{

// Cell size 16 (default) is load-bearing: it aligns with H.264's 16x16
// macroblock partitioning so each cell's center sample falls inside a single
// macroblock and survives quantization at the bitrates we benchmark
// (~4 Mbps). It also matches the JPEG MCU height we use in frame_gen, so the
// 16-px cell is also aligned with libjpeg's 16-row processing band. Do not
// change `cell` casually; if you do, re-run `MarkerRoundtrip.H264Roundtrip*`
// and `MarkerRoundtrip.JpegRoundtrip*` to confirm the new value still
// recovers cleanly through both encoders.
//
// Wire format (20 bytes, 160 bits):
//   0..1   magic 0xB3 0x57
//   2..3   version 0x0001 LE
//   4..11  ts_ns (LE u64, CLOCK_MONOTONIC)
//   12..15 frame_id (LE u32)
//   16..17 width_hint (LE u16)
//   18..19 CRC-16/CCITT-FALSE over bytes 0..17
constexpr int PAYLOAD_BYTES = 20;
constexpr int PAYLOAD_BITS = PAYLOAD_BYTES * 8;
constexpr uint8_t MAGIC0 = 0xB3;
constexpr uint8_t MAGIC1 = 0x57;
constexpr uint16_t VERSION = 0x0001;

// Pixel intensities (BT.601 limited range Y).
constexpr uint8_t Y_BLACK = 16;
constexpr uint8_t Y_WHITE = 235;

struct Marker {
  uint64_t ts_ns = 0;
  uint32_t frame_id = 0;
  uint16_t width_hint = 0;
};

uint16_t crc16_ccitt_false(const uint8_t* data, int len);

// Returns the height (in pixel rows) the marker strip occupies for a given
// width and cell size. Caller-supplied buffers must have at least this many
// rows free at the top of the frame.
int strip_height(int width, int cell);

// Encode marker into the Y plane of a YUV image (or the Y plane of any
// luma-first format). Cells are written as Y_BLACK / Y_WHITE squares.
void encode_y(uint8_t* y_plane, int stride, int width, int height,
              const Marker& m, int cell = 16);

// Force U=V=128 in the rows that the marker strip covers, so the strip is
// pure greyscale regardless of underlying chroma. Required when the carrier
// is a YUV format that downstream consumers convert to RGB and then estimate
// luma as (R+G+B)/3 (ros_recv on rgb8). Without this, heavy chroma in the
// strip rows shifts the reconstructed "luma" enough to flip cell bits.
// `uv_stride` is the byte stride of one chroma plane; for I420 at width W
// that is W/2. `width` and `height` are full Y-plane dimensions.
void clear_uv_strip_i420(uint8_t* u_plane, uint8_t* v_plane, int uv_stride,
                         int width, int height, int cell = 16);

// Encode marker into a packed RGB (3 byte/pixel) image. R=G=B=Y_BLACK or Y_WHITE.
void encode_rgb(uint8_t* rgb, int stride, int width, int height,
                const Marker& m, int cell = 16);

// Decode marker from Y plane. Returns true iff magic and CRC match.
bool decode_y(const uint8_t* y_plane, int stride, int width, int height,
              Marker& out, int cell = 16);

// Decode marker from packed RGB. Computes Y = (R+G+B)/3 at sample point.
bool decode_rgb(const uint8_t* rgb, int stride, int width, int height,
                Marker& out, int cell = 16);

} // namespace ros_camera_server_benchmarks::marker

#endif
