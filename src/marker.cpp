/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/marker.hpp"

#include <cstring>

namespace ros_camera_server_benchmarks::marker
{

uint16_t crc16_ccitt_false(const uint8_t* data, int len)
{
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static void pack_payload(const Marker& m, uint8_t out[PAYLOAD_BYTES])
{
  out[0] = MAGIC0;
  out[1] = MAGIC1;
  out[2] = static_cast<uint8_t>(VERSION & 0xFF);
  out[3] = static_cast<uint8_t>((VERSION >> 8) & 0xFF);
  for (int i = 0; i < 8; ++i) {
    out[4 + i] = static_cast<uint8_t>((m.ts_ns >> (8 * i)) & 0xFF);
  }
  for (int i = 0; i < 4; ++i) {
    out[12 + i] = static_cast<uint8_t>((m.frame_id >> (8 * i)) & 0xFF);
  }
  out[16] = static_cast<uint8_t>(m.width_hint & 0xFF);
  out[17] = static_cast<uint8_t>((m.width_hint >> 8) & 0xFF);
  uint16_t crc = crc16_ccitt_false(out, 18);
  out[18] = static_cast<uint8_t>(crc & 0xFF);
  out[19] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

static bool unpack_payload(const uint8_t in[PAYLOAD_BYTES], Marker& out)
{
  if (in[0] != MAGIC0 || in[1] != MAGIC1) return false;
  uint16_t crc_expected = crc16_ccitt_false(in, 18);
  uint16_t crc_got = static_cast<uint16_t>(in[18]) |
                     static_cast<uint16_t>(in[19]) << 8;
  if (crc_expected != crc_got) return false;
  uint16_t version = static_cast<uint16_t>(in[2]) |
                     static_cast<uint16_t>(in[3]) << 8;
  if (version != VERSION) return false;
  out.ts_ns = 0;
  for (int i = 0; i < 8; ++i) {
    out.ts_ns |= static_cast<uint64_t>(in[4 + i]) << (8 * i);
  }
  out.frame_id = 0;
  for (int i = 0; i < 4; ++i) {
    out.frame_id |= static_cast<uint32_t>(in[12 + i]) << (8 * i);
  }
  out.width_hint = static_cast<uint16_t>(in[16]) |
                   static_cast<uint16_t>(in[17]) << 8;
  return true;
}

// Bit -> (cell_x, cell_y) layout: left to right across cells_per_row cells per
// row, then wrap to next row. Row 0 holds bits [0, cells_per_row),
// row 1 holds [cells_per_row, 2 * cells_per_row), etc.
static void bit_position(int bit_index, int cells_per_row, int& cx, int& cy)
{
  cx = bit_index % cells_per_row;
  cy = bit_index / cells_per_row;
}

int strip_height(int width, int cell)
{
  int cells_per_row = width / cell;
  if (cells_per_row <= 0) return 0;
  int rows = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  return rows * cell;
}

void encode_y(uint8_t* y_plane, int stride, int width, int height,
              const Marker& m, int cell)
{
  uint8_t payload[PAYLOAD_BYTES];
  pack_payload(m, payload);

  int cells_per_row = width / cell;
  int rows_needed = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  if (cells_per_row <= 0 || rows_needed * cell > height) return;

  // Clear strip to black (predictable reference for cells we don't paint).
  for (int y = 0; y < rows_needed * cell; ++y) {
    std::memset(y_plane + y * stride, Y_BLACK, width);
  }

  for (int bit = 0; bit < PAYLOAD_BITS; ++bit) {
    int byte = bit / 8;
    int b = bit % 8;
    int v = (payload[byte] >> b) & 1;
    int cx, cy;
    bit_position(bit, cells_per_row, cx, cy);
    int x0 = cx * cell;
    int y0 = cy * cell;
    uint8_t intensity = v ? Y_WHITE : Y_BLACK;
    for (int dy = 0; dy < cell; ++dy) {
      uint8_t* row = y_plane + (y0 + dy) * stride + x0;
      std::memset(row, intensity, cell);
    }
  }
}

void clear_uv_strip_i420(uint8_t* u_plane, uint8_t* v_plane, int uv_stride,
                         int width, int height, int cell)
{
  int cells_per_row = width / cell;
  int rows_needed = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  if (cells_per_row <= 0 || rows_needed * cell > height) return;

  // Y strip covers `rows_needed * cell` rows; 4:2:0 chroma is sub-sampled
  // 2:1 vertically, so cover ceil(strip_rows / 2) chroma rows.
  int y_strip_rows = rows_needed * cell;
  int uv_strip_rows = (y_strip_rows + 1) / 2;
  int uv_w = width / 2;
  for (int yy = 0; yy < uv_strip_rows; ++yy) {
    std::memset(u_plane + yy * uv_stride, 128, uv_w);
    std::memset(v_plane + yy * uv_stride, 128, uv_w);
  }
}

void encode_rgb(uint8_t* rgb, int stride, int width, int height,
                const Marker& m, int cell)
{
  uint8_t payload[PAYLOAD_BYTES];
  pack_payload(m, payload);

  int cells_per_row = width / cell;
  int rows_needed = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  if (cells_per_row <= 0 || rows_needed * cell > height) return;

  for (int y = 0; y < rows_needed * cell; ++y) {
    uint8_t* row = rgb + y * stride;
    for (int x = 0; x < width; ++x) {
      row[3 * x + 0] = Y_BLACK;
      row[3 * x + 1] = Y_BLACK;
      row[3 * x + 2] = Y_BLACK;
    }
  }

  for (int bit = 0; bit < PAYLOAD_BITS; ++bit) {
    int byte = bit / 8;
    int b = bit % 8;
    int v = (payload[byte] >> b) & 1;
    int cx, cy;
    bit_position(bit, cells_per_row, cx, cy);
    int x0 = cx * cell;
    int y0 = cy * cell;
    uint8_t intensity = v ? Y_WHITE : Y_BLACK;
    for (int dy = 0; dy < cell; ++dy) {
      uint8_t* row = rgb + (y0 + dy) * stride + 3 * x0;
      for (int dx = 0; dx < cell; ++dx) {
        row[3 * dx + 0] = intensity;
        row[3 * dx + 1] = intensity;
        row[3 * dx + 2] = intensity;
      }
    }
  }
}

bool decode_y(const uint8_t* y_plane, int stride, int width, int height,
              Marker& out, int cell)
{
  int cells_per_row = width / cell;
  int rows_needed = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  if (cells_per_row <= 0 || rows_needed * cell > height) return false;

  uint8_t payload[PAYLOAD_BYTES];
  std::memset(payload, 0, sizeof(payload));

  for (int bit = 0; bit < PAYLOAD_BITS; ++bit) {
    int cx, cy;
    bit_position(bit, cells_per_row, cx, cy);
    int sx = cx * cell + cell / 2;
    int sy = cy * cell + cell / 2;
    uint8_t sample = y_plane[sy * stride + sx];
    int v = sample >= ((Y_BLACK + Y_WHITE) / 2) ? 1 : 0;
    int byte = bit / 8;
    int b = bit % 8;
    payload[byte] |= static_cast<uint8_t>(v << b);
  }

  return unpack_payload(payload, out);
}

bool decode_rgb(const uint8_t* rgb, int stride, int width, int height,
                Marker& out, int cell)
{
  int cells_per_row = width / cell;
  int rows_needed = (PAYLOAD_BITS + cells_per_row - 1) / cells_per_row;
  if (cells_per_row <= 0 || rows_needed * cell > height) return false;

  uint8_t payload[PAYLOAD_BYTES];
  std::memset(payload, 0, sizeof(payload));

  for (int bit = 0; bit < PAYLOAD_BITS; ++bit) {
    int cx, cy;
    bit_position(bit, cells_per_row, cx, cy);
    int sx = cx * cell + cell / 2;
    int sy = cy * cell + cell / 2;
    const uint8_t* p = rgb + sy * stride + 3 * sx;
    int avg = (p[0] + p[1] + p[2]) / 3;
    int v = avg >= ((Y_BLACK + Y_WHITE) / 2) ? 1 : 0;
    int byte = bit / 8;
    int b = bit % 8;
    payload[byte] |= static_cast<uint8_t>(v << b);
  }

  return unpack_payload(payload, out);
}

} // namespace ros_camera_server_benchmarks::marker
