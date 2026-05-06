/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/marker.hpp"
#include "ros_camera_server_benchmarks/recv_loop.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ros_camera_server_benchmarks;
using namespace ros_camera_server_benchmarks::marker;

namespace {

// strip_height(640, 16) = 64 px, leaving plenty of room within 480.
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kCell = 16;

// Build a Y-only buffer (the decoder only reads Y) with the marker stamped
// in the top strip. Background Y=128 so the marker squares (16/235) stand out.
std::vector<uint8_t> make_y_plane(uint64_t ts_ns, uint32_t frame_id)
{
  std::vector<uint8_t> y(static_cast<size_t>(kWidth) * kHeight, 128);
  Marker m;
  m.ts_ns = ts_ns;
  m.frame_id = frame_id;
  m.width_hint = static_cast<uint16_t>(kWidth);
  encode_y(y.data(), kWidth, kWidth, kHeight, m, kCell);
  return y;
}

std::string tmp_path(const std::string& name)
{
  auto dir = std::filesystem::temp_directory_path() / "rcs_recv_loop_test";
  std::filesystem::create_directories(dir);
  return (dir / name).string();
}

int count_csv_rows(const std::string& path)
{
  std::ifstream f(path);
  int n = 0;
  std::string line;
  while (std::getline(f, line)) ++n;
  return n - 1;  // header
}

} // namespace

TEST(RecvLoopTest, WarmupSkipsEarlyFrames)
{
  std::string csv = tmp_path("warmup.csv");
  std::filesystem::remove(csv);
  {
    RecvLoop loop({ csv, "test", /*warmup=*/3, /*max=*/0, kCell });
    ASSERT_FALSE(loop.failed());
    for (uint32_t i = 0; i < 5; ++i) {
      auto y = make_y_plane(1000 + i, i);
      EXPECT_TRUE(loop.on_frame(y.data(), kWidth, kWidth, kHeight, 2000 + i));
    }
    EXPECT_EQ(5u, loop.seen());
    EXPECT_EQ(5u, loop.decoded());
  }
  // 5 frames seen, warmup=3 means frames 4 and 5 (1-indexed) are recorded.
  EXPECT_EQ(2, count_csv_rows(csv));
}

TEST(RecvLoopTest, MaxFramesStops)
{
  std::string csv = tmp_path("max.csv");
  std::filesystem::remove(csv);
  RecvLoop loop({ csv, "test", /*warmup=*/0, /*max=*/3, kCell });
  ASSERT_FALSE(loop.failed());
  EXPECT_FALSE(loop.done());

  for (uint32_t i = 0; i < 2; ++i) {
    auto y = make_y_plane(1000 + i, i);
    EXPECT_TRUE(loop.on_frame(y.data(), kWidth, kWidth, kHeight, 2000 + i));
  }
  EXPECT_FALSE(loop.done());

  // Third frame trips the cap; on_frame returns false.
  auto y = make_y_plane(1003, 2);
  EXPECT_FALSE(loop.on_frame(y.data(), kWidth, kWidth, kHeight, 2003));
  EXPECT_TRUE(loop.done());
}

TEST(RecvLoopTest, BadMarkerCountedAsSeenNotDecoded)
{
  std::string csv = tmp_path("bad.csv");
  std::filesystem::remove(csv);
  RecvLoop loop({ csv, "test", /*warmup=*/0, /*max=*/0, kCell });
  ASSERT_FALSE(loop.failed());

  // All-zero buffer has no valid magic / CRC.
  std::vector<uint8_t> garbage(static_cast<size_t>(kWidth) * kHeight, 0);
  EXPECT_TRUE(loop.on_frame(garbage.data(), kWidth, kWidth, kHeight, 1234));
  EXPECT_EQ(1u, loop.seen());
  EXPECT_EQ(0u, loop.decoded());

  auto good = make_y_plane(5555, 1);
  EXPECT_TRUE(loop.on_frame(good.data(), kWidth, kWidth, kHeight, 6666));
  EXPECT_EQ(2u, loop.seen());
  EXPECT_EQ(1u, loop.decoded());
}

TEST(RecvLoopTest, CsvLatencyContent)
{
  std::string csv = tmp_path("content.csv");
  std::filesystem::remove(csv);
  {
    RecvLoop loop({ csv, "test", /*warmup=*/0, /*max=*/0, kCell });
    ASSERT_FALSE(loop.failed());
    auto y = make_y_plane(/*ts_ns=*/1'000'000, /*frame_id=*/42);
    EXPECT_TRUE(loop.on_frame(y.data(), kWidth, kWidth, kHeight,
                              /*recv_ts_ns=*/1'500'000));
  }
  std::ifstream f(csv);
  std::string header, row;
  std::getline(f, header);
  std::getline(f, row);
  EXPECT_EQ("frame_id,send_ts_ns,recv_ts_ns,latency_ns", header);
  EXPECT_EQ("42,1000000,1500000,500000", row);
}

TEST(RecvLoopTest, FailedOnUnopenableCsv)
{
  RecvLoop loop({ "/this/path/does/not/exist/x.csv", "test", 0, 0, kCell });
  EXPECT_TRUE(loop.failed());
}

// The `<csv>.stats` sidecar is the only place silently-corrupted markers
// surface in summary.json — bench_aggregate.py reads it to compute
// decoded_ratio. If this contract breaks, marker regressions become
// invisible because corrupted frames are also absent from the latency CSV.
TEST(RecvLoopTest, WritesStatsSidecar)
{
  std::string csv = tmp_path("sidecar.csv");
  std::string stats = csv + ".stats";
  std::filesystem::remove(csv);
  std::filesystem::remove(stats);
  {
    RecvLoop loop({ csv, "test", /*warmup=*/0, /*max=*/0, kCell });
    ASSERT_FALSE(loop.failed());
    auto good = make_y_plane(123, 1);
    EXPECT_TRUE(loop.on_frame(good.data(), kWidth, kWidth, kHeight, 456));
    std::vector<uint8_t> bad(static_cast<size_t>(kWidth) * kHeight, 0);
    EXPECT_TRUE(loop.on_frame(bad.data(), kWidth, kWidth, kHeight, 789));
  }
  std::ifstream f(stats);
  ASSERT_TRUE(f.good());
  std::string seen_line, decoded_line;
  std::getline(f, seen_line);
  std::getline(f, decoded_line);
  EXPECT_EQ("seen=2", seen_line);
  EXPECT_EQ("decoded=1", decoded_line);
}
