/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/frame_render.hpp"
#include "ros_camera_server_benchmarks/marker.hpp"
#include "ros_camera_server_benchmarks/mjpeg_encode.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using ros_camera_server_benchmarks::frame_render::BaseVideoLoader;
using ros_camera_server_benchmarks::frame_render::VideoFrame;
using ros_camera_server_benchmarks::frame_render::i420_to_rgb24;
using ros_camera_server_benchmarks::frame_render::render_i420;
using ros_camera_server_benchmarks::i420_to_jpeg;
using ros_camera_server_benchmarks::now_ns_monotonic;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_pub");

  std::string topic = node->declare_parameter<std::string>("topic", "/bench/image");
  std::string kind  = node->declare_parameter<std::string>("kind", "raw"); // raw|compressed
  int width = node->declare_parameter<int>("width", 1280);
  int height = node->declare_parameter<int>("height", 720);
  int fps = node->declare_parameter<int>("fps", 30);
  int cell = node->declare_parameter<int>("cell", 16);
  // BENCH_VIDEO support: load high-motion content once and overlay the
  // marker on each frame, so G3 (ros_pub-driven) scenarios see the same
  // encoder workload as G2/G4 (frame_gen-driven). Without this the encoder
  // bitrate budget runs slack on flat-gray frames and G3 numbers are
  // optimistic relative to G2/G4.
  std::string video_path = node->declare_parameter<std::string>("video", "");
  int video_cycle_s = node->declare_parameter<int>("video_cycle_seconds", 5);
  // Sidecar for per-run producer-side encode timing, parallel to frame_gen's
  // --underrun-csv. bench_aggregate.py subtracts encode_total_ns/encode_count
  // from each per-frame latency so YUV vs JPEG comparisons aren't biased by
  // the harness's own format conversion cost.
  std::string encode_stats_path =
      node->declare_parameter<std::string>("encode_stats", "");

  // Match usb_cam's default (reliable, volatile, keep_last). Reliable here
  // is a downstream-compatibility decision: web_video_server and
  // rtsp_image_transport republish subscribe with RELIABLE, so a BE
  // publisher gets a QoS-incompatible match and never delivers a frame
  // (observed in cam-stream-WVS / RTSP runs against usb_cam, which is
  // reliable; same scenarios with a BE ros_pub produced 0 rows). BE
  // subscribers (ros_camera_server, gst_bridge rosimagesrc, ros_recv)
  // accept a reliable publisher fine, so this is the broadest setting.
  rclcpp::QoS qos(10);
  qos.reliable().keep_last(10).durability_volatile();

  auto pub_image = (kind == "raw")
      ? node->create_publisher<sensor_msgs::msg::Image>(topic, qos)
      : nullptr;
  auto pub_compressed = (kind == "compressed")
      ? node->create_publisher<sensor_msgs::msg::CompressedImage>(topic, qos)
      : nullptr;
  if (!pub_image && !pub_compressed) {
    RCLCPP_FATAL(node->get_logger(), "kind must be 'raw' or 'compressed'");
    return 2;
  }

  // Streaming base-video loader. Producer publishes frames as soon as the
  // first one is decoded (one decode latency, not "whole cycle"); see the
  // matching change in frame_gen.cpp for rationale.
  BaseVideoLoader loader(video_path, width, height, fps, video_cycle_s);
  if (!video_path.empty()) {
    if (loader.failed()) {
      RCLCPP_WARN(node->get_logger(),
                  "video='%s' loader failed; falling back to flat gray",
                  video_path.c_str());
    } else if (!loader.wait_first_frame(std::chrono::seconds(10))) {
      RCLCPP_WARN(node->get_logger(),
                  "video='%s' first frame not ready in 10 s; "
                  "falling back to flat gray",
                  video_path.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "base video first frame ready, cycle capacity %zu",
                  loader.cycle_capacity());
    }
  }

  std::vector<uint8_t> i420_buf, rgb_buf, jpeg_buf;
  uint32_t frame_id = 0;
  const auto frame_period =
      std::chrono::nanoseconds(1000000000LL / fps);
  auto next = std::chrono::steady_clock::now();

  uint64_t encode_count = 0;
  uint64_t encode_total_ns = 0;
  uint64_t encode_max_ns = 0;

  while (rclcpp::ok()) {
    uint64_t ts = now_ns_monotonic();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(ts / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(ts % 1000000000ULL);

    const VideoFrame* base = loader.failed() ? nullptr : loader.frame(frame_id);
    render_i420(i420_buf, width, height, frame_id, ts, cell, base);

    uint64_t enc_t0 = now_ns_monotonic();
    if (kind == "raw") {
      i420_to_rgb24(i420_buf.data(), rgb_buf, width, height);
    } else {
      i420_to_jpeg(i420_buf.data(), width, height, jpeg_buf);
    }
    uint64_t enc_dt = now_ns_monotonic() - enc_t0;
    encode_total_ns += enc_dt;
    if (enc_dt > encode_max_ns) encode_max_ns = enc_dt;
    ++encode_count;

    if (kind == "raw") {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "bench";
      msg.width = width;
      msg.height = height;
      msg.encoding = "rgb8";
      msg.is_bigendian = 0;
      msg.step = width * 3;
      msg.data = rgb_buf;
      pub_image->publish(std::move(msg));
    } else {
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "bench";
      msg.format = "jpeg";
      msg.data = jpeg_buf;
      pub_compressed->publish(std::move(msg));
    }

    ++frame_id;
    rclcpp::spin_some(node);
    next += frame_period;
    std::this_thread::sleep_until(next);
  }

  if (!encode_stats_path.empty()) {
    std::ofstream f(encode_stats_path);
    if (f) {
      f << "# ros_pub producer counters\n";
      f << "frames_produced=" << frame_id << "\n";
      f << "encode_count=" << encode_count << "\n";
      f << "encode_total_ns=" << encode_total_ns << "\n";
      f << "encode_max_ns=" << encode_max_ns << "\n";
    } else {
      RCLCPP_WARN(node->get_logger(), "cannot write encode_stats to %s",
                  encode_stats_path.c_str());
    }
  }

  rclcpp::shutdown();
  return 0;
}
