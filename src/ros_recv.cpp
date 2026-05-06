/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian.
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/recv_loop.hpp"
#include "ros_camera_server_benchmarks/ros_recv_internals.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using ros_camera_server_benchmarks::RecvLoop;
using ros_camera_server_benchmarks::RecvLoopConfig;
using ros_camera_server_benchmarks::now_ns_monotonic;
using ros_camera_server_benchmarks::ros_recv::jpeg_to_y_plane;

namespace {

bool rgb_to_y_plane(const sensor_msgs::msg::Image& msg,
                    std::vector<uint8_t>& y_plane, int& width, int& height)
{
  width = msg.width;
  height = msg.height;
  y_plane.resize(static_cast<size_t>(width) * height);
  if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
    for (int yy = 0; yy < height; ++yy) {
      const uint8_t* row = msg.data.data() + yy * msg.step;
      uint8_t* out = y_plane.data() + yy * width;
      for (int x = 0; x < width; ++x) {
        // approx luma; sufficient since marker is black/white.
        out[x] = static_cast<uint8_t>((row[3 * x] + row[3 * x + 1] +
                                       row[3 * x + 2]) / 3);
      }
    }
    return true;
  }
  if (msg.encoding == "mono8") {
    for (int yy = 0; yy < height; ++yy) {
      std::memcpy(y_plane.data() + yy * width,
                  msg.data.data() + yy * msg.step, width);
    }
    return true;
  }
  return false;
}

} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros_recv");

  std::string topic = node->declare_parameter<std::string>("topic", "/bench/image");
  std::string kind = node->declare_parameter<std::string>("kind", "raw");
  std::string csv_path = node->declare_parameter<std::string>("csv", "ros_recv.csv");
  int warmup = node->declare_parameter<int>("warmup", 150);
  int max_frames = node->declare_parameter<int>("max_frames", 750);
  int cell = node->declare_parameter<int>("cell", 16);

  RecvLoop recv_loop({ csv_path, "ros_recv",
                       static_cast<uint32_t>(warmup),
                       static_cast<uint32_t>(max_frames), cell });
  if (recv_loop.failed()) {
    RCLCPP_FATAL(node->get_logger(), "cannot open %s", csv_path.c_str());
    return 1;
  }

  // Match the QoS image_transport publishers use by default: best-effort,
  // volatile, keep_last(10). usb_cam publishes with sensor_data QoS
  // (best-effort), so a reliable subscriber would never receive its frames
  // (RELIABLE sub + BEST_EFFORT pub = incompatible). Best-effort here also
  // keeps gscam / ros_camera_server / ros_pub frames flowing — those publishers default
  // to reliable, and a best-effort subscriber on a reliable publisher is a
  // compatible downgrade.
  rclcpp::QoS qos(10);
  qos.best_effort().keep_last(10).durability_volatile();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed;

  if (kind == "raw") {
    sub_image = node->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        [&recv_loop](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          // Raw rgb8/mono8 needs no decode to be display-ready: the message
          // arrives in a presentable format. Stamp recv at message arrival,
          // before the rgb_to_y_plane pass that exists only to recover the
          // marker.
          uint64_t recv = now_ns_monotonic();
          std::vector<uint8_t> y;
          int w = 0, h = 0;
          if (!rgb_to_y_plane(*msg, y, w, h)) return;
          // rgb_to_y_plane writes contiguous rows of `w` bytes each, so the
          // Y-plane stride equals the width.
          if (!recv_loop.on_frame(y.data(), w, w, h, recv)) {
            rclcpp::shutdown();
          }
        });
  } else if (kind == "compressed") {
    sub_compressed = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, qos,
        [&recv_loop](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          // CompressedImage requires JPEG decode to display. jpeg_to_y_plane
          // stamps recv right after jpeg_finish_decompress so the decode is
          // counted as part of measured latency but the subsequent marker
          // extraction is not.
          std::vector<uint8_t> y;
          int w = 0, h = 0;
          uint64_t recv = 0;
          if (!jpeg_to_y_plane(msg->data.data(), msg->data.size(), y, w, h, recv))
            return;
          if (!recv_loop.on_frame(y.data(), w, w, h, recv)) {
            rclcpp::shutdown();
          }
        });
  } else {
    RCLCPP_FATAL(node->get_logger(), "kind must be raw or compressed");
    return 2;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
