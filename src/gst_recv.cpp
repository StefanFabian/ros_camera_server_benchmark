/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/recv_loop.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

using ros_camera_server_benchmarks::RecvLoop;
using ros_camera_server_benchmarks::RecvLoopConfig;
using ros_camera_server_benchmarks::now_ns_monotonic;

namespace {

std::atomic<bool> g_stop{ false };
void on_signal(int) { g_stop = true; }

struct Ctx {
  RecvLoop* loop_state = nullptr;
  GMainLoop* loop = nullptr;
};

GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data)
{
  auto* ctx = static_cast<Ctx*>(user_data);
  GstSample* sample = gst_app_sink_pull_sample(sink);
  if (!sample) return GST_FLOW_ERROR;

  GstCaps* caps = gst_sample_get_caps(sample);
  GstVideoInfo vinfo;
  gst_video_info_init(&vinfo);
  if (!gst_video_info_from_caps(&vinfo, caps)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }
  int width = GST_VIDEO_INFO_WIDTH(&vinfo);
  int height = GST_VIDEO_INFO_HEIGHT(&vinfo);
  int y_stride = GST_VIDEO_INFO_PLANE_STRIDE(&vinfo, 0);

  GstBuffer* buf = gst_sample_get_buffer(sample);
  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &vinfo, buf, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }
  const uint8_t* y_plane =
      static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));

  // recv stamped after appsink delivers a decoded I420 frame (display-ready)
  // but before marker bit extraction. Pinned by
  // MarkerRoundtrip.JpegRecvTimestampOrdering.
  uint64_t recv = now_ns_monotonic();
  if (!ctx->loop_state->on_frame(y_plane, y_stride, width, height, recv)) {
    g_main_loop_quit(ctx->loop);
  }

  gst_video_frame_unmap(&frame);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

gboolean on_bus(GstBus*, GstMessage* msg, gpointer user_data)
{
  auto* ctx = static_cast<Ctx*>(user_data);
  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    GError* err = nullptr;
    gchar* dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    std::fprintf(stderr, "GST error: %s (%s)\n", err ? err->message : "?",
                 dbg ? dbg : "");
    if (err) g_error_free(err);
    g_free(dbg);
    g_main_loop_quit(ctx->loop);
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
    g_main_loop_quit(ctx->loop);
  }
  return TRUE;
}

void usage()
{
  std::cerr << "Usage: gst_recv --mode MODE --uri URI --csv FILE "
               "[--warmup-frames 150] [--max-frames 750] [--cell 16]\n"
               "Modes:\n"
               "  udp-rtp          udp://host:port -> udpsrc -> rtph264depay -> ...\n"
               "  http-mjpeg       http://host:port/... -> souphttpsrc ! "
               "multipartdemux ! decodebin\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string mode;
  std::string uri;
  std::string csv_path = "gst_recv.csv";
  int warmup = 150;
  int max_frames = 750;
  int cell = 16;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* n) -> const char* {
      if (i + 1 >= argc) { std::cerr << n << " requires value\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--mode") mode = next("--mode");
    else if (a == "--uri") uri = next("--uri");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--warmup-frames") warmup = std::atoi(next("--warmup-frames"));
    else if (a == "--max-frames") max_frames = std::atoi(next("--max-frames"));
    else if (a == "--cell") cell = std::atoi(next("--cell"));
    else if (a == "--help" || a == "-h") { usage(); return 0; }
    else { std::cerr << "unknown: " << a << "\n"; usage(); return 2; }
  }
  if (mode.empty() || uri.empty()) { usage(); return 2; }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  gst_init(&argc, &argv);

  // Pipeline tail is shared across all transports: decoded I420 -> appsink.
  // Decoder is auto-plugged via decodebin; pin via GST_PLUGIN_FEATURE_RANK
  // or env_fairness.sh as needed.
  const char* tail =
      "videoconvert ! video/x-raw,format=I420 ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false";

  char desc[1536];
  if (mode == "udp-rtp") {
    // UDP-RTP receive: no jitter buffer, no retransmit window. Reports the
    // raw encode + 1-frame pipeline buffering + decode cost. udpsrc listens
    // on the URI's port; sender (rcs's rtp output, or start_floor_pipeline
    // in run_scenario.sh) connects with udpsink.
    auto colon = uri.rfind(':');
    int port = colon == std::string::npos ? 0
                                          : std::atoi(uri.c_str() + colon + 1);
    if (port <= 0) {
      std::fprintf(stderr, "udp-rtp: bad port in uri '%s'\n", uri.c_str());
      return 2;
    }
    std::snprintf(desc, sizeof(desc),
                  "udpsrc port=%d caps=\"application/x-rtp,media=video,"
                  "clock-rate=90000,encoding-name=H264,payload=96\" ! "
                  "rtph264depay ! h264parse ! decodebin ! %s",
                  port, tail);
  } else if (mode == "http-mjpeg") {
    // web_video_server's mjpeg streamer wraps each frame in a multipart
    // boundary; multipartdemux + jpegparse + decodebin recovers the raw I420.
    // decodebin auto-plugs vajpegdec / nvjpegdec when present (pinned by
    // env_fairness.sh's GST_PLUGIN_FEATURE_RANK to the selected family) and
    // falls back to libjpeg-turbo `jpegdec` on hosts without a HW JPEG
    // decoder. Symmetric with the H.264 paths, which also use decodebin.
    std::snprintf(desc, sizeof(desc),
                  "souphttpsrc location=%s is-live=true ! "
                  "multipartdemux ! image/jpeg ! jpegparse ! decodebin ! %s",
                  uri.c_str(), tail);
  } else {
    std::cerr << "unknown --mode: " << mode << "\n";
    usage();
    return 2;
  }

  GError* err = nullptr;
  GstElement* pipeline = gst_parse_launch(desc, &err);
  if (!pipeline) {
    std::fprintf(stderr, "gst_parse_launch: %s\npipeline: %s\n",
                 err ? err->message : "unknown", desc);
    if (err) g_error_free(err);
    return 1;
  }

  RecvLoop recv_loop({ csv_path, "gst_recv[" + mode + "]",
                       static_cast<uint32_t>(warmup),
                       static_cast<uint32_t>(max_frames), cell });
  if (recv_loop.failed()) return 1;

  Ctx ctx;
  ctx.loop_state = &recv_loop;
  ctx.loop = g_main_loop_new(nullptr, FALSE);

  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  GstAppSinkCallbacks cbs = {};
  cbs.new_sample = on_new_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, &ctx, nullptr);

  GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, on_bus, &ctx);
  gst_object_unref(bus);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  guint timeout_id = g_timeout_add(200, [](gpointer data) -> gboolean {
    auto* c = static_cast<Ctx*>(data);
    if (g_stop.load()) { g_main_loop_quit(c->loop); return G_SOURCE_REMOVE; }
    return G_SOURCE_CONTINUE;
  }, &ctx);
  (void)timeout_id;

  g_main_loop_run(ctx.loop);

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(sink);
  gst_object_unref(pipeline);
  g_main_loop_unref(ctx.loop);

  return 0;
}
