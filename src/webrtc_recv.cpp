/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 *
 *  WebRTC client receiver. Connects to ros_camera_server's signaling
 *  endpoint at ws://host:port/{camera_id}/{output_index}, receives the
 *  server-initiated SDP offer, replies with an answer, and decodes the
 *  resulting H.264 RTP stream via webrtcbin. Marker recovery and CSV
 *  output mirror gst_recv.
 *
 *  Wire format (matches ros_camera_server/src/webrtc/webrtc_peer.cpp):
 *    {"type":"offer|answer","sdp":"<sdp text>"}
 *    {"ice":{"candidate":"<cand>","sdpMLineIndex":<n>}}
 */

#include "ros_camera_server_benchmarks/recv_loop.hpp"
#include "ros_camera_server_benchmarks/time_util.hpp"

// gst-plugins-bad WebRTC headers warn loudly about being unstable API.
// We're aware; the same headers ship in ros_camera_server's webrtc/.
#define GST_USE_UNSTABLE_API
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/video/video.h>
#include <gst/webrtc/webrtc.h>
#include <libsoup/soup.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>

using nlohmann::json;
using ros_camera_server_benchmarks::RecvLoop;
using ros_camera_server_benchmarks::RecvLoopConfig;
using ros_camera_server_benchmarks::now_ns_monotonic;

namespace {

std::atomic<bool> g_stop{ false };
void on_signal(int) { g_stop = true; }

struct Ctx {
  RecvLoop* loop_state = nullptr;
  GMainLoop* loop = nullptr;

  // GStreamer pipeline + webrtc.
  GstElement* pipeline = nullptr;
  GstElement* webrtc = nullptr;

  // Signaling state.
  SoupSession* session = nullptr;
  SoupWebsocketConnection* ws = nullptr;

  // Connection retry: ros_camera_server's signaling server may not be up
  // when the receiver starts. Without retry, "Connection refused" before
  // ros_camera_server binds aborts the run.
  std::string ws_uri;
  int connect_retries_remaining = 0;
};

void start_ws_connect(Ctx* ctx);  // forward decl

// =============================================================================
// Appsink callback: identical to gst_recv. recv_ts captured before
// decode_y, after appsink delivers a decoded I420 frame.
// =============================================================================

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

  uint64_t recv = now_ns_monotonic();
  if (!ctx->loop_state->on_frame(y_plane, y_stride, width, height, recv)) {
    g_main_loop_quit(ctx->loop);
  }

  gst_video_frame_unmap(&frame);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

// =============================================================================
// Signaling helpers (JSON over libsoup-3 websocket).
// =============================================================================

void send_json(Ctx* ctx, const json& msg)
{
  if (!ctx->ws ||
      soup_websocket_connection_get_state(ctx->ws) != SOUP_WEBSOCKET_STATE_OPEN) {
    return;
  }
  std::string text = msg.dump();
  soup_websocket_connection_send_text(ctx->ws, text.c_str());
}

// =============================================================================
// webrtcbin signal handlers.
// =============================================================================

void on_ice_candidate(GstElement*, guint mline, gchar* candidate, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  json msg;
  msg["ice"] = { { "candidate", candidate ? candidate : "" },
                 { "sdpMLineIndex", mline } };
  send_json(ctx, msg);
}

// decodebin is dynamic (it builds the decoder graph at caps-negotiation
// time), so we can't pre-link it to a static videoconvert tail. The
// pad-added on decodebin fires once the H.264 stream is fully parsed and
// the decoder is selected; at that point we wire decodebin's src pad to a
// static videoconvert -> I420 capsfilter -> appsink tail.
void on_decodebin_pad_added(GstElement*, GstPad* pad, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  GstCaps* caps = gst_pad_get_current_caps(pad);
  if (!caps) caps = gst_pad_query_caps(pad, nullptr);
  if (!caps) return;
  const GstStructure* s = gst_caps_get_structure(caps, 0);
  bool is_video = s && g_str_has_prefix(gst_structure_get_name(s), "video/");
  gst_caps_unref(caps);
  if (!is_video) return;

  GstElement* conv = gst_bin_get_by_name(GST_BIN(ctx->pipeline), "wrtc_conv");
  if (!conv) return;
  GstPad* sinkpad = gst_element_get_static_pad(conv, "sink");
  if (!gst_pad_is_linked(sinkpad)) {
    if (gst_pad_link(pad, sinkpad) != GST_PAD_LINK_OK) {
      std::fprintf(stderr, "webrtc_recv: decodebin->videoconvert link failed\n");
    }
  }
  gst_object_unref(sinkpad);
  gst_object_unref(conv);
}

void on_pad_added(GstElement*, GstPad* pad, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  if (GST_PAD_DIRECTION(pad) != GST_PAD_SRC) return;

  // depay -> h264parse -> decodebin -> videoconvert -> I420 -> appsink.
  // Matches gst_recv's tail so the WebRTC arm decodes through the same
  // VA-API decoder env_fairness.sh ranks first; previously this chain
  // hard-coded avdec_h264 and so measured software decode while every
  // other arm measured VA-API.
  GstElement* depay = gst_element_factory_make("rtph264depay", nullptr);
  GstElement* parse = gst_element_factory_make("h264parse", nullptr);
  GstElement* dec = gst_element_factory_make("decodebin", nullptr);
  GstElement* conv = gst_element_factory_make("videoconvert", "wrtc_conv");
  GstElement* capsf = gst_element_factory_make("capsfilter", nullptr);
  GstElement* sink = gst_element_factory_make("appsink", "sink");
  if (!depay || !parse || !dec || !conv || !capsf || !sink) {
    std::fprintf(stderr, "webrtc_recv: failed to create decode elements\n");
    return;
  }

  GstCaps* caps = gst_caps_from_string("video/x-raw,format=I420");
  g_object_set(capsf, "caps", caps, nullptr);
  gst_caps_unref(caps);

  g_object_set(sink, "emit-signals", TRUE, "sync", FALSE, "max-buffers", 4,
               "drop", FALSE, nullptr);
  GstAppSinkCallbacks cbs = {};
  cbs.new_sample = on_new_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, ctx, nullptr);

  g_signal_connect(dec, "pad-added", G_CALLBACK(on_decodebin_pad_added), ctx);

  gst_bin_add_many(GST_BIN(ctx->pipeline), depay, parse, dec, conv, capsf, sink,
                   nullptr);
  if (!gst_element_link_many(depay, parse, dec, nullptr)) {
    std::fprintf(stderr, "webrtc_recv: depay->parse->decodebin link failed\n");
    return;
  }
  if (!gst_element_link_many(conv, capsf, sink, nullptr)) {
    std::fprintf(stderr, "webrtc_recv: conv->caps->sink link failed\n");
    return;
  }
  gst_element_sync_state_with_parent(depay);
  gst_element_sync_state_with_parent(parse);
  gst_element_sync_state_with_parent(dec);
  gst_element_sync_state_with_parent(conv);
  gst_element_sync_state_with_parent(capsf);
  gst_element_sync_state_with_parent(sink);

  GstPad* sinkpad = gst_element_get_static_pad(depay, "sink");
  if (gst_pad_link(pad, sinkpad) != GST_PAD_LINK_OK) {
    std::fprintf(stderr, "webrtc_recv: pad-link failed\n");
  }
  gst_object_unref(sinkpad);
}

// =============================================================================
// Answer flow: server sent us an offer; create+send answer, set local desc.
// =============================================================================

void on_answer_created(GstPromise* promise, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  const GstStructure* reply = gst_promise_get_reply(promise);
  GstWebRTCSessionDescription* answer = nullptr;
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION,
                    &answer, nullptr);
  gst_promise_unref(promise);
  if (!answer) {
    std::fprintf(stderr, "webrtc_recv: create-answer failed\n");
    return;
  }
  g_signal_emit_by_name(ctx->webrtc, "set-local-description", answer, nullptr);
  gchar* sdp_text = gst_sdp_message_as_text(answer->sdp);
  json msg;
  msg["type"] = "answer";
  msg["sdp"] = sdp_text ? sdp_text : "";
  send_json(ctx, msg);
  g_free(sdp_text);
  gst_webrtc_session_description_free(answer);
}

void handle_offer(Ctx* ctx, const std::string& sdp_text)
{
  GstSDPMessage* sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) {
    std::fprintf(stderr, "webrtc_recv: gst_sdp_message_new failed\n");
    return;
  }
  if (gst_sdp_message_parse_buffer(reinterpret_cast<const guint8*>(sdp_text.data()),
                                   sdp_text.size(), sdp) != GST_SDP_OK) {
    std::fprintf(stderr, "webrtc_recv: SDP parse failed\n");
    gst_sdp_message_free(sdp);
    return;
  }
  GstWebRTCSessionDescription* offer =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp);
  GstPromise* p = gst_promise_new();
  g_signal_emit_by_name(ctx->webrtc, "set-remote-description", offer, p);
  gst_promise_unref(p);
  gst_webrtc_session_description_free(offer);

  // Create answer.
  GstPromise* ap = gst_promise_new_with_change_func(on_answer_created, ctx, nullptr);
  g_signal_emit_by_name(ctx->webrtc, "create-answer", nullptr, ap);
}

void handle_ice(Ctx* ctx, const json& ice)
{
  if (!ice.contains("candidate") || !ice.contains("sdpMLineIndex")) return;
  std::string cand = ice["candidate"].get<std::string>();
  guint mline = ice["sdpMLineIndex"].get<guint>();
  g_signal_emit_by_name(ctx->webrtc, "add-ice-candidate", mline, cand.c_str());
}

void on_ws_message(SoupWebsocketConnection*, gint type, GBytes* msg, gpointer ud)
{
  if (type != SOUP_WEBSOCKET_DATA_TEXT) return;
  auto* ctx = static_cast<Ctx*>(ud);
  gsize size = 0;
  const char* data = static_cast<const char*>(g_bytes_get_data(msg, &size));
  std::string text(data, size);
  json parsed;
  try {
    parsed = json::parse(text);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "webrtc_recv: JSON parse: %s\n", e.what());
    return;
  }
  if (parsed.contains("type") && parsed.contains("sdp")) {
    std::string t = parsed["type"].get<std::string>();
    std::string sdp = parsed["sdp"].get<std::string>();
    if (t == "offer") handle_offer(ctx, sdp);
    // We never receive "answer" since we are always the answerer in this flow.
  } else if (parsed.contains("ice")) {
    handle_ice(ctx, parsed["ice"]);
  }
}

void on_ws_closed(SoupWebsocketConnection*, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  std::fprintf(stderr, "webrtc_recv: websocket closed\n");
  if (ctx->loop) g_main_loop_quit(ctx->loop);
}

// =============================================================================
// Pipeline + websocket bring-up.
// =============================================================================

void on_ws_connected(GObject* source, GAsyncResult* res, gpointer ud)
{
  auto* ctx = static_cast<Ctx*>(ud);
  GError* err = nullptr;
  ctx->ws = soup_session_websocket_connect_finish(SOUP_SESSION(source), res, &err);
  if (!ctx->ws) {
    if (ctx->connect_retries_remaining > 0) {
      ctx->connect_retries_remaining--;
      std::fprintf(stderr, "webrtc_recv: ws connect not yet up (%s), retrying\n",
                   err ? err->message : "unknown");
      if (err) g_error_free(err);
      // Retry after 500ms via the main loop.
      g_timeout_add(500, [](gpointer data) -> gboolean {
        start_ws_connect(static_cast<Ctx*>(data));
        return G_SOURCE_REMOVE;
      }, ctx);
      return;
    }
    std::fprintf(stderr, "webrtc_recv: ws connect failed: %s\n",
                 err ? err->message : "unknown");
    if (err) g_error_free(err);
    g_main_loop_quit(ctx->loop);
    return;
  }
  g_signal_connect(ctx->ws, "message", G_CALLBACK(on_ws_message), ctx);
  g_signal_connect(ctx->ws, "closed", G_CALLBACK(on_ws_closed), ctx);

  // Build the receive pipeline. webrtcbin handles RTP demux + ICE + DTLS;
  // bundle-policy=max-bundle matches what ros_camera_server's WebrtcOutput
  // configures on its end (see webrtc_output.cpp:320).
  ctx->pipeline = gst_pipeline_new("webrtc_recv_pipeline");
  ctx->webrtc = gst_element_factory_make("webrtcbin", "recv_webrtc");
  if (!ctx->webrtc) {
    std::fprintf(stderr, "webrtc_recv: webrtcbin element missing\n");
    g_main_loop_quit(ctx->loop);
    return;
  }
  // bundle-policy=max-bundle (3): GstWebRTCBundlePolicy enum value.
  // Server publishes a single bundled RTP m-line, so this matches.
  g_object_set(ctx->webrtc, "bundle-policy", 3,
               "stun-server", "stun://stun.l.google.com:19302", nullptr);
  gst_bin_add(GST_BIN(ctx->pipeline), ctx->webrtc);

  g_signal_connect(ctx->webrtc, "on-ice-candidate",
                   G_CALLBACK(on_ice_candidate), ctx);
  g_signal_connect(ctx->webrtc, "pad-added",
                   G_CALLBACK(on_pad_added), ctx);

  // Pre-add a video transceiver in RECVONLY mode so the server's offer can
  // negotiate against it before any media flows. webrtcbin will create the
  // transceiver lazily otherwise, but doing it eagerly keeps SDP setup
  // deterministic.
  GstCaps* recv_caps = gst_caps_from_string(
      "application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,"
      "payload=96");
  GstWebRTCRTPTransceiver* t = nullptr;
  g_signal_emit_by_name(ctx->webrtc, "add-transceiver",
                        GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_RECVONLY,
                        recv_caps, &t);
  if (t) g_object_unref(t);
  gst_caps_unref(recv_caps);

  if (gst_element_set_state(ctx->pipeline, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    std::fprintf(stderr, "webrtc_recv: failed to start pipeline\n");
    g_main_loop_quit(ctx->loop);
    return;
  }
}

// Kicks off / re-kicks an async websocket connect. Used for both the initial
// dial and connect-refused retries.
void start_ws_connect(Ctx* ctx)
{
  SoupMessage* msg = soup_message_new(SOUP_METHOD_GET, ctx->ws_uri.c_str());
  if (!msg) {
    std::fprintf(stderr, "webrtc_recv: invalid URI: %s\n", ctx->ws_uri.c_str());
    g_main_loop_quit(ctx->loop);
    return;
  }
  soup_session_websocket_connect_async(ctx->session, msg, nullptr, nullptr,
                                       G_PRIORITY_DEFAULT, nullptr,
                                       on_ws_connected, ctx);
  g_object_unref(msg);
}

void usage()
{
  std::cerr << "Usage: webrtc_recv --uri ws://HOST:PORT/CAM/IDX --csv FILE "
               "[--warmup-frames 150] [--max-frames 750] [--cell 16] "
               "[--connect-timeout-s 30]\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string ws_uri;
  std::string csv_path = "webrtc_recv.csv";
  int warmup = 150;
  int max_frames = 750;
  int cell = 16;
  int connect_timeout_s = 30;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* n) -> const char* {
      if (i + 1 >= argc) { std::cerr << n << " requires value\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--uri") ws_uri = next("--uri");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--warmup-frames") warmup = std::atoi(next("--warmup-frames"));
    else if (a == "--max-frames") max_frames = std::atoi(next("--max-frames"));
    else if (a == "--cell") cell = std::atoi(next("--cell"));
    else if (a == "--connect-timeout-s") connect_timeout_s = std::atoi(next("--connect-timeout-s"));
    else if (a == "--help" || a == "-h") { usage(); return 0; }
    else { std::cerr << "unknown: " << a << "\n"; usage(); return 2; }
  }
  if (ws_uri.empty()) { usage(); return 2; }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  gst_init(&argc, &argv);

  RecvLoop recv_loop({ csv_path, "webrtc_recv",
                       static_cast<uint32_t>(warmup),
                       static_cast<uint32_t>(max_frames), cell });
  if (recv_loop.failed()) return 1;

  Ctx ctx;
  ctx.loop_state = &recv_loop;
  ctx.loop = g_main_loop_new(nullptr, FALSE);

  ctx.session = soup_session_new();
  ctx.ws_uri = ws_uri;
  // Retry every 500ms until connect_timeout_s elapses. This handles
  // start-order races: receivers run before ros_camera_server's signaling
  // server binds. 30s default covers the worst-case stack startup time.
  ctx.connect_retries_remaining = std::max(0, connect_timeout_s) * 2;
  start_ws_connect(&ctx);

  guint timeout_id = g_timeout_add(200, [](gpointer data) -> gboolean {
    auto* c = static_cast<Ctx*>(data);
    if (g_stop.load()) { g_main_loop_quit(c->loop); return G_SOURCE_REMOVE; }
    return G_SOURCE_CONTINUE;
  }, &ctx);
  (void)timeout_id;

  g_main_loop_run(ctx.loop);

  if (ctx.pipeline) {
    gst_element_set_state(ctx.pipeline, GST_STATE_NULL);
    gst_object_unref(ctx.pipeline);
  }
  if (ctx.ws) {
    if (soup_websocket_connection_get_state(ctx.ws) == SOUP_WEBSOCKET_STATE_OPEN) {
      soup_websocket_connection_close(ctx.ws,
                                      SOUP_WEBSOCKET_CLOSE_NORMAL, "bye");
    }
    g_object_unref(ctx.ws);
  }
  if (ctx.session) g_object_unref(ctx.session);
  g_main_loop_unref(ctx.loop);

  return 0;
}
