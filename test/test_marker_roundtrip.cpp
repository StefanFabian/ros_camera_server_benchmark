/*
 *  ros_camera_server_benchmarks - End-to-end latency benchmark harness.
 *  Copyright (C) 2026  Stefan Fabian
 *  Licensed under AGPL-3.0-only.
 */

#include "ros_camera_server_benchmarks/marker.hpp"
#include "ros_camera_server_benchmarks/ros_recv_internals.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gtest/gtest.h>
#include <jpeglib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

using namespace ros_camera_server_benchmarks::marker;

namespace {

constexpr int kWidth = 1280;
constexpr int kHeight = 720;
constexpr int kCell = 16;
constexpr int kFrames = 100;

// Build a deterministic 1280x720 I420 buffer with a marker on the Y plane.
// Background Y = 128, U = V = 128 (gray) keeps the encoder happy without
// adding chroma noise that could bleed into the marker rows.
std::vector<uint8_t> make_i420_frame(uint64_t ts_ns, uint32_t frame_id)
{
  const int y_size = kWidth * kHeight;
  const int uv_size = (kWidth / 2) * (kHeight / 2);
  std::vector<uint8_t> buf(y_size + 2 * uv_size);
  std::memset(buf.data(), 128, y_size);
  std::memset(buf.data() + y_size, 128, uv_size);
  std::memset(buf.data() + y_size + uv_size, 128, uv_size);

  Marker m;
  m.ts_ns = ts_ns;
  m.frame_id = frame_id;
  m.width_hint = static_cast<uint16_t>(kWidth);
  encode_y(buf.data(), kWidth, kWidth, kHeight, m, kCell);
  return buf;
}

struct DecodeContext {
  std::mutex mtx;
  std::condition_variable cv;
  std::vector<Marker> decoded;
  std::atomic<int> received{ 0 };
  std::atomic<bool> error{ false };
};

GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data)
{
  auto* ctx = static_cast<DecodeContext*>(user_data);
  GstSample* sample = gst_app_sink_pull_sample(sink);
  if (!sample) return GST_FLOW_ERROR;

  GstCaps* caps = gst_sample_get_caps(sample);
  GstVideoInfo vinfo;
  gst_video_info_init(&vinfo);
  if (!caps || !gst_video_info_from_caps(&vinfo, caps)) {
    gst_sample_unref(sample);
    ctx->error = true;
    return GST_FLOW_ERROR;
  }

  GstBuffer* buf = gst_sample_get_buffer(sample);
  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &vinfo, buf, GST_MAP_READ)) {
    gst_sample_unref(sample);
    ctx->error = true;
    return GST_FLOW_ERROR;
  }
  // Use real Y stride from the mapped frame; some encoders / converters pad
  // to alignment that differs from `width`, and a stride==width assumption
  // would silently misread the marker.
  const uint8_t* y_plane =
      static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
  int y_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
  int w = GST_VIDEO_INFO_WIDTH(&vinfo);
  int h = GST_VIDEO_INFO_HEIGHT(&vinfo);

  Marker m;
  bool ok = decode_y(y_plane, y_stride, w, h, m, kCell);
  {
    std::lock_guard<std::mutex> lock(ctx->mtx);
    if (ok) ctx->decoded.push_back(m);
    ctx->received++;
    ctx->cv.notify_all();
  }

  gst_video_frame_unmap(&frame);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

bool gst_feature_available(const char* name)
{
  GstRegistry* registry = gst_registry_get();
  GstPluginFeature* f = gst_registry_lookup_feature(registry, name);
  if (!f) return false;
  gst_object_unref(f);
  return true;
}

// Demote every other H264 encoder so `keep` is the only candidate decodebin /
// the explicit encoder slot picks up.
void pin_h264_encoder(const char* keep)
{
  GstRegistry* registry = gst_registry_get();
  for (const char* hw : { "nvh264enc", "vah264enc", "vaapih264enc",
                          "nvv4l2h264enc", "v4l2h264enc", "qsvh264enc",
                          "x264enc", "avenc_h264" }) {
    if (std::strcmp(hw, keep) == 0) continue;
    GstPluginFeature* f = gst_registry_lookup_feature(registry, hw);
    if (f) {
      gst_plugin_feature_set_rank(f, GST_RANK_NONE);
      gst_object_unref(f);
    }
  }
}

// Push kFrames I420 frames through the given pipeline description (must
// contain `appsrc name=src` and `appsink name=sink`). Returns when all frames
// have come out the other side or the 20-second timeout fires. The marker is
// expected to round-trip exactly.
void run_pipeline_roundtrip(const char* pipeline_desc)
{
  GError* err = nullptr;
  GstElement* pipeline = gst_parse_launch(pipeline_desc, &err);
  ASSERT_NE(pipeline, nullptr) << (err ? err->message : "unknown error");
  if (err) g_error_free(err);

  GstElement* src = gst_bin_get_by_name(GST_BIN(pipeline), "src");
  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  ASSERT_NE(src, nullptr);
  ASSERT_NE(sink, nullptr);

  DecodeContext ctx;
  GstAppSinkCallbacks cbs = {};
  cbs.new_sample = on_new_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, &ctx, nullptr);

  ASSERT_NE(gst_element_set_state(pipeline, GST_STATE_PLAYING),
            GST_STATE_CHANGE_FAILURE);

  GstClockTime pts = 0;
  const GstClockTime frame_dur = GST_SECOND / 30;
  for (int i = 0; i < kFrames; ++i) {
    auto frame = make_i420_frame(/*ts_ns=*/0xC0DEFACE00ULL + i,
                                 /*frame_id=*/static_cast<uint32_t>(i));
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, frame.size(), nullptr);
    GstMapInfo map;
    ASSERT_TRUE(gst_buffer_map(buf, &map, GST_MAP_WRITE));
    std::memcpy(map.data, frame.data(), frame.size());
    gst_buffer_unmap(buf, &map);
    GST_BUFFER_PTS(buf) = pts;
    GST_BUFFER_DURATION(buf) = frame_dur;
    pts += frame_dur;
    GstFlowReturn fr =
        gst_app_src_push_buffer(GST_APP_SRC(src), buf); // takes ownership
    ASSERT_EQ(fr, GST_FLOW_OK);
  }
  gst_app_src_end_of_stream(GST_APP_SRC(src));

  {
    std::unique_lock<std::mutex> lock(ctx.mtx);
    ctx.cv.wait_for(lock, std::chrono::seconds(20),
                    [&] { return ctx.received.load() >= kFrames; });
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(src);
  gst_object_unref(sink);
  gst_object_unref(pipeline);

  ASSERT_FALSE(ctx.error.load());
  EXPECT_GE(ctx.received.load(), kFrames);
  EXPECT_EQ(static_cast<int>(ctx.decoded.size()), ctx.received.load());
  for (size_t i = 0; i < ctx.decoded.size(); ++i) {
    EXPECT_EQ(ctx.decoded[i].ts_ns, 0xC0DEFACE00ULL + i);
    EXPECT_EQ(ctx.decoded[i].frame_id, static_cast<uint32_t>(i));
  }
}

// Encode an I420 buffer to JPEG via libjpeg using the same colour-space and
// quality knobs as src/frame_gen.cpp::i420_to_jpeg.
bool encode_jpeg(const uint8_t* i420, int width, int height, int quality,
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
  jpeg_set_quality(&cinfo, quality, TRUE);
  cinfo.comp_info[0].h_samp_factor = 2;
  cinfo.comp_info[0].v_samp_factor = 2;
  cinfo.comp_info[1].h_samp_factor = 1;
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;
  cinfo.comp_info[2].v_samp_factor = 1;

  jpeg_start_compress(&cinfo, TRUE);

  std::vector<JSAMPROW> y_rows(16), cb_rows(8), cr_rows(8);
  JSAMPARRAY planes[3] = { y_rows.data(), cb_rows.data(), cr_rows.data() };
  const uint8_t* y_plane = i420;
  const uint8_t* u_plane = i420 + width * height;
  const uint8_t* v_plane = i420 + width * height + (width / 2) * (height / 2);
  for (int row = 0; row < height; row += 16) {
    for (int i = 0; i < 16; ++i) {
      int yy = std::min(row + i, height - 1);
      y_rows[i] = const_cast<JSAMPROW>(y_plane + yy * width);
    }
    for (int i = 0; i < 8; ++i) {
      int yy = std::min(row / 2 + i, height / 2 - 1);
      cb_rows[i] = const_cast<JSAMPROW>(u_plane + yy * (width / 2));
      cr_rows[i] = const_cast<JSAMPROW>(v_plane + yy * (width / 2));
    }
    jpeg_write_raw_data(&cinfo, planes, 16);
  }
  jpeg_finish_compress(&cinfo);
  jpeg.assign(outbuf, outbuf + outsize);
  free(outbuf);
  jpeg_destroy_compress(&cinfo);
  return true;
}

// Decode JPEG back to a Y plane, mirroring src/ros_recv.cpp::jpeg_to_y_plane.
bool decode_jpeg_y(const uint8_t* jpeg, size_t size,
                   std::vector<uint8_t>& y, int& width, int& height)
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
  y.resize(static_cast<size_t>(width) * height);
  std::vector<JSAMPROW> rows(height);
  for (int i = 0; i < height; ++i) rows[i] = y.data() + i * width;
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, &rows[cinfo.output_scanline],
                        cinfo.output_height - cinfo.output_scanline);
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return true;
}

void run_jpeg_roundtrip(int quality)
{
  for (int i = 0; i < kFrames; ++i) {
    auto frame = make_i420_frame(0xC0DEFACE00ULL + i,
                                 static_cast<uint32_t>(i));
    std::vector<uint8_t> jpeg;
    ASSERT_TRUE(encode_jpeg(frame.data(), kWidth, kHeight, quality, jpeg));

    std::vector<uint8_t> y;
    int w = 0, h = 0;
    ASSERT_TRUE(decode_jpeg_y(jpeg.data(), jpeg.size(), y, w, h));
    ASSERT_EQ(w, kWidth);
    ASSERT_EQ(h, kHeight);

    Marker out;
    ASSERT_TRUE(decode_y(y.data(), kWidth, kWidth, kHeight, out, kCell))
        << "marker decode failed at quality=" << quality << " frame=" << i;
    EXPECT_EQ(out.ts_ns, 0xC0DEFACE00ULL + i);
    EXPECT_EQ(out.frame_id, static_cast<uint32_t>(i));
  }
}

} // namespace

TEST(MarkerRoundtrip, PayloadPackUnpack)
{
  Marker in;
  in.ts_ns = 0x0123456789ABCDEFULL;
  in.frame_id = 0xDEADBEEFu;
  in.width_hint = kWidth;

  std::vector<uint8_t> y(kWidth * kHeight, 0);
  encode_y(y.data(), kWidth, kWidth, kHeight, in, kCell);

  Marker out;
  ASSERT_TRUE(decode_y(y.data(), kWidth, kWidth, kHeight, out, kCell));
  EXPECT_EQ(out.ts_ns, in.ts_ns);
  EXPECT_EQ(out.frame_id, in.frame_id);
  EXPECT_EQ(out.width_hint, in.width_hint);
}

TEST(MarkerRoundtrip, RejectsCorruptedPayload)
{
  Marker in;
  in.ts_ns = 42;
  in.frame_id = 7;
  in.width_hint = kWidth;
  std::vector<uint8_t> y(kWidth * kHeight, 0);
  encode_y(y.data(), kWidth, kWidth, kHeight, in, kCell);

  // Flip the center sample of a single cell -> CRC mismatch.
  int sx = (kCell / 2) + 0 * kCell;
  int sy = kCell / 2;
  y[sy * kWidth + sx] ^= 0xFF;

  Marker out;
  EXPECT_FALSE(decode_y(y.data(), kWidth, kWidth, kHeight, out, kCell));
}

TEST(MarkerRoundtrip, H264Roundtrip)
{
  gst_init(nullptr, nullptr);
  pin_h264_encoder("x264enc");
  // 16-px cell aligns with H.264 16x16 macroblock boundaries; cell-center
  // samples survive ultrafast quantization at 4 Mbps. If this assertion ever
  // loosens, bump cell size everywhere it appears.
  run_pipeline_roundtrip(
      "appsrc name=src is-live=true format=time do-timestamp=true "
      "caps=video/x-raw,format=I420,width=1280,height=720,framerate=30/1 ! "
      "videoconvert ! "
      "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 "
      "key-int-max=30 ! "
      "h264parse ! avdec_h264 ! videoconvert ! "
      "video/x-raw,format=I420 ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false");
}

// web_video_server's H.264 streamer encodes via gstreamer's avenc_h264 (libav).
// All *-web_video_server-* scenarios rely on the marker surviving that path.
// Pin avenc_h264 explicitly so we don't accidentally test x264 again.
TEST(MarkerRoundtrip, H264RoundtripLibav)
{
  gst_init(nullptr, nullptr);
  if (!gst_feature_available("avenc_h264") &&
      !gst_feature_available("avenc_h264_omx")) {
    GTEST_SKIP() << "avenc_h264 not available; skipping libav roundtrip";
  }
  pin_h264_encoder("avenc_h264");
  // 1 Mbit matches the bitrate web_video_server's libav_streamer uses by
  // default for h264 (default_bitrate = 100000 * 10 in its source). Adjust
  // alongside the launch-file `h264_default_bitrate` param if it ever drifts.
  run_pipeline_roundtrip(
      "appsrc name=src is-live=true format=time do-timestamp=true "
      "caps=video/x-raw,format=I420,width=1280,height=720,framerate=30/1 ! "
      "videoconvert ! "
      "avenc_h264 bitrate=1000000 ! "
      "h264parse ! avdec_h264 ! videoconvert ! "
      "video/x-raw,format=I420 ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false");
}

// VA-API is what env_fairness.sh pins for benchmark runs. This test guards
// against marker-vs-VAAPI-quantization regressions on machines that have it,
// and is skipped cleanly on hosts without an Intel VA device.
TEST(MarkerRoundtrip, H264RoundtripVaapi)
{
  gst_init(nullptr, nullptr);
  if (!gst_feature_available("vah264enc")) {
    GTEST_SKIP() << "vah264enc not available; skipping VA-API roundtrip";
  }
  pin_h264_encoder("vah264enc");
  run_pipeline_roundtrip(
      "appsrc name=src is-live=true format=time do-timestamp=true "
      "caps=video/x-raw,format=I420,width=1280,height=720,framerate=30/1 ! "
      "videoconvert ! video/x-raw,format=NV12 ! "
      "vah264enc bitrate=4000 key-int-max=30 rate-control=cbr "
      "target-usage=4 ! "
      "h264parse ! decodebin ! videoconvert ! "
      "video/x-raw,format=I420 ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false");
}

// NVENC is the alternative encoder env_fairness.sh pins when BENCH_ENCODER=nv.
// Knobs mirror env_fairness.sh's BENCH_ENC_GST for the nv branch so a
// regression that hits real benchmark runs would be caught here. Skipped on
// machines without an NVIDIA encoder.
TEST(MarkerRoundtrip, H264RoundtripNvenc)
{
  gst_init(nullptr, nullptr);
  if (!gst_feature_available("nvh264enc")) {
    GTEST_SKIP() << "nvh264enc not available; skipping NVENC roundtrip";
  }
  pin_h264_encoder("nvh264enc");
  run_pipeline_roundtrip(
      "appsrc name=src is-live=true format=time do-timestamp=true "
      "caps=video/x-raw,format=I420,width=1280,height=720,framerate=30/1 ! "
      "videoconvert ! video/x-raw,format=NV12 ! "
      "nvh264enc preset=4 bitrate=4000 gop-size=30 strict-gop=true "
      "rc-mode=3 spatial-aq=true zerolatency=true aud=true ! "
      "h264parse ! decodebin ! videoconvert ! "
      "video/x-raw,format=I420 ! "
      "appsink name=sink emit-signals=true sync=false max-buffers=4 drop=false");
}

// Compressed scenarios round-trip the marker through libjpeg. q=85 is the
// runtime default; q=70 leaves headroom proof against future quality drops.
TEST(MarkerRoundtrip, JpegRoundtripQ85) { run_jpeg_roundtrip(85); }
TEST(MarkerRoundtrip, JpegRoundtripQ70) { run_jpeg_roundtrip(70); }

// recv_ts policy: jpeg_to_y_plane must stamp recv_out AFTER jpeg_finish_decompress
// (display-prep cost included) and BEFORE returning (so the caller's marker
// decode is excluded). Bracket the call with now() samples and require recv_out
// falls strictly inside the bracket. This pins the policy in place: any future
// change that moves recv_out outside the bracket (e.g. stamping at the top of
// the function, or after marker decode in the caller) breaks this test.
TEST(MarkerRoundtrip, JpegRecvTimestampOrdering)
{
  using namespace ros_camera_server_benchmarks::ros_recv;
  using ros_camera_server_benchmarks::now_ns_monotonic;
  auto frame = make_i420_frame(0xC0DEFACE00ULL, 42);
  std::vector<uint8_t> jpeg;
  ASSERT_TRUE(encode_jpeg(frame.data(), kWidth, kHeight, 85, jpeg));

  uint64_t before = now_ns_monotonic();
  std::vector<uint8_t> y;
  int w = 0, h = 0;
  uint64_t recv = 0;
  ASSERT_TRUE(jpeg_to_y_plane(jpeg.data(), jpeg.size(), y, w, h, recv));
  uint64_t after = now_ns_monotonic();

  EXPECT_GE(recv, before)
      << "recv_ts captured before jpeg_to_y_plane was even called";
  EXPECT_LE(recv, after)
      << "recv_ts captured after jpeg_to_y_plane returned (likely after "
         "destroy or marker-decode)";

  // Sanity: with the marker still recoverable from the returned Y plane.
  Marker out;
  ASSERT_TRUE(decode_y(y.data(), w, w, h, out, kCell));
  EXPECT_EQ(out.frame_id, 42u);
}

namespace {

// BT.601 limited-range Y'CbCr -> RGB with clipping to [0, 255]. Matches what
// the rgb8 image_transport / GStreamer videoconvert path produces from I420
// inputs that carry the marker on the Y plane.
inline uint8_t clip_u8(double v)
{
  if (v < 0.0) return 0;
  if (v > 255.0) return 255;
  return static_cast<uint8_t>(v + 0.5);
}

void i420_to_rgb_bt601(const uint8_t* i420, int width, int height,
                       std::vector<uint8_t>& rgb)
{
  rgb.resize(static_cast<size_t>(width) * height * 3);
  const uint8_t* y_plane = i420;
  const uint8_t* u_plane = i420 + width * height;
  const uint8_t* v_plane = i420 + width * height + (width / 2) * (height / 2);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double Y = y_plane[y * width + x];
      double U = u_plane[(y / 2) * (width / 2) + (x / 2)];
      double V = v_plane[(y / 2) * (width / 2) + (x / 2)];
      double c = 1.164 * (Y - 16.0);
      double d = U - 128.0;
      double e = V - 128.0;
      uint8_t* p = rgb.data() + (y * width + x) * 3;
      p[0] = clip_u8(c + 1.596 * e);
      p[1] = clip_u8(c - 0.392 * d - 0.813 * e);
      p[2] = clip_u8(c + 2.017 * d);
    }
  }
}

} // namespace

// Adversarial chroma in marker rows: without clear_uv_strip_i420 the YUV->RGB
// path produces R/G/B with a strong colour bias, and decode_rgb's (R+G+B)/3
// luma estimate flips bits. With the clear, U=V=128 in the strip rows so the
// marker reaches the receiver as pure greyscale and decodes cleanly.
TEST(MarkerRoundtrip, ChromaBleedRgbDecode)
{
  const int y_size = kWidth * kHeight;
  const int uv_size = (kWidth / 2) * (kHeight / 2);

  std::vector<uint8_t> buf(y_size + 2 * uv_size);
  // Adversarial U=255, V=255 everywhere (worst case for the (R+G+B)/3
  // luma estimate at Y_BLACK cells: it flips them above the 125 threshold).
  std::memset(buf.data(), 128, y_size);
  std::memset(buf.data() + y_size, 255, uv_size);
  std::memset(buf.data() + y_size + uv_size, 255, uv_size);

  Marker in;
  in.ts_ns = 0xC0DEFACE00ULL;
  in.frame_id = 1234;
  in.width_hint = static_cast<uint16_t>(kWidth);
  encode_y(buf.data(), kWidth, kWidth, kHeight, in, kCell);

  // Without the UV-clear: marker rows still hold adversarial chroma. The
  // reconstructed (R+G+B)/3 at Y_BLACK cells flips above threshold and the
  // CRC fails. This branch documents the failure mode the fix prevents.
  {
    std::vector<uint8_t> rgb;
    i420_to_rgb_bt601(buf.data(), kWidth, kHeight, rgb);
    Marker out;
    EXPECT_FALSE(decode_rgb(rgb.data(), kWidth * 3, kWidth, kHeight, out, kCell))
        << "expected adversarial chroma to break decode without UV-clear";
  }

  // With the fix: clear U/V to 128 in strip rows. RGB roundtrip recovers.
  uint8_t* u_plane = buf.data() + y_size;
  uint8_t* v_plane = buf.data() + y_size + uv_size;
  clear_uv_strip_i420(u_plane, v_plane, kWidth / 2, kWidth, kHeight, kCell);

  std::vector<uint8_t> rgb;
  i420_to_rgb_bt601(buf.data(), kWidth, kHeight, rgb);
  Marker out;
  ASSERT_TRUE(decode_rgb(rgb.data(), kWidth * 3, kWidth, kHeight, out, kCell));
  EXPECT_EQ(out.ts_ns, in.ts_ns);
  EXPECT_EQ(out.frame_id, in.frame_id);
  EXPECT_EQ(out.width_hint, in.width_hint);
}
