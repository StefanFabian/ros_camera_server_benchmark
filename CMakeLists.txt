cmake_minimum_required(VERSION 3.16)
project(ros_camera_server_benchmarks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(JPEG REQUIRED IMPORTED_TARGET libjpeg)
# WebRTC client uses webrtcbin + libsoup websocket. Optional: webrtc_recv is
# only built when both are found, so machines without gst-plugins-bad / libsoup
# can still build the rest of the benchmark.
pkg_check_modules(GST_WEBRTC IMPORTED_TARGET gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
pkg_check_modules(SOUP IMPORTED_TARGET libsoup-3.0)
find_package(nlohmann_json QUIET)

add_library(benchmark_marker src/marker.cpp src/ros_recv_internals.cpp)
target_include_directories(benchmark_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(benchmark_marker PkgConfig::JPEG)

# Shared per-frame bookkeeping for every receiver: marker decode, warmup-skip,
# max-frames stop, latency CSV append, summary line. gst_recv / webrtc_recv /
# ros_recv all funnel into this so the comparison surface is one TU, not
# three near-duplicates.
add_library(benchmark_recv_loop src/recv_loop.cpp)
target_include_directories(benchmark_recv_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(benchmark_recv_loop benchmark_marker)

# Shared render TU: BaseVideoLoader (streaming decode), render_i420,
# i420_to_rgb24, i420_to_yuyv. Used by frame_gen (V4L2 producer) and ros_pub
# (ROS topic producer) so both see the same encoder workload when BENCH_VIDEO
# is set.
add_library(benchmark_frame_render src/frame_render.cpp)
target_include_directories(benchmark_frame_render PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(benchmark_frame_render benchmark_marker PkgConfig::GST)

add_executable(frame_gen src/frame_gen.cpp)
target_link_libraries(frame_gen benchmark_frame_render PkgConfig::JPEG PkgConfig::GST)

add_executable(ros_pub src/ros_pub.cpp)
ament_target_dependencies(ros_pub rclcpp sensor_msgs image_transport)
target_link_libraries(ros_pub benchmark_frame_render PkgConfig::JPEG)

# Generalized GStreamer receiver parameterized over transport
# (udp-rtp, http-mjpeg). Used by all G2/G3/G4 stream scenarios except
# WebRTC (which gets its own binary because it needs websocket
# signaling) and RTSP (which uses image_transport republish instead).
add_executable(gst_recv src/gst_recv.cpp)
target_link_libraries(gst_recv benchmark_recv_loop PkgConfig::GST)

add_executable(ros_recv src/ros_recv.cpp)
ament_target_dependencies(ros_recv rclcpp sensor_msgs)
target_link_libraries(ros_recv benchmark_recv_loop PkgConfig::JPEG)

# WebRTC receiver: implements ros_camera_server's signaling protocol over
# libsoup websocket and decodes via webrtcbin. Built only when both deps are
# available; G*-webrtc scenarios skip cleanly if the binary is absent.
set(WEBRTC_RECV_BUILT FALSE)
if(GST_WEBRTC_FOUND AND SOUP_FOUND)
  add_executable(webrtc_recv src/webrtc_recv.cpp)
  target_link_libraries(webrtc_recv
    benchmark_recv_loop PkgConfig::GST PkgConfig::GST_WEBRTC PkgConfig::SOUP)
  if(nlohmann_json_FOUND)
    target_link_libraries(webrtc_recv nlohmann_json::nlohmann_json)
  endif()
  set(WEBRTC_RECV_BUILT TRUE)
endif()

# LD_PRELOAD shim that fixes v4l2loopback's missing V4L2_CAP_TIMEPERFRAME
# in VIDIOC_G_PARM, which otherwise makes usb_cam_node_exe abort.
add_library(v4l2loopback_timeperframe_shim SHARED
  src/v4l2loopback_timeperframe_shim.c)
set_target_properties(v4l2loopback_timeperframe_shim PROPERTIES
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
  PREFIX "lib")
target_link_libraries(v4l2loopback_timeperframe_shim ${CMAKE_DL_LIBS})

# LD_PRELOAD shim that pins ffmpeg_image_transport's VAAPI encoder to the
# DRM render node in $BENCH_VAAPI_DRM_DEVICE. Needed because libavutil's
# autodetect picks /dev/dri/renderD128 with no fallback when the iGPU is
# not the primary DRM device (e.g. NVIDIA dGPU + Intel iGPU host).
add_library(vaapi_drm_redirect_shim SHARED
  src/vaapi_drm_redirect_shim.c)
set_target_properties(vaapi_drm_redirect_shim PROPERTIES
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
  PREFIX "lib")
target_link_libraries(vaapi_drm_redirect_shim ${CMAKE_DL_LIBS})

install(TARGETS benchmark_marker benchmark_recv_loop benchmark_frame_render
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS frame_gen ros_pub gst_recv ros_recv
  DESTINATION lib/${PROJECT_NAME})

if(WEBRTC_RECV_BUILT)
  install(TARGETS webrtc_recv DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS v4l2loopback_timeperframe_shim vaapi_drm_redirect_shim
  LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_marker_roundtrip test/test_marker_roundtrip.cpp)
  target_link_libraries(test_marker_roundtrip benchmark_marker PkgConfig::GST PkgConfig::JPEG)

  ament_add_gtest(test_mjpeg_subsampling test/test_mjpeg_subsampling.cpp)
  target_link_libraries(test_mjpeg_subsampling benchmark_marker PkgConfig::JPEG)

  ament_add_gtest(test_recv_loop test/test_recv_loop.cpp)
  target_link_libraries(test_recv_loop benchmark_recv_loop)
endif()

ament_package()
