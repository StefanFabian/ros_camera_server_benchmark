# Source me. Pins encoder/decoder selection so the same elements are used
# across every stack-under-test, removing GPU-driver / CPU variance as a
# confounder.
#
# Encoder family resolution (probe + interactive prompt) lives in
# resolve_encoder.sh so run_all.sh can resolve once up front and keep the
# benchmark loop uninterrupted. This script only builds the BITRATE-dependent
# encoder fragment and the GST_PLUGIN_FEATURE_RANK demotions, which are
# scenario-local.
#
# Inputs (set by run_scenario.sh before sourcing):
#   BITRATE   target bitrate in kbps (mandatory; the gst encoder string bakes it in)
#
# Outputs (exported, consumed by run_scenario.sh shell helpers and the
# ros_camera_server config envsubst step):
#   BENCH_ENCODER          'va', 'nv', or 'mpp' (selected encoder family)
#   BENCH_ENC_GST          gst-launch-1.0 encoder element + knobs (single token-friendly string)
#   BENCH_RCS_ENC          'VA_LP', 'NV', or 'MPP' (string consumed by ros_camera_server's codecFromString)
#   BENCH_FFMPEG_ENC       libav encoder name (h264_vaapi/h264_nvenc/h264_rkmpp) for ffmpeg_image_transport
#   BENCH_FFMPEG_AV_OPTS   comma-separated libav AVOptions for the chosen encoder (passed as `encoder_av_options`).
#                          Pins low-latency tuning equivalent to the GStreamer
#                          settings above (zerolatency / no B-frames / fast preset)
#                          so cross-stack p50 reflects pipeline overhead, not codec choice.
#   BENCH_VAAPI_DRM_DEVICE  va branch only: render node to redirect renderD* opens to
#                          (consumed by vaapi_drm_redirect_shim under LD_PRELOAD).
#   GST_PLUGIN_FEATURE_RANK  augmented to demote competing encoders/decoders

# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/resolve_encoder.sh"

# Build the gst-launch encoder fragment + the ros_camera_server codec name.
# Knobs mirror ros_camera_server/src/codecs/h264.hpp (VA_LP block lines
# 135–143, NV block 163–171, MPP block 184–193) so the bench's gst-launch
# helpers and ros_camera_server's own pipeline emit identical encoder
# configuration on whichever backend is selected.
case "${BENCH_ENCODER}" in
  va)
    BENCH_ENC_GST="vah264lpenc bitrate=${BITRATE} key-int-max=30 aud=true min-force-key-unit-interval=1000000000 num-slices=1 cabac=true dct8x8=true rate-control=vbr ref-frames=1"
    BENCH_RCS_ENC="VA_LP"
    BENCH_FFMPEG_ENC="h264_vaapi"
    # h264_vaapi: rc_mode=VBR (matches vah264lpenc rate-control=vbr).
    # quality=7 is the speed/quality slider's fastest end (low-latency).
    # idr_interval mirrors gop_size for a closed-GOP stream.
    BENCH_FFMPEG_AV_OPTS="rc_mode:VBR,quality:7,idr_interval:30,async_depth:1"
    # ffmpeg_image_transport calls av_hwdevice_ctx_create(VAAPI, NULL, ...)
    # which iterates /dev/dri/renderD%d from 128 and stops on the first
    # opener — no fallback when vaInitialize() fails. On hosts where
    # renderD128 is not VAAPI-capable (NVIDIA dGPU primary + Intel iGPU
    # secondary) every encode session aborts. Probe each render node and
    # export the first VAAPI-encode-capable one; the launch files preload
    # vaapi_drm_redirect_shim which redirects renderD* opens to it.
    BENCH_VAAPI_DRM_DEVICE=""
    for _va_dev in /dev/dri/renderD*; do
      [[ -e "${_va_dev}" ]] || continue
      if vainfo --display drm --device "${_va_dev}" 2>/dev/null \
           | grep -q 'VAEntrypointEncSliceLP\|VAEntrypointEncSlice'; then
        BENCH_VAAPI_DRM_DEVICE="${_va_dev}"; break
      fi
    done
    unset _va_dev
    if [[ -z "${BENCH_VAAPI_DRM_DEVICE}" ]]; then
      echo "[env_fairness] no VAAPI-capable render node found; ffmpeg_transport-h264_vaapi arms will fail." >&2
    fi
    export BENCH_VAAPI_DRM_DEVICE
    # VA-selected: demote nv*/qsv*/v4l2h264*/x264enc/mpp*/avdec_h264 so VA wins.
    # Software fallbacks (x264enc, avdec_h264) ranked NONE rather than
    # MARGINAL — if the chosen hardware encoder fails to instantiate at
    # runtime we want explicit failure, not a silent software fallback that
    # breaks the "pinned encoder" contract.
    # JPEG-decoder demotions mirror the H.264 ones so decodebin on the
    # http-mjpeg receive path picks vajpegdec when VA is selected. jpegdec
    # (libjpeg-turbo software) is left un-demoted as the fallback when no
    # HW JPEG decoder is installed.
    _BENCH_RANK_DEMOTIONS="nvh264enc:NONE,nvh264device0enc:NONE,nvv4l2h264enc:NONE,v4l2h264enc:NONE,qsvh264enc:NONE,mpph264enc:NONE,x264enc:NONE,nvh264dec:NONE,nvv4l2decoder:NONE,v4l2h264dec:NONE,qsvh264dec:NONE,mpph264dec:NONE,avdec_h264:NONE,nvjpegdec:NONE,qsvjpegdec:NONE,mppjpegdec:NONE"
    ;;
  nv)
    BENCH_ENC_GST="nvh264enc preset=4 bitrate=${BITRATE} gop-size=30 strict-gop=true rc-mode=3 min-force-key-unit-interval=1000000000 spatial-aq=true zerolatency=true aud=true"
    BENCH_RCS_ENC="NV"
    BENCH_FFMPEG_ENC="h264_nvenc"
    # h264_nvenc: preset=p1 (fastest), tune=ll (low-latency), zerolatency=1
    # mirror nvh264enc's preset=4/zerolatency=true. delay=0 disables NVENC's
    # internal frame-reorder buffer; rc=vbr matches GStreamer rc-mode=3.
    BENCH_FFMPEG_AV_OPTS="preset:p1,tune:ll,zerolatency:1,delay:0,rc:vbr"
    # NV-selected: demote va*/qsv*/v4l2h264*/x264enc/mpp*/avdec_h264, leave
    # NV decoders ranked normally so receivers' decodebin picks NVDEC.
    # Software fallbacks (x264enc, avdec_h264) ranked NONE for the same
    # reason as the VA branch above.
    # JPEG-decoder demotions mirror the H.264 ones so decodebin on the
    # http-mjpeg receive path picks nvjpegdec when NV is selected. jpegdec
    # (libjpeg-turbo software) is left un-demoted as the fallback.
    _BENCH_RANK_DEMOTIONS="vah264enc:NONE,vah264lpenc:NONE,vaapih264enc:NONE,qsvh264enc:NONE,v4l2h264enc:NONE,mpph264enc:NONE,x264enc:NONE,vah264dec:NONE,vah264lpdec:NONE,vaapih264dec:NONE,qsvh264dec:NONE,v4l2h264dec:NONE,mpph264dec:NONE,avdec_h264:NONE,vajpegdec:NONE,qsvjpegdec:NONE,mppjpegdec:NONE"
    ;;
  mpp)
    # mpph264enc.bps takes bits-per-second (the VA/NV `bitrate` properties
    # take kbps); convert from BITRATE here so the same operating point
    # holds across families. profile=66 (baseline) / rc-mode=0 (vbr) /
    # max-pending=1 / zero-copy-pkt match the MPP branch in
    # ros_camera_server/src/codecs/h264.hpp:184-193.
    BENCH_ENC_GST="mpph264enc bps=$((BITRATE * 1000)) gop=30 header-mode=1 profile=66 rc-mode=0 max-pending=1 min-force-key-unit-interval=1000000000 zero-copy-pkt=true"
    BENCH_RCS_ENC="MPP"
    BENCH_FFMPEG_ENC="h264_rkmpp"
    # h264_rkmpp: rc_mode=VBR matches mpph264enc rc-mode=0; level=3.0 baseline
    # mirrors profile=66; mpph264enc has max-pending=1 — async_frames=1 is
    # the ffmpeg_rkmpp equivalent.
    BENCH_FFMPEG_AV_OPTS="rc_mode:VBR,level:30,profile:baseline,async_frames:1"
    # MPP-selected: demote va*/nv*/qsv*/v4l2h264*/x264enc/avdec_h264 so
    # mpph264enc wins for the producer side, and demote competing H.264
    # decoders so receivers' decodebin picks mpph264dec.
    # JPEG-decoder demotions mirror the H.264 ones so decodebin on the
    # http-mjpeg receive path picks mppjpegdec when MPP is selected.
    _BENCH_RANK_DEMOTIONS="vah264enc:NONE,vah264lpenc:NONE,vaapih264enc:NONE,nvh264enc:NONE,nvh264device0enc:NONE,nvv4l2h264enc:NONE,qsvh264enc:NONE,v4l2h264enc:NONE,x264enc:NONE,vah264dec:NONE,vah264lpdec:NONE,vaapih264dec:NONE,nvh264dec:NONE,nvv4l2decoder:NONE,qsvh264dec:NONE,v4l2h264dec:NONE,avdec_h264:NONE,vajpegdec:NONE,nvjpegdec:NONE,qsvjpegdec:NONE,vaapijpegdec:NONE"
    ;;
esac
export BENCH_ENC_GST BENCH_RCS_ENC BENCH_FFMPEG_ENC BENCH_FFMPEG_AV_OPTS

if [[ -n "${GST_PLUGIN_FEATURE_RANK:-}" ]]; then
  export GST_PLUGIN_FEATURE_RANK="${GST_PLUGIN_FEATURE_RANK%,},${_BENCH_RANK_DEMOTIONS}"
else
  export GST_PLUGIN_FEATURE_RANK="${_BENCH_RANK_DEMOTIONS}"
fi
unset _BENCH_RANK_DEMOTIONS
