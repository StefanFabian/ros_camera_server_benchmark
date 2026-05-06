# Source me. Resolves BENCH_ENCODER ('va', 'nv', or 'mpp') exactly once and exports it.
#
# Probe vah264lpenc / nvh264enc / mpph264enc via gst-inspect-1.0 and pick:
#   * explicit BENCH_ENCODER if already set (validated)
#   * the only available backend if just one is installed
#   * interactive prompt on /dev/tty if more than one is installed
#
# Idempotent: re-sourcing with BENCH_ENCODER already set is a no-op (just
# re-validates). This lets run_all.sh resolve once at the start so the
# benchmark loop runs uninterrupted; per-scenario env_fairness.sh sourcing
# inherits the already-set value and skips the prompt.

if [[ -n "${BENCH_ENCODER:-}" ]]; then
  case "${BENCH_ENCODER}" in
    va|nv|mpp) export BENCH_ENCODER; return 0 ;;
    *) echo "[resolve_encoder] BENCH_ENCODER must be 'va', 'nv', or 'mpp', got '${BENCH_ENCODER}'" >&2; exit 2 ;;
  esac
fi

_BENCH_HAS_VA=0
_BENCH_HAS_NV=0
_BENCH_HAS_MPP=0
gst-inspect-1.0 vah264lpenc >/dev/null 2>&1 && _BENCH_HAS_VA=1
gst-inspect-1.0 nvh264enc   >/dev/null 2>&1 && _BENCH_HAS_NV=1
gst-inspect-1.0 mpph264enc  >/dev/null 2>&1 && _BENCH_HAS_MPP=1

_BENCH_AVAIL_COUNT=$(( _BENCH_HAS_VA + _BENCH_HAS_NV + _BENCH_HAS_MPP ))

if (( _BENCH_AVAIL_COUNT == 0 )); then
  echo "[resolve_encoder] none of vah264lpenc, nvh264enc, mpph264enc is installed." >&2
  echo "[resolve_encoder]   install gstreamer1.0-plugins-bad (VA / NV) or gstreamer1.0-rockchip (MPP)." >&2
  exit 2
elif (( _BENCH_AVAIL_COUNT == 1 )); then
  if   (( _BENCH_HAS_VA  == 1 )); then BENCH_ENCODER=va
  elif (( _BENCH_HAS_NV  == 1 )); then BENCH_ENCODER=nv
  else                                 BENCH_ENCODER=mpp
  fi
else
  _BENCH_OPTS=()
  (( _BENCH_HAS_VA  == 1 )) && _BENCH_OPTS+=(va)
  (( _BENCH_HAS_NV  == 1 )) && _BENCH_OPTS+=(nv)
  (( _BENCH_HAS_MPP == 1 )) && _BENCH_OPTS+=(mpp)
  _BENCH_PROMPT="$(IFS=/; echo "${_BENCH_OPTS[*]}")"
  echo "[resolve_encoder] multiple H.264 encoders available: ${_BENCH_OPTS[*]}." >&2
  echo "[resolve_encoder] Pick encoder [${_BENCH_PROMPT}]:" >&2
  echo "[resolve_encoder]   (set BENCH_ENCODER=va|nv|mpp to skip this prompt in scripts)" >&2
  read -r _BENCH_PICK </dev/tty
  case "${_BENCH_PICK}" in
    va|VA)   (( _BENCH_HAS_VA  == 1 )) && BENCH_ENCODER=va  || { echo "[resolve_encoder] 'va' not available" >&2; exit 2; } ;;
    nv|NV)   (( _BENCH_HAS_NV  == 1 )) && BENCH_ENCODER=nv  || { echo "[resolve_encoder] 'nv' not available" >&2; exit 2; } ;;
    mpp|MPP) (( _BENCH_HAS_MPP == 1 )) && BENCH_ENCODER=mpp || { echo "[resolve_encoder] 'mpp' not available" >&2; exit 2; } ;;
    *) echo "[resolve_encoder] invalid choice '${_BENCH_PICK}'" >&2; exit 2 ;;
  esac
  unset _BENCH_PICK _BENCH_OPTS _BENCH_PROMPT
fi
export BENCH_ENCODER
unset _BENCH_HAS_VA _BENCH_HAS_NV _BENCH_HAS_MPP _BENCH_AVAIL_COUNT
