# Source me. Warns once per invocation when the CPU governor is not
# 'performance' — frequency scaling adds wake-up jitter to receiver
# dispatch and noise to cpu.csv. No-op when BENCH_SKIP_GOVERNOR_CHECK=1
# (user accepts the noise) or BENCH_GOVERNOR_CHECKED=1 (run_all.sh
# already warned; per-scenario subprocesses inherit it). Silent skip
# when /sys/.../cpufreq is unavailable (containers, some VMs) — absence
# isn't actionable for the user.
[[ -n "${BENCH_SKIP_GOVERNOR_CHECK:-}" ]] && return 0
[[ -n "${BENCH_GOVERNOR_CHECKED:-}"    ]] && return 0

shopt -s nullglob
_bench_gov_files=(/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor)
shopt -u nullglob

if (( ${#_bench_gov_files[@]} == 0 )); then
  unset _bench_gov_files
  export BENCH_GOVERNOR_CHECKED=1
  return 0
fi

mapfile -t _bench_govs < <(cat "${_bench_gov_files[@]}" 2>/dev/null | sort -u)
_bench_bad=()
for g in "${_bench_govs[@]}"; do
  [[ "$g" != "performance" ]] && _bench_bad+=("$g")
done

if (( ${#_bench_bad[@]} > 0 )); then
  echo "[bench] WARNING: CPU governor is '${_bench_bad[*]}' (not 'performance')." >&2
  echo "[bench]   Frequency scaling adds latency jitter; numbers will be noisier." >&2
  echo "[bench]   Fix:     sudo cpupower frequency-set -g performance" >&2
  echo "[bench]   Silence: export BENCH_SKIP_GOVERNOR_CHECK=1" >&2
fi

unset _bench_gov_files _bench_govs _bench_bad g
export BENCH_GOVERNOR_CHECKED=1
