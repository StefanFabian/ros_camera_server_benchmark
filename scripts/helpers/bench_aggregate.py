#!/usr/bin/env python3
"""Aggregate per-frame latency CSVs into summary.json + append a row to report.csv.

CSV schema (produced by gst_recv / webrtc_recv / ros_recv):
    frame_id,send_ts_ns,recv_ts_ns,latency_ns

Per-scenario inputs in <output-dir>:
    stream.csv          - encoded-stream receiver (gst_recv/webrtc_recv).
                          Present in G2/G3/G4.
    ros_raw.csv         - raw ROS topic receiver (G1, G4).
    ros_jpeg.csv        - compressed ROS topic receiver (G1, G4).
    frame_gen_stats.csv - frame_gen producer counters (deadline misses,
                          encode_total_ns, encode_count, ...).
    ros_pub_stats.csv   - ros_pub producer counters (encode_total_ns,
                          encode_count). Present in ros-stream-* runs.

Producer-side encode time (i420->wire-format conversion in frame_gen /
ros_pub) is bracketed by the per-frame timestamp and so is included in
the receiver's latency_ns. The aggregator subtracts encode_total_ns /
encode_count from each latency before computing percentiles so YUV vs
JPEG comparisons reflect SUT cost rather than harness overhead. Raw
percentiles are preserved as `*_ms_raw` for sanity checking.

Output summary fields per receiver:
    count, mean_ms, stddev_ms, p50_ms, p95_ms, p99_ms, max_ms,
    expected, frame_loss, fps_effective

Empty receivers (count == 0) are omitted from report.csv. If a scenario's
*expected* receivers (per-group classification below) all came back empty,
the script exits non-zero so run_scenario.sh fails the run instead of
silently producing an empty row.

Note: warmup is enforced at the receiver (rows below `warmup-frames` are not
written). This script does no further trimming.
"""
import argparse
import csv
import json
import os
import re
import statistics
import sys
from pathlib import Path

RECEIVERS = ("stream", "ros_raw", "ros_jpeg")

# Per-group receiver expectations. Group is parsed off the leading
# `{cam-ros|cam-stream|ros-stream|cam-both}-...` prefix of the scenario id;
# see scripts/run_all.sh for the full matrix.
#   cam-ros    — Camera -> ROS topic only.        Requires one ros_*; no stream.
#   cam-stream — Camera -> stream only.           Requires stream; no ros_*.
#   ros-stream — ROS image -> stream.             Requires stream; no ros_*.
#   cam-both   — Camera -> stream AND ros image.  Requires both: stream + one ros_*.
GROUP_REQUIREMENTS = {
    "cam-ros":    {"required": [("ros_raw", "ros_jpeg")], "stream_required": False},
    "cam-stream": {"required": [], "stream_required": True},
    "ros-stream": {"required": [], "stream_required": True},
    "cam-both":   {"required": [("ros_raw", "ros_jpeg")], "stream_required": True},
}

SCENARIO_ID_RE = re.compile(
    r"^(cam-ros|cam-stream|ros-stream|cam-both)-[A-Za-z0-9_+-]+$")


def load_csv(path: Path):
    rows = []
    if not path.exists():
        return rows
    with path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                rows.append({
                    "frame_id": int(row["frame_id"]),
                    "send_ts_ns": int(row["send_ts_ns"]),
                    "recv_ts_ns": int(row["recv_ts_ns"]),
                    "latency_ns": int(row["latency_ns"]),
                })
            except (KeyError, ValueError):
                continue
    return rows


def percentile(sorted_vals, p):
    if not sorted_vals:
        return None
    k = (len(sorted_vals) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = k - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def load_recv_stats(csv_path: Path):
    """Read the `<csv>.stats` sidecar written by RecvLoop's destructor.

    Returns dict with `seen` and `decoded` keys (ints), or None if missing or
    malformed. The CSV only contains successfully-decoded frames (RecvLoop
    skips writes on marker CRC/magic failures), so the sidecar is the only
    place where silent marker-decode failures surface.
    """
    stats_path = Path(str(csv_path) + ".stats")
    if not stats_path.exists():
        return None
    out = {}
    try:
        for line in stats_path.read_text().splitlines():
            line = line.strip()
            if not line or "=" not in line:
                continue
            k, v = line.split("=", 1)
            try:
                out[k.strip()] = int(v.strip())
            except ValueError:
                continue
    except OSError:
        return None
    return out or None


def summarise(rows, expected_after_warmup, fps, recv_stats=None,
              producer_encode_mean_ms=0.0):
    out = {
        "count": len(rows),
        "expected": expected_after_warmup,
        "frame_loss": None,
        "fps_effective": None,
        "mean_ms": None, "stddev_ms": None,
        "p50_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None,
        # Pre-subtraction percentiles. `*_ms` reports the SUT-attributable
        # number (latency minus producer-side harness encode); `*_ms_raw` is
        # the unadjusted wire latency for sanity checking.
        "p50_ms_raw": None, "p95_ms_raw": None, "p99_ms_raw": None,
        "mean_ms_raw": None, "max_ms_raw": None,
        "producer_encode_mean_ms": producer_encode_mean_ms,
        # gap_count isolates steady-state drops from startup/teardown trim.
        # frame_loss collapses both into one number; gap_count vs (expected -
        # count) tells you which is which.
        "first_id": None, "last_id": None, "gap_count": None,
        # seen / decoded come from the receiver's `<csv>.stats` sidecar.
        # decoded_ratio < 1.0 indicates the marker silently failed to decode
        # on some frames (codec quantization, frame corruption, etc.); those
        # frames are absent from the latency CSV entirely and would otherwise
        # be invisible in the percentile numbers.
        "seen": None, "decoded": None, "decoded_ratio": None,
    }
    if recv_stats is not None:
        out["seen"] = recv_stats.get("seen")
        out["decoded"] = recv_stats.get("decoded")
        if (out["seen"] is not None and out["decoded"] is not None
                and out["seen"] > 0):
            out["decoded_ratio"] = out["decoded"] / out["seen"]
    if not rows:
        return out
    latencies_ms = sorted(r["latency_ns"] / 1e6 for r in rows)
    out["mean_ms_raw"] = statistics.fmean(latencies_ms)
    out["stddev_ms"] = statistics.pstdev(latencies_ms) if len(latencies_ms) > 1 else 0.0
    out["p50_ms_raw"] = percentile(latencies_ms, 0.50)
    out["p95_ms_raw"] = percentile(latencies_ms, 0.95)
    out["p99_ms_raw"] = percentile(latencies_ms, 0.99)
    out["max_ms_raw"] = latencies_ms[-1]
    # Subtract the producer-side harness encode (a per-run scalar) so the
    # primary p50/p95/etc. reflect SUT latency. With a constant offset the
    # subtraction commutes with percentile, so applying it after sorting is
    # equivalent and cheaper than re-deriving over a shifted list.
    adj = producer_encode_mean_ms
    out["mean_ms"] = out["mean_ms_raw"] - adj
    out["p50_ms"] = out["p50_ms_raw"] - adj
    out["p95_ms"] = out["p95_ms_raw"] - adj
    out["p99_ms"] = out["p99_ms_raw"] - adj
    out["max_ms"] = out["max_ms_raw"] - adj
    if expected_after_warmup > 0:
        out["frame_loss"] = max(0.0, 1.0 - len(rows) / expected_after_warmup)
    span_s = (rows[-1]["recv_ts_ns"] - rows[0]["recv_ts_ns"]) / 1e9
    if span_s > 0:
        out["fps_effective"] = (len(rows) - 1) / span_s
    else:
        out["fps_effective"] = float(fps)
    # UDP-RTP on localhost and ROS-reliable preserve order, so id_min..id_max
    # defines the observed window. Anything missing inside that window is a
    # steady-state drop; missing rows before id_min / after id_max are
    # startup/teardown.
    ids = [r["frame_id"] for r in rows]
    id_min = min(ids)
    id_max = max(ids)
    out["first_id"] = id_min
    out["last_id"] = id_max
    out["gap_count"] = max(0, (id_max - id_min + 1) - len(rows))
    return out


def load_cpu_stack(csv_path: Path, warmup_s: float):
    """Aggregate cpu_sample.py output by `comm` (process basename).

    Returns dict comm -> {"mean_pct", "p95_pct", "n"}, plus a synthetic
    "_total" key with summed means across comms (per-tick total, then
    averaged). Rows below `warmup_s` of monotonic time are dropped so
    library startup spikes don't dominate; this mirrors the per-receiver
    warmup-frames trim.
    """
    if not csv_path.exists():
        return None
    warmup_ns = int(warmup_s * 1e9)
    by_comm = {}
    by_tick = {}
    try:
        with csv_path.open() as f:
            r = csv.DictReader(f)
            for row in r:
                try:
                    tick = int(row["tick_ns"])
                    cpu = float(row["cpu_pct"])
                except (KeyError, ValueError):
                    continue
                if tick < warmup_ns:
                    continue
                comm = row.get("comm", "")
                by_comm.setdefault(comm, []).append(cpu)
                by_tick.setdefault(tick, 0.0)
                by_tick[tick] += cpu
    except OSError:
        return None
    if not by_comm:
        return None
    out = {}
    for comm, vals in by_comm.items():
        s = sorted(vals)
        out[comm] = {
            "mean_pct": statistics.fmean(vals),
            "p95_pct": percentile(s, 0.95),
            "n": len(vals),
        }
    if by_tick:
        tick_totals = sorted(by_tick.values())
        out["_total"] = {
            "mean_pct": statistics.fmean(tick_totals),
            "p95_pct": percentile(tick_totals, 0.95),
            "n": len(tick_totals),
        }
    return out


def load_producer_stats(path: Path):
    if not path.exists():
        return None
    stats = {}
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" in line:
                k, v = line.split("=", 1)
                k = k.strip()
                v = v.strip()
                try:
                    stats[k] = int(v)
                except ValueError:
                    try:
                        stats[k] = float(v)
                    except ValueError:
                        stats[k] = v
    return stats or None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--duration", type=int, required=True)
    ap.add_argument("--warmup-frames", type=int, required=True)
    ap.add_argument("--fps", type=int, required=True)
    ap.add_argument("--report-csv", required=True)
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    expected_total = args.duration * args.fps
    expected_after = max(0, expected_total - args.warmup_frames)

    summary = {
        "scenario": args.scenario,
        "duration_s": args.duration,
        "warmup_frames": args.warmup_frames,
        "expected_after_warmup": expected_after,
    }

    # Producer encode time is per-run and shared across receivers; load it
    # before summarising so each receiver sees the same subtraction. Either
    # frame_gen (cam-*) or ros_pub (ros-stream-*) writes one of these; both
    # use the same key=value schema.
    producer = load_producer_stats(out_dir / "frame_gen_stats.csv")
    if producer is None:
        producer = load_producer_stats(out_dir / "ros_pub_stats.csv")
    encode_mean_ms = 0.0
    if producer is not None:
        ec = producer.get("encode_count") or 0
        et = producer.get("encode_total_ns") or 0
        if isinstance(ec, (int, float)) and isinstance(et, (int, float)) and ec > 0:
            encode_mean_ms = (et / ec) / 1e6

    for kind in RECEIVERS:
        path = out_dir / f"{kind}.csv"
        rows = load_csv(path)
        recv_stats = load_recv_stats(path)
        summary[kind] = summarise(rows, expected_after, args.fps, recv_stats,
                                  encode_mean_ms)

    cpu_stack = load_cpu_stack(out_dir / "cpu.csv",
                               warmup_s=args.warmup_frames / max(1, args.fps))
    if cpu_stack is not None:
        summary["cpu_stack"] = cpu_stack

    producer_underruns = 0
    if producer is not None:
        summary["producer"] = producer
        # Any non-zero underrun means frame_gen missed a 30 fps deadline at
        # least once. The run is still scientifically valid (latency rows are
        # individually accurate) but cross-stack comparisons are no longer at
        # a fair 30 fps. Surface as a warning so it can't slip past a casual
        # report.csv scan.
        u = producer.get("underruns")
        if isinstance(u, (int, float)) and u > 0:
            producer_underruns = int(u)
            summary["producer_warning"] = (
                f"frame_gen reported {producer_underruns} underrun(s); "
                "fps degraded below target")

    # Surface marker-decode regressions in summary.json. A ratio below 1.0
    # means some frames arrived with a corrupted marker — they are missing
    # from the latency percentiles and would otherwise be invisible.
    decode_warnings = []
    for kind in RECEIVERS:
        s = summary[kind]
        r = s.get("decoded_ratio")
        if r is not None and r < 0.995 and (s.get("seen") or 0) > 0:
            decode_warnings.append(
                f"{kind}: decoded {s['decoded']}/{s['seen']} "
                f"({r * 100:.1f}%)")
    if decode_warnings:
        summary["marker_decode_warning"] = (
            "marker decoded on <99.5% of seen frames: "
            + "; ".join(decode_warnings))

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    # Append flat rows to report.csv
    report_path = Path(args.report_csv)
    new = not report_path.exists()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow([
                "scenario", "receiver", "count", "expected", "frame_loss",
                "fps_effective", "mean_ms", "stddev_ms",
                "p50_ms", "p95_ms", "p99_ms", "max_ms",
                "p50_ms_raw", "producer_encode_mean_ms",
                "producer_underruns",
                "seen", "decoded", "decoded_ratio",
                "cpu_stack_total_mean_pct", "cpu_stack_total_p95_pct",
            ])
        cpu_total = (cpu_stack or {}).get("_total")
        cpu_mean = f"{cpu_total['mean_pct']:.2f}" if cpu_total else ""
        cpu_p95 = f"{cpu_total['p95_pct']:.2f}" if cpu_total else ""
        for kind in RECEIVERS:
            s = summary[kind]
            if s["count"] == 0:
                continue
            w.writerow([
                args.scenario, kind, s["count"], s["expected"],
                f"{s['frame_loss']:.4f}" if s["frame_loss"] is not None else "",
                f"{s['fps_effective']:.2f}" if s["fps_effective"] is not None else "",
                f"{s['mean_ms']:.3f}", f"{s['stddev_ms']:.3f}",
                f"{s['p50_ms']:.3f}", f"{s['p95_ms']:.3f}",
                f"{s['p99_ms']:.3f}", f"{s['max_ms']:.3f}",
                f"{s['p50_ms_raw']:.3f}" if s["p50_ms_raw"] is not None else "",
                f"{s['producer_encode_mean_ms']:.3f}",
                producer_underruns,
                s["seen"] if s["seen"] is not None else "",
                s["decoded"] if s["decoded"] is not None else "",
                f"{s['decoded_ratio']:.4f}" if s["decoded_ratio"] is not None else "",
                cpu_mean, cpu_p95,
            ])

    # One-line per-receiver status. Full numbers live in summary.json /
    # report.csv; dumping the whole JSON to stdout drowned the actual
    # per-scenario verdict (which receivers had data, which were empty).
    parts = []
    for kind in RECEIVERS:
        s = summary[kind]
        if s["count"] == 0:
            continue
        parts.append(
            f"{kind} count={s['count']}/{s['expected']} "
            f"p50={s['p50_ms']:.2f}ms p95={s['p95_ms']:.2f}ms")
    print(f"[bench_aggregate {args.scenario}] {'; '.join(parts) if parts else 'no receiver produced rows'}")

    # Receiver-coverage check: surface scenarios that produced no usable data
    # so silently empty CSVs don't reach the report.
    errors = []
    m = SCENARIO_ID_RE.match(args.scenario)
    if not m:
        print(f"[bench_aggregate] warning: unknown scenario id {args.scenario!r}",
              file=sys.stderr)
    else:
        group = m.group(1)
        req = GROUP_REQUIREMENTS[group]
        if req["stream_required"] and summary["stream"]["count"] == 0:
            errors.append("stream receiver produced 0 rows")
        for option_set in req["required"]:
            # option_set is a tuple of receiver names; at least one must be
            # populated for the scenario to count.
            if not any(summary[r]["count"] > 0 for r in option_set):
                errors.append(
                    f"{'/'.join(option_set)} all produced 0 rows")
    if errors:
        for e in errors:
            print(f"[bench_aggregate] ERROR ({args.scenario}): {e}",
                  file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
