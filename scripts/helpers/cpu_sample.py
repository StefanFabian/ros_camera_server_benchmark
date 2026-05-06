#!/usr/bin/env python3
"""Sample CPU usage of stack-under-test processes during a benchmark run.

Inputs: a set of session ids — one per `setsid`-spawned helper started during
the STACK phase of run_scenario.sh. Each tick we walk /proc, filter by
session id, and compute per-pid %CPU as
    delta_jiffies / clk_tck * interval_hz * 100
which matches the Linux/`top` convention (>100% on multi-thread saturation).

Output csv columns: tick_ns,sid,pid,comm,cpu_pct. tick_ns is the elapsed time
since sampler start, on the monotonic clock. New pids that appear mid-run
contribute a 0% row on their first observation (no prior sample to diff
against), then real deltas thereafter.
"""
import argparse
import csv
import os
import signal
import time
from pathlib import Path

CLK_TCK = os.sysconf("SC_CLK_TCK")


def read_stat(pid):
    """Return (comm, session_id, utime+stime jiffies) for pid, or None."""
    try:
        with open(f"/proc/{pid}/stat", "r") as f:
            data = f.read()
    except (OSError, FileNotFoundError):
        return None
    # comm is wrapped in parens and may itself contain spaces/parens. Split
    # around the *last* ')' so the suffix is whitespace-separated like the
    # proc(5) man page describes.
    rparen = data.rfind(")")
    if rparen < 0:
        return None
    comm = data[data.find("(") + 1:rparen]
    rest = data[rparen + 2:].split()
    if len(rest) < 13:
        return None
    try:
        session = int(rest[3])
        utime = int(rest[11])
        stime = int(rest[12])
    except ValueError:
        return None
    return comm, session, utime + stime


def scan(target_sids):
    out = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        info = read_stat(pid)
        if info is None:
            continue
        comm, sid, total = info
        if sid in target_sids:
            out[pid] = (comm, sid, total)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sids", required=True,
                    help="comma-separated session ids of stack-under-test")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--interval-hz", type=float, default=2.0)
    ap.add_argument("--duration", type=float, required=True)
    args = ap.parse_args()

    target_sids = {int(s) for s in args.sids.split(",") if s.strip()}
    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if not target_sids:
        # No stack PIDs (scenarios with empty STACK column). Header-only csv
        # so bench_aggregate.py can still read it without a special case.
        csv_path.write_text("tick_ns,sid,pid,comm,cpu_pct\n")
        return

    interval_s = 1.0 / args.interval_hz
    interval_ns = int(interval_s * 1e9)
    n_ticks = max(1, int(args.duration * args.interval_hz))

    # cleanup() in run_scenario.sh sends SIGTERM via the process group. Stop
    # the loop on the next iteration so the partial csv is flushed.
    stop_requested = {"flag": False}

    def handle_term(_signum, _frame):
        stop_requested["flag"] = True

    signal.signal(signal.SIGTERM, handle_term)
    signal.signal(signal.SIGINT, handle_term)

    # buffering=1 is line-buffered text mode — every tick is on disk before
    # the next one starts, so a kill mid-run still produces a usable csv.
    with csv_path.open("w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["tick_ns", "sid", "pid", "comm", "cpu_pct"])

        prev = scan(target_sids)
        t0 = time.monotonic_ns()
        for i in range(n_ticks):
            target_t = t0 + (i + 1) * interval_ns
            while not stop_requested["flag"]:
                now = time.monotonic_ns()
                if now >= target_t:
                    break
                # Sleep in small chunks so a SIGTERM is observed promptly.
                time.sleep(min(0.05, (target_t - now) / 1e9))
            if stop_requested["flag"]:
                break

            curr = scan(target_sids)
            tick_ns = time.monotonic_ns() - t0
            for pid, (comm, sid, total) in curr.items():
                prev_entry = prev.get(pid)
                prev_total = prev_entry[2] if prev_entry is not None else total
                delta = total - prev_total
                cpu_pct = delta / CLK_TCK * args.interval_hz * 100.0
                w.writerow([tick_ns, sid, pid, comm, f"{cpu_pct:.2f}"])
            prev = curr


if __name__ == "__main__":
    main()
