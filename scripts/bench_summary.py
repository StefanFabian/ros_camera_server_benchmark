#!/usr/bin/env python3
"""Aggregate per-run summary.json files into a markdown table.

run_all.sh writes per-run results under results/run{1..N}/<scenario>/summary.json.
This script reads them all and prints a markdown table with one row per
(scenario, receiver): p50 median / min / max, p95 median, and per-stack
total CPU usage (mean / p95 of cpu_stack._total). Scenarios that didn't
get any data (count==0 across every receiver / every run) are skipped.
"""
import argparse
import json
import statistics
import sys
from pathlib import Path


def median(xs):
    return statistics.median(xs) if xs else None


HEADERS = ("scenario", "recv", "p50_med", "p50_min", "p50_max",
           "p95_med", "cpu_mean", "cpu_p95")
NUMERIC_COLS = {"p50_med", "p50_min", "p50_max", "p95_med",
                "cpu_mean", "cpu_p95"}


def fmt(v):
    if v is None:
        return "—"
    return f"{v:.2f}"


def render_markdown(rows):
    formatted = []
    for r in rows:
        is_rcs = "-rcs-" in r["scenario"]
        row = {}
        for h in HEADERS:
            text = r[h] if h in ("scenario", "recv") else fmt(r[h])
            if is_rcs:
                text = f"**{text}**"
            row[h] = text
        formatted.append(row)

    widths = {h: len(h) for h in HEADERS}
    for row in formatted:
        for h in HEADERS:
            widths[h] = max(widths[h], len(row[h]))

    def cell(text, width, right):
        return text.rjust(width) if right else text.ljust(width)

    lines = []
    header = "| " + " | ".join(cell(h, widths[h], h in NUMERIC_COLS) for h in HEADERS) + " |"
    sep = "| " + " | ".join(
        ("-" * (widths[h] - 1) + ":") if h in NUMERIC_COLS else ("-" * widths[h])
        for h in HEADERS
    ) + " |"
    lines.append(header)
    lines.append(sep)
    for row in formatted:
        lines.append("| " + " | ".join(cell(row[h], widths[h], h in NUMERIC_COLS)
                                       for h in HEADERS) + " |")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--results-root",
        type=Path,
        default=Path("results"),
        help="directory containing run1/ run2/ ...",
    )
    ap.add_argument(
        "--receivers",
        nargs="+",
        default=["stream", "ros_raw", "ros_jpeg"],
        help="receiver kinds to report",
    )
    args = ap.parse_args()

    runs = sorted(p for p in args.results_root.glob("run*") if p.is_dir())
    if not runs:
        print(f"no run* directories under {args.results_root}", file=sys.stderr)
        return 2

    # per-scenario accumulator: scenario -> receiver -> [summary dicts...]
    acc: dict[str, dict[str, list[dict]]] = {}
    # per-scenario cpu_stack._total samples across runs
    cpu_acc: dict[str, dict[str, list[float]]] = {}
    # producer underruns observed per (scenario, run): scenario -> [counts...]
    underruns: dict[str, list[int]] = {}
    for run in runs:
        for sc_dir in sorted(run.iterdir()):
            if not sc_dir.is_dir():
                continue
            summary = sc_dir / "summary.json"
            if not summary.exists():
                continue
            try:
                data = json.loads(summary.read_text())
            except json.JSONDecodeError:
                print(f"warning: bad json {summary}", file=sys.stderr)
                continue
            scenario = data.get("scenario", sc_dir.name)
            for recv in args.receivers:
                d = data.get(recv, {})
                if not d or d.get("count") in (None, 0):
                    continue
                acc.setdefault(scenario, {}).setdefault(recv, []).append(d)
            total = ((data.get("cpu_stack") or {}).get("_total") or {})
            cb = cpu_acc.setdefault(scenario, {"mean": [], "p95": []})
            if total.get("mean_pct") is not None:
                cb["mean"].append(total["mean_pct"])
            if total.get("p95_pct") is not None:
                cb["p95"].append(total["p95_pct"])
            producer = data.get("producer") or {}
            u = producer.get("underruns") or 0
            if isinstance(u, (int, float)) and u > 0:
                underruns.setdefault(scenario, []).append(int(u))

    rows = []
    for scenario in sorted(acc):
        cb = cpu_acc.get(scenario, {"mean": [], "p95": []})
        cpu_mean = median(cb["mean"])
        cpu_p95 = median(cb["p95"])
        for recv, recv_rows in acc[scenario].items():
            p50s = [r["p50_ms"] for r in recv_rows if r.get("p50_ms") is not None]
            p95s = [r["p95_ms"] for r in recv_rows if r.get("p95_ms") is not None]
            if not p50s:
                continue
            rows.append({
                "scenario": scenario,
                "recv": recv,
                "p50_med": median(p50s),
                "p50_min": min(p50s),
                "p50_max": max(p50s),
                "p95_med": median(p95s),
                "cpu_mean": cpu_mean,
                "cpu_p95": cpu_p95,
            })

    if rows:
        print(render_markdown(rows))

    if underruns:
        print("\nproducer underruns (frame_gen missed deadline; fps degraded):",
              file=sys.stderr)
        for scenario in sorted(underruns):
            counts = underruns[scenario]
            print(f"  {scenario}: {len(counts)}/{len(runs)} runs affected, "
                  f"counts={counts}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
