#!/usr/bin/env python3
"""Render per-group bar charts from benchmark summary.json files.

Walks `results/run*/<scenario>/summary.json` (the layout produced by
`run_all.sh`), aggregates p50 / p95 latency and total CPU usage across
runs, and writes one PNG per scenario group:

    <output-dir>/cam-ros.png
    <output-dir>/cam-stream.png
    <output-dir>/ros-stream.png
    <output-dir>/cam-both.png

Each chart has two stacked panels sharing the x-axis. Top panel shows
p50 (solid) and p95 (faded overlay) latency per receiver; error bars
span the per-run p50 min..max. Bottom panel shows the per-stack total
CPU usage (mean, p95 as faded overlay) summed across all comms.

Within cam-both each stack contributes both a stream and a ROS bar
drawn side-by-side; stacks are sorted by stream p50 ascending. Other
groups have one bar per scenario sorted by p50 ascending.
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RECEIVERS = ("stream", "ros_raw", "ros_jpeg")

GROUP_TITLES = {
    "cam-ros":    "Camera → ROS topic",
    "cam-stream": "Camera → encoded stream",
    "ros-stream": "ROS image → encoded stream",
    "cam-both":   "Camera → stream + ROS topic",
}

GROUP_RE = re.compile(r"^(cam-ros|cam-stream|ros-stream|cam-both)-")

RECEIVER_COLORS = {
    "stream":   "#1f77b4",
    "ros_raw":  "#2ca02c",
    "ros_jpeg": "#ff7f0e",
}

CPU_COLOR = "#9467bd"
P95_ALPHA = 0.25
INTRA_STACK_GAP = 0.0
INTER_STACK_GAP = 0.6
BAR_WIDTH = 0.8

# Font sizes (points). Enlarged so the figures stay legible when placed at
# full A4 width in a paper. Tweak here to scale all text together.
BASE_FONTSIZE = 13       # default text / fallback
TITLE_FONTSIZE = 17      # group title (top, bold)
SUBTITLE_FONTSIZE = 14   # descriptor line under the title
AXIS_LABEL_FONTSIZE = 16
TICK_FONTSIZE = 14
VALUE_FONTSIZE = 12      # numeric labels drawn on top of bars
LEGEND_FONTSIZE = 13


def _apply_font_sizes() -> None:
    """Bump matplotlib's base font sizes for paper-ready figures."""
    plt.rcParams.update({
        "font.size": BASE_FONTSIZE,
        "axes.titlesize": SUBTITLE_FONTSIZE,
        "axes.labelsize": AXIS_LABEL_FONTSIZE,
        "xtick.labelsize": TICK_FONTSIZE,
        "ytick.labelsize": TICK_FONTSIZE,
        "legend.fontsize": LEGEND_FONTSIZE,
        "figure.titlesize": TITLE_FONTSIZE,
    })


def median(xs):
    return statistics.median(xs) if xs else None


def parse_group(scenario: str) -> str | None:
    m = GROUP_RE.match(scenario)
    return m.group(1) if m else None


def collect(results_root: Path):
    """Walk run*/<scenario>/summary.json. Return:
        acc[group][scenario][receiver] = {"p50": [...], "p95": [...], "count": [...]}
        cpu[group][scenario] = {"mean": [...], "p95": [...]}  # cpu_stack._total
    """
    runs = sorted(p for p in results_root.glob("run*") if p.is_dir())
    if not runs:
        print(f"no run* directories under {results_root}", file=sys.stderr)
        return {}, {}, []

    acc: dict[str, dict[str, dict[str, dict[str, list]]]] = {}
    cpu: dict[str, dict[str, dict[str, list]]] = {}
    for run in runs:
        for sc_dir in sorted(run.iterdir()):
            if not sc_dir.is_dir():
                continue
            sj = sc_dir / "summary.json"
            if not sj.exists():
                continue
            try:
                data = json.loads(sj.read_text())
            except json.JSONDecodeError:
                print(f"warning: bad json {sj}", file=sys.stderr)
                continue
            scenario = data.get("scenario", sc_dir.name)
            group = parse_group(scenario)
            if group is None:
                continue
            for recv in RECEIVERS:
                d = data.get(recv) or {}
                if not d.get("count"):
                    continue
                bucket = (acc.setdefault(group, {})
                             .setdefault(scenario, {})
                             .setdefault(recv, {"p50": [], "p95": [], "count": []}))
                if d.get("p50_ms") is not None:
                    bucket["p50"].append(d["p50_ms"])
                if d.get("p95_ms") is not None:
                    bucket["p95"].append(d["p95_ms"])
                if d.get("count") is not None:
                    bucket["count"].append(d["count"])

            total = ((data.get("cpu_stack") or {}).get("_total") or {})
            cb = cpu.setdefault(group, {}).setdefault(scenario,
                                                     {"mean": [], "p95": []})
            if total.get("mean_pct") is not None:
                cb["mean"].append(total["mean_pct"])
            if total.get("p95_pct") is not None:
                cb["p95"].append(total["p95_pct"])
    return acc, cpu, runs


def receiver_rows(group_data: dict, scenario: str):
    """List (receiver, p50_med, p50_min, p50_max, p95_med, n_runs) for a scenario."""
    out = []
    recvs = group_data.get(scenario, {})
    for recv in RECEIVERS:
        d = recvs.get(recv)
        if not d or not d["p50"]:
            continue
        out.append((
            recv,
            median(d["p50"]),
            min(d["p50"]),
            max(d["p50"]),
            median(d["p95"]) if d["p95"] else None,
            len(d["p50"]),
        ))
    return out


def build_bars(group: str, group_data: dict):
    """Decide x positions and per-bar metadata for one group.

    cam-both: each scenario produces a stack of (stream + ros_*) bars
    drawn side-by-side; stacks sorted by stream p50, alpha-fall-back when
    stream is missing. Other groups: one bar per (scenario, receiver),
    sorted by p50.

    Returns (bars, stack_centers, stack_labels) where bars is a list of
    dicts with keys: x, label, recv, p50, p50_lo, p50_hi, p95, scenario.
    stack_centers/labels parallel arrays for x-axis stack labels (cam-both
    only — empty list otherwise).
    """
    scenarios = list(group_data.keys())

    if group == "cam-both":
        def stack_sort_key(sc):
            recvs = group_data[sc]
            stream = recvs.get("stream")
            if stream and stream["p50"]:
                return (0, median(stream["p50"]), sc)
            for r in ("ros_raw", "ros_jpeg"):
                d = recvs.get(r)
                if d and d["p50"]:
                    return (1, median(d["p50"]), sc)
            return (2, float("inf"), sc)

        scenarios.sort(key=stack_sort_key)

        bars = []
        stack_centers = []
        stack_labels = []
        x = 0.0
        for sc in scenarios:
            rrs = receiver_rows(group_data, sc)
            if not rrs:
                continue
            xs_in_stack = []
            for recv, p50, p50_min, p50_max, p95, n in rrs:
                bars.append({
                    "x": x, "label": recv, "recv": recv,
                    "p50": p50, "p50_lo": p50 - p50_min, "p50_hi": p50_max - p50,
                    "p95": p95, "scenario": sc,
                })
                xs_in_stack.append(x)
                x += BAR_WIDTH + INTRA_STACK_GAP
            if xs_in_stack:
                stack_centers.append(sum(xs_in_stack) / len(xs_in_stack))
                stack_labels.append(_stack_label(group, sc))
            x += INTER_STACK_GAP
        return bars, stack_centers, stack_labels

    # Single-receiver groups: flatten and sort by p50.
    flat = []
    for sc in scenarios:
        for r in receiver_rows(group_data, sc):
            recv, p50, p50_min, p50_max, p95, n = r
            flat.append({
                "label": _short_scenario(group, sc),
                "recv": recv,
                "p50": p50, "p50_lo": p50 - p50_min, "p50_hi": p50_max - p50,
                "p95": p95, "scenario": sc,
            })
    flat.sort(key=lambda b: (b["p50"], b["label"]))
    bars = []
    x = 0.0
    for b in flat:
        b["x"] = x
        bars.append(b)
        x += BAR_WIDTH + INTER_STACK_GAP
    return bars, [], []


def _short_scenario(group: str, scenario: str) -> str:
    """Drop the group prefix for tighter x-tick labels."""
    prefix = f"{group}-"
    return scenario[len(prefix):] if scenario.startswith(prefix) else scenario


def _stack_label(group: str, scenario: str) -> str:
    """x-tick label for a cam-both stack, split over two lines as
    PIPELINE / INPUT-TRANSPORT (e.g. 'rcs' / 'mjpeg-decoded-rtp').

    Scenario after the group prefix is PIPELINE-INPUT-TRANSPORT where
    PIPELINE is the first token (may contain '+'), TRANSPORT the last, and
    INPUT everything in between (one or two tokens, e.g. 'mjpeg-decoded').
    """
    short = _short_scenario(group, scenario)
    parts = short.split("-")
    if len(parts) < 3:
        return short
    pipeline, transport = parts[0], parts[-1]
    inp = "-".join(parts[1:-1])
    return f"{pipeline}\n{inp}-{transport}"


def plot_group(group: str, group_data: dict, group_cpu: dict,
               n_runs: int, out_path: Path) -> bool:
    bars, stack_centers, stack_labels = build_bars(group, group_data)
    if not bars:
        print(f"skip {group}: no data", file=sys.stderr)
        return False

    n_bars = len(bars)
    fig_w = max(8.0, 0.55 * n_bars + 4.0)
    fig, (ax, ax_cpu) = plt.subplots(
        2, 1, figsize=(fig_w, 7.5), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )

    xs = np.array([b["x"] for b in bars])
    p50 = np.array([b["p50"] for b in bars])
    p50_lo = np.array([b["p50_lo"] for b in bars])
    p50_hi = np.array([b["p50_hi"] for b in bars])
    p95 = np.array([b["p95"] if b["p95"] is not None else np.nan for b in bars])
    colors = [RECEIVER_COLORS.get(b["recv"], "#888") for b in bars]

    ax.bar(xs, p95, width=BAR_WIDTH, color=colors, alpha=P95_ALPHA,
           edgecolor="none", zorder=1)
    ax.bar(xs, p50, width=BAR_WIDTH, color=colors, edgecolor="black",
           linewidth=0.4,
           yerr=np.vstack([p50_lo, p50_hi]),
           error_kw={"ecolor": "black", "elinewidth": 0.8, "capsize": 3},
           zorder=2)
    for xi, v in zip(xs, p50):
        ax.text(xi, v, f"{v:.1f}", ha="center", va="bottom",
                fontsize=VALUE_FONTSIZE, zorder=3)
    ax.set_ylabel("Latency (ms)", fontsize=AXIS_LABEL_FONTSIZE)
    # Subtitle pinned just above the axes; main title raised above it by `pad`.
    # Both anchored to the axes top so the layout engine spaces them and the
    # gap to the plot stays constant regardless of figure size.
    ax.annotate(f"bars: p50 median (per-run min..max), "
                f"faded: p95 median  N={n_runs} run(s)",
                xy=(0.5, 1.0), xycoords="axes fraction",
                xytext=(0, 6), textcoords="offset points",
                ha="center", va="bottom",
                fontsize=SUBTITLE_FONTSIZE, color="#444")
    ax.set_title(GROUP_TITLES.get(group, group),
                 fontsize=TITLE_FONTSIZE, fontweight="bold", pad=28)
    ax.grid(axis="y", linestyle=":", alpha=0.5, zorder=0)
    ax.set_axisbelow(True)

    used_recvs = sorted({b["recv"] for b in bars})
    if len(used_recvs) > 1:
        from matplotlib.patches import Patch
        handles = [Patch(facecolor=RECEIVER_COLORS[r], edgecolor="black",
                         linewidth=0.4, label=r) for r in used_recvs]
        ax.legend(handles=handles, loc="upper left", fontsize=LEGEND_FONTSIZE,
                  framealpha=0.9, title="receiver",
                  title_fontsize=LEGEND_FONTSIZE)

    # CPU panel — one bar per scenario (per stack in cam-both), placed at
    # the centre of that stack's latency bars and widened to span them.
    cpu_xs, cpu_widths, cpu_mean, cpu_p95 = [], [], [], []
    if group == "cam-both":
        # Group bar positions by scenario.
        by_sc: dict[str, list[float]] = {}
        for b in bars:
            by_sc.setdefault(b["scenario"], []).append(b["x"])
        for sc, positions in by_sc.items():
            d = group_cpu.get(sc) or {}
            if not d.get("mean"):
                continue
            cpu_xs.append(sum(positions) / len(positions))
            span = (max(positions) - min(positions)) + BAR_WIDTH
            cpu_widths.append(span)
            cpu_mean.append(median(d["mean"]))
            cpu_p95.append(median(d["p95"]) if d["p95"] else np.nan)
    else:
        for b in bars:
            d = group_cpu.get(b["scenario"]) or {}
            if not d.get("mean"):
                continue
            cpu_xs.append(b["x"])
            cpu_widths.append(BAR_WIDTH)
            cpu_mean.append(median(d["mean"]))
            cpu_p95.append(median(d["p95"]) if d["p95"] else np.nan)

    if cpu_xs:
        cpu_xs_a = np.array(cpu_xs)
        cpu_w_a = np.array(cpu_widths)
        cpu_mean_a = np.array(cpu_mean)
        cpu_p95_a = np.array(cpu_p95)
        ax_cpu.bar(cpu_xs_a, cpu_p95_a, width=cpu_w_a, color=CPU_COLOR,
                   alpha=P95_ALPHA, edgecolor="none", zorder=1)
        ax_cpu.bar(cpu_xs_a, cpu_mean_a, width=cpu_w_a, color=CPU_COLOR,
                   edgecolor="black", linewidth=0.4, zorder=2)
        for xi, v in zip(cpu_xs_a, cpu_mean_a):
            ax_cpu.text(xi, v, f"{v:.0f}", ha="center", va="bottom",
                        fontsize=VALUE_FONTSIZE, zorder=3)
    else:
        ax_cpu.text(0.5, 0.5, "no cpu_stack data", ha="center", va="center",
                    transform=ax_cpu.transAxes, fontsize=VALUE_FONTSIZE,
                    color="#666")

    ax_cpu.set_ylabel("CPU (%)", fontsize=AXIS_LABEL_FONTSIZE)
    ax_cpu.grid(axis="y", linestyle=":", alpha=0.5, zorder=0)
    ax_cpu.set_axisbelow(True)

    # x-tick labels: cam-both gets two-line PIPELINE / INPUT-TRANSPORT labels
    # (one per stack); other groups keep one rotated label per bar.
    if group == "cam-both" and stack_centers:
        ax_cpu.set_xticks(stack_centers)
        tick_labels = ax_cpu.set_xticklabels(stack_labels, rotation=45,
                                             ha="right", rotation_mode="anchor",
                                             fontsize=TICK_FONTSIZE)
    else:
        ax_cpu.set_xticks(xs)
        tick_labels = ax_cpu.set_xticklabels([b["label"] for b in bars],
                                             rotation=45, ha="right",
                                             fontsize=TICK_FONTSIZE)
    for t in tick_labels:
        if t.get_text().lower().startswith("rcs"):
            t.set_fontweight("bold")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"wrote {out_path}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results-root", type=Path, default=Path("results"),
                    help="directory containing run1/ run2/ ... (default: results)")
    ap.add_argument("--output-dir", type=Path, default=Path("plots"),
                    help="where PNGs are written (default: plots)")
    ap.add_argument("--groups", nargs="+", default=None,
                    help="restrict to listed groups "
                         "(cam-ros cam-stream ros-stream cam-both). default: all")
    args = ap.parse_args()

    if not args.results_root.exists():
        print(f"results root {args.results_root} does not exist", file=sys.stderr)
        return 2

    _apply_font_sizes()
    acc, cpu, runs = collect(args.results_root)
    if not acc:
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)

    groups = args.groups if args.groups else sorted(acc)
    written = 0
    for g in groups:
        if g not in acc:
            print(f"skip {g}: not present in results", file=sys.stderr)
            continue
        out = args.output_dir / f"{g}.png"
        if plot_group(g, acc[g], cpu.get(g, {}), len(runs), out):
            written += 1
    return 0 if written else 1


if __name__ == "__main__":
    sys.exit(main())
