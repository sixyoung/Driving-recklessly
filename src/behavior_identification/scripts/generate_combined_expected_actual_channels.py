#!/usr/bin/env python3
"""Generate three combined expected-channel and measured-trajectory figures."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
OUT = ROOT / "combined_expected_actual_channels"
OUT.mkdir(parents=True, exist_ok=True)

TARGETS = {
    "normal": 32,
    "red_light_long": 144,
    "aggressive_decel": 106,
    "lane_deviation": 116,
}

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT_PATH)
CJK_FONT = font_manager.FontProperties(fname=FONT_PATH).get_name()
mpl.rcParams.update(
    {
        "font.family": CJK_FONT,
        "axes.unicode_minus": False,
        "font.size": 12,
        "axes.labelsize": 13,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "legend.fontsize": 10,
        "axes.linewidth": 1.1,
        "savefig.bbox": "tight",
        "savefig.facecolor": "white",
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
    }
)

BLUE = "#2166AC"
BLUE_FILL = "#BFD7EE"
RED = "#D73027"
ORANGE = "#E69F00"
GRAY = "#4B5563"


def load(scenario: str) -> pd.DataFrame:
    frame = pd.read_csv(ROOT / f"{scenario}.csv")
    frame = frame.loc[frame["vehicle_id"] == TARGETS[scenario]].copy()
    if frame.empty:
        raise RuntimeError(f"No target data for {scenario}")
    frame = frame.sort_values("ros_time").reset_index(drop=True)
    frame["elapsed_s"] = frame["ros_time"] - frame["ros_time"].iloc[0]
    return frame


def smooth(values, window=7, alpha=0.34):
    """Display filter only: centered median followed by EWMA."""
    series = pd.Series(np.asarray(values, dtype=float))
    median = series.rolling(window, center=True, min_periods=1).median()
    return median.ewm(alpha=alpha, adjust=False).mean().to_numpy()


def style(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.75)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.margins(x=0.01)


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=300)
    fig.savefig(OUT / f"{stem}.svg")
    plt.close(fig)


def estimate_stop_position(frame):
    red = frame.loc[
        frame["nearest_stop_distance_m"].notna()
        & frame["nearest_stop_state"].astype(str).str.lower().eq("red")
        & frame["nearest_stop_distance_m"].between(0.0, 35.0)
    ]
    if red.empty:
        red = frame.loc[frame["nearest_stop_distance_m"].notna()]
    projected = red["travel_distance_m"] + red["nearest_stop_distance_m"]
    return float(projected.median())


def plot_st():
    frame = load("red_light_long")
    time = frame["elapsed_s"].to_numpy()
    actual_raw = frame["travel_distance_m"].to_numpy()
    actual = smooth(actual_raw, 7)
    speed = smooth(frame["speed_mps"], 7)

    stop_s = estimate_stop_position(frame)
    stop_margin = 2.0
    cap = stop_s - stop_margin
    brake_start = cap - 33.0
    expected = actual.copy()
    approach = actual > brake_start
    expected[approach] = brake_start + (cap - brake_start) * np.tanh(
        (actual[approach] - brake_start) / (cap - brake_start)
    )
    expected = np.maximum.accumulate(expected)
    # S–T is a time-position feasible corridor. Speed uncertainty accumulates
    # into several metres of longitudinal-position tolerance.
    half_width = np.clip(2.50 + 0.35 * speed, 2.50, 7.50)
    lower = np.maximum(expected - half_width, 0.0)
    upper = np.minimum(expected + half_width, stop_s)

    fig, ax = plt.subplots(figsize=(8.2, 5.0), constrained_layout=True)
    ax.fill_between(time, lower, upper, color=BLUE_FILL, alpha=0.78, label="预期行为通道")
    ax.plot(time, lower, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(time, upper, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(time, expected, color=BLUE, linewidth=1.8, label="通道中心")
    ax.plot(time, actual, color=RED, linewidth=2.5, label="实测轨迹（滤波显示）")
    ax.axhline(stop_s, color=GRAY, linestyle=":", linewidth=1.5, label="停止线约束")
    exit_idx = np.flatnonzero(approach & (actual > upper))
    if len(exit_idx):
        i = int(exit_idx[0])
        ax.scatter(time[i], actual[i], s=58, color=ORANGE, edgecolor="white", linewidth=0.8, zorder=6, label="首次通道偏离")
    ax.set(xlabel="时间 t（s）", ylabel="纵向行程 S（m）", xlim=(time.min(), time.max()), ylim=(0, max(actual.max(), stop_s) * 1.04))
    ax.legend(frameon=False, loc="upper left", ncol=2)
    style(ax)
    save(fig, "01_ST_expected_channel_and_actual")
    return {"samples": len(frame), "stop_position_m": stop_s, "channel_half_width_m": [float(half_width.min()), float(half_width.max())]}


def plot_sl():
    frame = load("lane_deviation")
    distance_all = smooth(frame["travel_distance_m"], 5)
    lateral_all = smooth(frame["lateral_deviation_m"], 7)
    # Stop before the second non-physical rise caused by Map02 switching the
    # reference lane after the real lane-change event has already completed.
    lateral_step = np.gradient(lateral_all)
    jump = np.flatnonzero(
        (distance_all > 60.0) & (lateral_all > 1.9) & (lateral_step > 0.08)
    )
    end = int(jump[0]) if len(jump) else len(frame)
    end = max(end, 4)
    distance = distance_all[:end]
    actual_raw = frame["lateral_deviation_m"].to_numpy()[:end]
    actual = lateral_all[:end]
    speed = smooth(frame["speed_mps"], 7)[:end]

    normal = load("normal")
    normal_s = smooth(normal["travel_distance_m"], 5)
    normal_l = smooth(normal["lateral_deviation_m"], 7)
    expected = np.interp(distance, normal_s, normal_l, left=normal_l[0], right=normal_l[-1])
    # Soft lane-keeping corridor; the hard road-geometric limit remains ±1.5 m.
    half_width = np.clip(0.62 + 0.10 * speed / 15.0, 0.62, 0.72)
    lower = expected - half_width
    upper = expected + half_width

    fig, ax = plt.subplots(figsize=(8.2, 5.0), constrained_layout=True)
    ax.fill_between(distance, lower, upper, color=BLUE_FILL, alpha=0.78, label="预期行为通道")
    ax.plot(distance, lower, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(distance, upper, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(distance, expected, color=BLUE, linewidth=1.8, label="通道中心")
    ax.plot(distance, actual, color=RED, linewidth=2.5, label="实测轨迹（滤波显示）")
    ax.axhline(1.5, color=GRAY, linestyle=":", linewidth=1.4, label="道路几何约束")
    ax.axhline(-1.5, color=GRAY, linestyle=":", linewidth=1.4)
    exit_idx = np.flatnonzero((actual < lower) | (actual > upper))
    if len(exit_idx):
        i = int(exit_idx[0])
        ax.scatter(distance[i], actual[i], s=58, color=ORANGE, edgecolor="white", linewidth=0.8, zorder=6, label="首次通道偏离")
    ymin = min(-1.9, float(lower.min()) - 0.25)
    ymax = max(2.2, float(actual.max()) * 1.10)
    ax.set(xlabel="纵向行程 S（m）", ylabel="横向偏移 L（m）", xlim=(distance.min(), distance.max()), ylim=(ymin, ymax))
    ax.legend(frameon=False, loc="upper left", ncol=2)
    style(ax)
    save(fig, "02_SL_expected_channel_and_actual")
    return {"samples_total": len(frame), "samples_event_window": end, "channel_half_width_m": [float(half_width.min()), float(half_width.max())]}


def plot_at():
    abnormal = load("aggressive_decel")
    time = abnormal["elapsed_s"].to_numpy()
    actual_raw = abnormal["accel_mps2"].to_numpy()
    actual = smooth(actual_raw, 3, alpha=0.50)

    # The neutral centre of the admissible acceleration domain is A = 0.
    expected = np.zeros_like(time)
    # A–T channel is the admissible acceleration domain itself, rather than
    # a narrow uncertainty band around the expected acceleration centre.
    lower = np.full_like(time, -3.0)
    upper = np.full_like(time, 3.0)

    fig, ax = plt.subplots(figsize=(8.2, 5.0), constrained_layout=True)
    ax.fill_between(time, lower, upper, color=BLUE_FILL, alpha=0.78, label="预期行为通道（±3 m/s²）")
    ax.plot(time, lower, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(time, upper, color=BLUE, linestyle="--", linewidth=1.15)
    ax.plot(time, expected, color=BLUE, linewidth=1.8, label="通道中心 A=0")
    ax.plot(time, actual, color=RED, linewidth=2.5, label="实测轨迹（滤波显示）")
    exit_idx = np.flatnonzero(
        (time >= 4.5) & ((actual < lower) | (actual > upper))
    )
    if len(exit_idx):
        i = int(exit_idx[0])
        ax.scatter(time[i], actual[i], s=58, color=ORANGE, edgecolor="white", linewidth=0.8, zorder=6, label="首次通道偏离")
    ax.set(xlabel="时间 t（s）", ylabel="纵向加速度 A（m/s²）", xlim=(time.min(), time.max()), ylim=(-4.2, 4.2))
    ax.legend(frameon=False, loc="upper right", ncol=2)
    style(ax)
    save(fig, "03_AT_expected_channel_and_actual")
    return {"samples": len(abnormal), "channel_bounds_mps2": [-3.0, 3.0]}


def main():
    audit = {"ST": plot_st(), "SL": plot_sl(), "AT": plot_at()}
    audit["filter"] = "centered median + EWMA, plotted exits follow the displayed filtered trajectory"
    audit["source"] = "recorded CARLA/ROS target-vehicle runtime CSV"
    (OUT / "channel_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    print("generated", len(list(OUT.glob("*.png"))), "PNG and", len(list(OUT.glob("*.svg"))), "SVG")


if __name__ == "__main__":
    main()
