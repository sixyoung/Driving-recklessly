#!/usr/bin/env python3
"""Create six standalone expected/actual channel plots from recorded runtime data."""

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
OUT = ROOT / "expected_actual_channels"
OUT.mkdir(parents=True, exist_ok=True)

TARGETS = {
    "normal": 32,
    "red_light": 47,
    "aggressive_decel": 106,
    "lane_deviation": 116,
}

FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
font_manager.fontManager.addfont(str(FONT_PATH))
CJK_FONT = font_manager.FontProperties(fname=str(FONT_PATH)).get_name()
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
    }
)


def load(scenario):
    frame = pd.read_csv(ROOT / f"{scenario}.csv")
    frame = frame.loc[frame["vehicle_id"] == TARGETS[scenario]].copy()
    frame = frame.sort_values("ros_time").reset_index(drop=True)
    frame["elapsed_s"] = frame["ros_time"] - frame["ros_time"].iloc[0]
    return frame


def smooth(values, window=7):
    """Robust display-only filter: centered median followed by EWMA."""
    series = pd.Series(np.asarray(values, dtype=float))
    median = series.rolling(window, center=True, min_periods=1).median()
    return median.ewm(alpha=0.34, adjust=False).mean().to_numpy()


def style(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save(fig, name):
    fig.savefig(OUT / f"{name}.png", dpi=240)
    fig.savefig(OUT / f"{name}.svg")
    plt.close(fig)


def estimate_stop_position(frame):
    candidates = frame.loc[
        frame["nearest_stop_distance_m"].notna()
        & frame["nearest_stop_state"].astype(str).str.lower().eq("red")
        & frame["nearest_stop_distance_m"].between(0.0, 35.0)
    ]
    if candidates.empty:
        candidates = frame.loc[frame["nearest_stop_distance_m"].notna()]
    projected = candidates["travel_distance_m"] + candidates["nearest_stop_distance_m"]
    return float(projected.median())


def st_figures():
    frame = load("red_light")
    time = frame["elapsed_s"].to_numpy()
    actual = smooth(frame["travel_distance_m"], 7)
    stop_s = estimate_stop_position(frame)
    # Expected center follows the measured approach but must settle 2 m before stop line.
    expected_center = np.minimum(actual, stop_s - 2.0)
    expected_center = np.maximum.accumulate(expected_center)
    lower = np.maximum(expected_center - 1.25, 0.0)
    upper = np.minimum(expected_center + 1.25, stop_s)

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.70, label="预期通道")
    ax.plot(time, expected_center, color="#2166AC", linewidth=2.3, label="预期轨迹")
    ax.axhline(stop_s, color="#D73027", linestyle="--", linewidth=1.5, label="停止线约束")
    ax.set(xlabel="时间 t（s）", ylabel="纵向行程 S（m）", xlim=(time.min(), time.max()))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "01_ST_expected_channel")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.60, label="预期通道")
    ax.plot(time, actual, color="#D73027", linewidth=2.5, label="实测轨迹（滤波显示）")
    ax.axhline(stop_s, color="#202020", linestyle="--", linewidth=1.5, label="停止线约束")
    crossing = np.flatnonzero(actual > stop_s)
    if len(crossing):
        idx = crossing[0]
        ax.scatter(time[idx], actual[idx], s=55, color="#D73027", edgecolor="white", zorder=5, label="首次越界")
    ax.set(xlabel="时间 t（s）", ylabel="纵向行程 S（m）", xlim=(time.min(), time.max()))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "02_ST_actual_trajectory")


def sl_figures():
    frame = load("lane_deviation")
    distance = smooth(frame["travel_distance_m"], 5)
    actual = smooth(frame["lateral_deviation_m"], 7)
    center = np.zeros_like(distance)
    lower = center - 1.5
    upper = center + 1.5

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(distance, lower, upper, color="#BFD7EE", alpha=0.70, label="预期通道")
    ax.plot(distance, center, color="#2166AC", linewidth=2.2, label="车道中心线")
    ax.plot(distance, lower, color="#2166AC", linestyle="--", linewidth=1.2)
    ax.plot(distance, upper, color="#2166AC", linestyle="--", linewidth=1.2, label="道路几何边界")
    ax.set(xlabel="纵向行程 S（m）", ylabel="横向偏移 L（m）", xlim=(distance.min(), distance.max()), ylim=(-2.2, 2.2))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "03_SL_expected_channel")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(distance, lower, upper, color="#BFD7EE", alpha=0.60, label="预期通道")
    ax.plot(distance, actual, color="#D73027", linewidth=2.5, label="实测横向偏移（滤波显示）")
    crossing = np.flatnonzero(actual > 1.5)
    if len(crossing):
        idx = crossing[0]
        ax.scatter(distance[idx], actual[idx], s=55, color="#D73027", edgecolor="white", zorder=5, label="首次越界")
    # Retain the actual scale while keeping the 1.5 m boundary visually readable.
    ymax = max(3.0, float(np.nanpercentile(actual, 98)) * 1.08)
    ax.set(xlabel="纵向行程 S（m）", ylabel="横向偏移 L（m）", xlim=(distance.min(), distance.max()), ylim=(-2.2, ymax))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "04_SL_actual_trajectory")


def at_figures():
    normal = load("normal")
    abnormal = load("aggressive_decel")
    time = abnormal["elapsed_s"].to_numpy()
    actual = smooth(abnormal["accel_mps2"], 5)
    # Interpolate the normal-run acceleration as the expected center on the same time base.
    normal_time = normal["elapsed_s"].to_numpy()
    normal_accel = smooth(normal["accel_mps2"], 7)
    expected_center = np.interp(time, normal_time, normal_accel)
    lower = expected_center - 3.0
    upper = expected_center + 3.0

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.70, label="预期通道")
    ax.plot(time, expected_center, color="#2166AC", linewidth=2.3, label="正常工况期望")
    ax.axhline(0, color="#707070", linewidth=0.9)
    ax.set(xlabel="时间 t（s）", ylabel="纵向加速度 A（m/s²）", xlim=(time.min(), time.max()))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "05_AT_expected_channel")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.60, label="预期通道")
    ax.plot(time, actual, color="#D73027", linewidth=2.5, label="实测加速度（滤波显示）")
    violation = (actual < lower) | (actual > upper)
    crossing = np.flatnonzero(violation)
    if len(crossing):
        idx = crossing[0]
        ax.scatter(time[idx], actual[idx], s=55, color="#D73027", edgecolor="white", zorder=5, label="首次越界")
    ax.axhline(0, color="#707070", linewidth=0.9)
    ax.set(xlabel="时间 t（s）", ylabel="纵向加速度 A（m/s²）", xlim=(time.min(), time.max()))
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    save(fig, "06_AT_actual_trajectory")


def main():
    st_figures()
    sl_figures()
    at_figures()
    print("generated", len(list(OUT.glob("*.png"))), "PNG and", len(list(OUT.glob("*.svg"))), "SVG")


if __name__ == "__main__":
    main()
