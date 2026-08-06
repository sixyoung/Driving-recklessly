#!/usr/bin/env python3
"""Corrected channel plots using a long S–T run and event-window S–L data."""

import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/home/bob/文档/备份/demo06/src/behavior_identification/scripts")
import generate_expected_actual_channels as g


g.TARGETS["red_light_long"] = 144


def st_figures():
    frame = g.load("red_light_long")
    time = frame["elapsed_s"].to_numpy()
    actual = g.smooth(frame["travel_distance_m"], 7)
    stop_s = g.estimate_stop_position(frame)
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
    g.style(ax)
    g.save(fig, "01_ST_expected_channel")

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
    g.style(ax)
    g.save(fig, "02_ST_actual_trajectory")


def sl_figures():
    frame = g.load("lane_deviation")
    distance = g.smooth(frame["travel_distance_m"], 5)
    actual = g.smooth(frame["lateral_deviation_m"], 7)
    discontinuity = np.flatnonzero(actual > 4.5)
    end = int(discontinuity[0]) if len(discontinuity) else len(frame)
    distance = distance[:end]
    actual = actual[:end]
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
    g.style(ax)
    g.save(fig, "03_SL_expected_channel")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(distance, lower, upper, color="#BFD7EE", alpha=0.60, label="预期通道")
    ax.plot(distance, actual, color="#D73027", linewidth=2.5, label="实测横向偏移（滤波显示）")
    crossing = np.flatnonzero(actual > 1.5)
    if len(crossing):
        idx = crossing[0]
        ax.scatter(distance[idx], actual[idx], s=55, color="#D73027", edgecolor="white", zorder=5, label="首次越界")
    ymax = max(3.0, float(np.nanmax(actual)) * 1.08)
    ax.set(xlabel="纵向行程 S（m）", ylabel="横向偏移 L（m）", xlim=(distance.min(), distance.max()), ylim=(-2.2, ymax))
    ax.legend(frameon=False, loc="upper left")
    g.style(ax)
    g.save(fig, "04_SL_actual_trajectory")


def at_figures():
    normal = g.load("normal")
    abnormal = g.load("aggressive_decel")
    time = abnormal["elapsed_s"].to_numpy()
    actual = g.smooth(abnormal["accel_mps2"], 5)
    normal_time = normal["elapsed_s"].to_numpy()
    expected_center = np.interp(time, normal_time, g.smooth(normal["accel_mps2"], 7))
    lower = np.full_like(time, -3.0)
    upper = np.full_like(time, 3.0)

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.70, label="预期通道")
    ax.plot(time, expected_center, color="#2166AC", linewidth=2.3, label="正常工况期望")
    ax.axhline(0, color="#707070", linewidth=0.9)
    ax.set(xlabel="时间 t（s）", ylabel="纵向加速度 A（m/s²）", xlim=(time.min(), time.max()), ylim=(-4.2, 4.2))
    ax.legend(frameon=False, loc="upper left")
    g.style(ax)
    g.save(fig, "05_AT_expected_channel")

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.fill_between(time, lower, upper, color="#BFD7EE", alpha=0.60, label="预期通道")
    ax.plot(time, actual, color="#D73027", linewidth=2.5, label="实测加速度（滤波显示）")
    raw_actual = abnormal["accel_mps2"].to_numpy()
    crossing = np.flatnonzero((raw_actual < -3.0) | (raw_actual > 3.0))
    if len(crossing):
        idx = crossing[0]
        ax.scatter(time[idx], actual[idx], s=55, color="#D73027", edgecolor="white", zorder=5, label="原始数据首次越界")
    ax.axhline(0, color="#707070", linewidth=0.9)
    ax.set(xlabel="时间 t（s）", ylabel="纵向加速度 A（m/s²）", xlim=(time.min(), time.max()), ylim=(-4.2, 4.2))
    ax.legend(frameon=False, loc="upper left")
    g.style(ax)
    g.save(fig, "06_AT_actual_trajectory")


if __name__ == "__main__":
    st_figures()
    sl_figures()
    at_figures()
    print("generated 6 corrected PNG and 6 SVG")
