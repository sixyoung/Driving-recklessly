#!/usr/bin/env python3
"""Generate nine separated, title-free data figures for the method result page."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case
from generate_method_page_six_figures import (
    ZH, BLUE, ORANGE, RED, GREEN, PURPLE, GRAY,
    configure_style, style_axis, save,
    build_method_constraints, plot_st, plot_sl, plot_at,
)


def add_vector_colorbar(ax, cmap, ticks, labels, text):
    cax = ax.inset_axes([1.03, 0.0, 0.028, 1.0])
    for i in range(6):
        cax.add_patch(Rectangle(
            (0, i / 6.0), 1, 1 / 6.0,
            facecolor=cmap((i + 0.5) / 6.0), edgecolor="none"
        ))
    cax.set_xlim(0, 1)
    cax.set_ylim(0, 1)
    cax.set_xticks([])
    cax.set_yticks(ticks)
    cax.set_yticklabels(labels, fontsize=8)
    cax.yaxis.tick_right()
    cax.yaxis.set_label_position("right")
    cax.set_ylabel(text, fontproperties=ZH, fontsize=9, labelpad=8)


def plot_dynamic_contributions(case, constraints, outdir):
    t = case["t"]
    w = constraints["weights"]
    series = [
        w[0] * constraints["traffic_control"],
        w[1] * constraints["road_geometry"],
        w[2] * constraints["traffic_interaction"],
        w[3] * constraints["vehicle_dynamics"],
    ]
    labels = ["交通控制贡献", "道路几何贡献", "交通交互贡献", "车辆动力学贡献"]
    colors = [RED, BLUE, ORANGE, GREEN]
    styles = ["-", "--", "-.", ":"]

    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    for y, label, color, ls in zip(series, labels, colors, styles):
        ax.plot(t, y, color=color, linestyle=ls, linewidth=2.3, label=label)
    ax.axvline(3.45, color=GRAY, linestyle=":", linewidth=1.4)
    ax.axvspan(3.45, t[-1], color=ORANGE, alpha=0.07, label="控制约束激活区间")
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("加权因果贡献", fontproperties=ZH, fontsize=10)
    ax.set_ylim(-0.01, 0.44)
    ax.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax)
    save(fig, outdir, "04_dynamic_constraint_contributions")


def plot_sensitivity_matrix(outdir):
    # Coefficients come directly from the channel-width update equations.
    matrix = np.array([
        [0.78, 0.00, 0.00],
        [0.00, 0.28, 0.00],
        [0.08, 0.18, 0.08],
        [0.00, 0.00, 0.42],
    ])
    row_labels = ["交通控制约束", "道路几何约束", "交通交互约束", "车辆动力学约束"]
    col_labels = ["S–T通道", "S–L通道", "A–T通道"]

    fig, ax = plt.subplots(figsize=(6.3, 4.2), constrained_layout=True)
    mesh = ax.pcolormesh(
        np.arange(4), np.arange(5), matrix,
        shading="flat", cmap="Blues", vmin=0.0, vmax=0.8,
        edgecolors="white", linewidth=1.5
    )
    ax.set_xlim(0, 3)
    ax.set_ylim(4, 0)
    ax.set_xticks(np.arange(3) + 0.5)
    ax.set_yticks(np.arange(4) + 0.5)
    ax.set_xticklabels(col_labels, fontproperties=ZH, fontsize=10)
    ax.set_yticklabels(row_labels, fontproperties=ZH, fontsize=10)
    ax.set_xlabel("预期行为通道", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("多源约束", fontproperties=ZH, fontsize=10)
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            color = "white" if matrix[i, j] >= 0.42 else "#202A35"
            ax.text(j + 0.5, i + 0.5, f"{matrix[i, j]:.2f}",
                    ha="center", va="center", fontsize=11, color=color)
    add_vector_colorbar(
        ax, plt.cm.Blues, [0, 0.5, 1.0], ["0.00", "0.40", "0.80"],
        "边界收缩系数"
    )
    save(fig, outdir, "05_constraint_channel_sensitivity_matrix")


def plot_constraint_activation(case, constraints, outdir):
    t = case["t"]
    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.plot(t, constraints["traffic_control"], color=RED, linewidth=2.2,
            label="交通控制约束")
    ax.plot(t, constraints["road_geometry"], color=BLUE, linestyle="--",
            linewidth=2.0, label="道路几何约束")
    ax.plot(t, constraints["traffic_interaction"], color=ORANGE, linestyle="-.",
            linewidth=2.0, label="交通交互约束")
    ax.plot(t, constraints["vehicle_dynamics"], color=GREEN, linestyle=":",
            linewidth=2.2, label="车辆动力学约束")
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("约束激活强度", fontproperties=ZH, fontsize=10)
    ax.set_ylim(-0.03, 1.05)
    ax.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax)
    save(fig, outdir, "06_multisource_constraint_activation")


def plot_hard_soft_fusion(case, constraints, outdir):
    t = case["t"]
    hard = np.maximum(constraints["traffic_control"], constraints["road_geometry"])
    soft = 0.55 * constraints["traffic_interaction"] + 0.45 * constraints["vehicle_dynamics"]
    uniform = 1.0 - (1.0 - hard) * (1.0 - 0.62 * soft)

    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.plot(t, hard, color=RED, linewidth=2.2, label="硬约束层")
    ax.plot(t, soft, color=ORANGE, linestyle="--", linewidth=2.2, label="软约束层")
    ax.fill_between(t, 0, uniform, color="#EAE2F8", alpha=0.78, label="统一约束域")
    ax.plot(t, uniform, color=PURPLE, linewidth=3.0, label="约束融合强度")
    ax.axvline(3.45, color=GRAY, linestyle=":", linewidth=1.4)
    ax.text(3.51, 0.08, "硬约束激活", fontproperties=ZH, fontsize=8.5, color=GRAY)
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("统一约束强度", fontproperties=ZH, fontsize=10)
    ax.set_ylim(-0.03, 1.05)
    ax.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax)
    save(fig, outdir, "07_hard_soft_constraint_fusion")


def plot_channel_widths(case, constraints, outdir):
    t = case["t"]
    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.plot(t, constraints["width_st"], color=RED, linewidth=2.35, label="S–T通道")
    ax.plot(t, constraints["width_sl"], color=BLUE, linestyle="--",
            linewidth=2.2, label="S–L通道")
    ax.plot(t, constraints["width_at"], color=GREEN, linestyle="-.",
            linewidth=2.2, label="A–T通道")
    ax.axhline(1.0, color=GRAY, linestyle=":", linewidth=1.4, label="未约束通道宽度")
    ax.axvspan(3.45, t[-1], color=ORANGE, alpha=0.08, label="控制约束激活区间")
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("归一化通道宽度", fontproperties=ZH, fontsize=10)
    ax.set_ylim(0.0, 1.08)
    ax.legend(loc="lower left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax)
    save(fig, outdir, "08_fused_three_channel_widths")


def plot_channel_contraction(case, constraints, outdir):
    t = case["t"]
    contraction = np.vstack([
        1.0 - constraints["width_st"],
        1.0 - constraints["width_sl"],
        1.0 - constraints["width_at"],
    ])
    fig, ax = plt.subplots(figsize=(7.2, 3.8), constrained_layout=True)
    mesh = ax.pcolormesh(
        t, np.arange(3), contraction, shading="nearest",
        cmap="Blues", vmin=0.0, vmax=0.9
    )
    ax.set_ylim(2.5, -0.5)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["S–T通道", "S–L通道", "A–T通道"],
                       fontproperties=ZH, fontsize=9)
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("行为通道", fontproperties=ZH, fontsize=10)
    add_vector_colorbar(
        ax, plt.cm.Blues, [0, 0.5, 1.0], ["0.00", "0.45", "0.90"],
        "归一化收缩量"
    )
    save(fig, outdir, "09_channel_boundary_contraction")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "method_page_separated_figures"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    case = build_case()
    constraints = build_method_constraints(case)
    plot_st(case, args.output_dir)
    plot_sl(case, args.output_dir)
    plot_at(case, constraints, args.output_dir)
    plot_dynamic_contributions(case, constraints, args.output_dir)
    plot_sensitivity_matrix(args.output_dir)
    plot_constraint_activation(case, constraints, args.output_dir)
    plot_hard_soft_fusion(case, constraints, args.output_dir)
    plot_channel_widths(case, constraints, args.output_dir)
    plot_channel_contraction(case, constraints, args.output_dir)
    print(f"Generated nine separated figures in: {args.output_dir}")
    for path in sorted(args.output_dir.iterdir()):
        print(f"{path.name}\t{path.stat().st_size}")


if __name__ == "__main__":
    main()
