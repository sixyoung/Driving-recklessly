#!/usr/bin/env python3
"""Generate six separate expected/actual behavior channel figures."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case  # noqa: E402


BLUE = "#1769AA"
LIGHT_BLUE = "#DCEEFF"
ORANGE = "#E68613"
LIGHT_ORANGE = "#FCE1BE"
RED = "#D62728"
DARK = "#202A35"
GRAY = "#6B7280"

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
ZH = FontProperties(fname=FONT_PATH)
ZH_BOLD = FontProperties(fname=FONT_PATH, weight="bold")


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.82)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)


def save_figure(fig, outdir, basename):
    fig.savefig(outdir / f"{basename}.png", dpi=300, facecolor="white")
    fig.savefig(outdir / f"{basename}.svg", facecolor="white")
    plt.close(fig)


def set_labels(ax, title, xlabel, ylabel):
    ax.set_title(title, fontproperties=ZH_BOLD, fontsize=14, color="#123A67", pad=10)
    ax.set_xlabel(xlabel, fontproperties=ZH, fontsize=10)
    ax.set_ylabel(ylabel, fontproperties=ZH, fontsize=10)
    style_axis(ax)


def plot_expected(ax, x, lower, upper, title, xlabel, ylabel, stop_line=None):
    center = 0.5 * (lower + upper)
    ax.fill_between(x, lower, upper, color=LIGHT_BLUE, alpha=0.95,
                    label="预期行为通道")
    ax.plot(x, lower, color=BLUE, linestyle="--", linewidth=1.6,
            label="预期通道下边界")
    ax.plot(x, upper, color=BLUE, linestyle="--", linewidth=1.6,
            label="预期通道上边界")
    ax.plot(x, center, color=BLUE, linewidth=2.2, label="预期通道中心")
    if stop_line is not None:
        ax.axhline(stop_line, color=GRAY, linestyle="-.", linewidth=1.35)
        ax.text(x[0] + 0.12, stop_line + 0.45, "停止线",
                fontproperties=ZH, color=GRAY, fontsize=8)
    set_labels(ax, title, xlabel, ylabel)
    ax.legend(loc="best", prop=ZH, fontsize=8, frameon=True)


def plot_actual(ax, x, center, half_width, title, xlabel, ylabel,
                stop_line=None, crossing_mask=None):
    lower = center - half_width
    upper = center + half_width
    ax.fill_between(x, lower, upper, color=LIGHT_ORANGE, alpha=0.78,
                    label="实际行为通道")
    ax.plot(x, lower, color=ORANGE, linestyle="--", linewidth=1.55,
            label="实际通道下边界")
    ax.plot(x, upper, color=ORANGE, linestyle="--", linewidth=1.55,
            label="实际通道上边界")
    ax.plot(x, center, color=DARK, linewidth=2.25, label="实际轨迹中心")
    if stop_line is not None:
        ax.axhline(stop_line, color=GRAY, linestyle="-.", linewidth=1.35)
        ax.text(x[0] + 0.12, stop_line + 0.45, "停止线",
                fontproperties=ZH, color=GRAY, fontsize=8)
    if crossing_mask is not None and np.any(crossing_mask):
        ax.plot(x[crossing_mask], center[crossing_mask], color=RED,
                linewidth=2.9, label="停止线越界段")
        first = int(np.argmax(crossing_mask))
        ax.scatter([x[first]], [center[first]], s=58, color=RED,
                   edgecolor="white", linewidth=0.9, zorder=6)
    set_labels(ax, title, xlabel, ylabel)
    ax.legend(loc="best", prop=ZH, fontsize=8, frameon=True)


def generate(case, outdir):
    figures = []

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    plot_expected(
        ax, case["t"], case["st_lower"], case["st_upper"],
        "S–T纵向时空预期通道", "时间 t (s)", "纵向位置 S (m)",
        stop_line=case["stop_line"]
    )
    save_figure(fig, outdir, "01_ST_expected_channel")

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    st_half_width = 0.28 + 0.025 * case["t"]
    plot_actual(
        ax, case["t"], case["s_actual"], st_half_width,
        "S–T纵向时空实际通道", "时间 t (s)", "纵向位置 S (m)",
        stop_line=case["stop_line"], crossing_mask=case["s_actual"] > case["stop_line"]
    )
    save_figure(fig, outdir, "02_ST_actual_channel")

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    plot_expected(
        ax, case["s_axis"], case["sl_lower"], case["sl_upper"],
        "S–L横向空间预期通道", "纵向位置 S (m)", "横向偏移 L (m)"
    )
    save_figure(fig, outdir, "03_SL_expected_channel")

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    plot_actual(
        ax, case["s_axis"], case["l_actual"], np.full_like(case["l_actual"], 0.18),
        "S–L横向空间实际通道", "纵向位置 S (m)", "横向偏移 L (m)"
    )
    save_figure(fig, outdir, "04_SL_actual_channel")

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    plot_expected(
        ax, case["t"], case["at_lower"], case["at_upper"],
        "A–T加速度预期通道", "时间 t (s)", "加速度 A (m/s²)"
    )
    save_figure(fig, outdir, "05_AT_expected_channel")

    fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
    plot_actual(
        ax, case["t"], case["a_actual"], np.full_like(case["a_actual"], 0.14),
        "A–T加速度实际通道", "时间 t (s)", "加速度 A (m/s²)"
    )
    save_figure(fig, outdir, "06_AT_actual_channel")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "six_channel_figures"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["svg.fonttype"] = "none"
    case = build_case()
    generate(case, args.output_dir)
    print(f"Generated six separate figures in: {args.output_dir}")
    for path in sorted(args.output_dir.iterdir()):
        print(f"  {path.name} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
