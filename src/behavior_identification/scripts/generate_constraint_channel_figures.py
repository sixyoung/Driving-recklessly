#!/usr/bin/env python3
"""Generate constraint-fusion and expected-vs-actual channel figures."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
GREEN = "#2A9D8F"
PURPLE = "#6F42C1"
DARK = "#202A35"
GRAY = "#6B7280"


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def configure_style():
    plt.rcParams.update({
        "font.family": "Noto Sans CJK SC",
        "font.sans-serif": ["Noto Sans CJK SC", "Droid Sans Fallback", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "svg.fonttype": "none",
        "axes.titleweight": "bold",
    })


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.65, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)


def build_constraints(case):
    t = case["t"]

    # Normalized activation/intensity of the four constraints.
    traffic_control = sigmoid((t - 3.45) / 0.08)
    road_geometry = np.clip(0.72 + 0.04 * np.sin(0.65 * t), 0.0, 1.0)
    traffic_interaction = np.clip(
        0.16 + 0.34 * np.exp(-((t - 4.15) / 0.95) ** 2), 0.0, 1.0
    )
    vehicle_dynamics = np.clip(case["d_at"], 0.0, 1.0)

    weights = np.array([0.40, 0.25, 0.15, 0.20])
    fused = (
        weights[0] * traffic_control
        + weights[1] * road_geometry
        + weights[2] * traffic_interaction
        + weights[3] * vehicle_dynamics
    )

    # Normalized channel widths after multi-source constraint coupling.
    width_st = np.clip(1.0 - 0.78 * traffic_control - 0.08 * traffic_interaction, 0.12, 1.0)
    width_sl = np.clip(1.0 - 0.28 * road_geometry - 0.18 * traffic_interaction, 0.45, 1.0)
    width_at = np.clip(1.0 - 0.42 * vehicle_dynamics - 0.08 * traffic_interaction, 0.48, 1.0)

    return {
        "traffic_control": traffic_control,
        "road_geometry": road_geometry,
        "traffic_interaction": traffic_interaction,
        "vehicle_dynamics": vehicle_dynamics,
        "fused": fused,
        "width_st": width_st,
        "width_sl": width_sl,
        "width_at": width_at,
        "weights": weights,
    }


def plot_constraint_fusion(case, constraints, outdir):
    t = case["t"]
    fig, axes = plt.subplots(
        2, 1, figsize=(9.2, 6.0), sharex=True,
        gridspec_kw={"height_ratios": [1.15, 0.85]}, constrained_layout=True
    )
    ax1, ax2 = axes

    ax1.plot(t, constraints["traffic_control"], color=RED, linewidth=2.1,
             label="交通控制约束")
    ax1.plot(t, constraints["road_geometry"], color=BLUE, linewidth=1.9,
             linestyle="--", label="道路几何约束")
    ax1.plot(t, constraints["traffic_interaction"], color=ORANGE, linewidth=1.9,
             linestyle="-.", label="交通交互约束")
    ax1.plot(t, constraints["vehicle_dynamics"], color=GREEN, linewidth=1.9,
             linestyle=":", label="车辆动力学约束")
    ax1.plot(t, constraints["fused"], color=PURPLE, linewidth=3.2,
             label="多源约束融合强度")
    ax1.axvspan(case["t0"], t[-1], color=RED, alpha=0.08)
    ax1.axvline(case["t0"], color=ORANGE, linestyle=":", linewidth=2.0)
    ax1.text(case["t0"] + 0.06, 0.91, r"首次越界 $t_0$", color=ORANGE,
             fontsize=9, fontweight="bold")
    ax1.set_ylabel("归一化约束强度", fontsize=10)
    ax1.set_ylim(-0.03, 1.05)
    ax1.set_title("(a) 多源异构约束的时序耦合结果", fontsize=13, color="#123A67")
    ax1.legend(loc="upper left", ncol=3, fontsize=8, frameon=True)
    style_axis(ax1)

    ax2.plot(t, constraints["width_st"], color=RED, linewidth=2.2, label="S–T通道宽度")
    ax2.plot(t, constraints["width_sl"], color=BLUE, linewidth=2.0,
             linestyle="--", label="S–L通道宽度")
    ax2.plot(t, constraints["width_at"], color=GREEN, linewidth=2.0,
             linestyle="-.", label="A–T通道宽度")
    ax2.axvspan(case["t0"], t[-1], color=RED, alpha=0.08, label="异常行为区间")
    ax2.set_xlabel("时间 t (s)", fontsize=10)
    ax2.set_ylabel("归一化通道宽度", fontsize=10)
    ax2.set_ylim(0.0, 1.05)
    ax2.set_title("(b) 约束融合驱动的预期通道动态收缩", fontsize=13, color="#123A67")
    ax2.legend(loc="lower left", ncol=2, fontsize=8, frameon=True)
    style_axis(ax2)

    fig.suptitle("多源约束耦合及预期行为通道动态更新结果",
                 fontsize=16, fontweight="bold", color="#0D3B78")
    fig.savefig(outdir / "constraint_fusion_and_channel_update.png", dpi=300, facecolor="white")
    fig.savefig(outdir / "constraint_fusion_and_channel_update.svg", facecolor="white")
    fig.savefig(outdir / "constraint_fusion_and_channel_update.pdf", facecolor="white")
    plt.close(fig)


def channel_actual_band(center, scale):
    return center - scale, center + scale


def plot_channel_comparison(ax, x, expected_low, expected_high,
                            actual_center, actual_low, actual_high,
                            title, xlabel, ylabel, violation=None, stop_line=None):
    ax.fill_between(x, expected_low, expected_high, color=LIGHT_BLUE, alpha=0.90,
                    label="预期行为通道")
    ax.plot(x, expected_low, color=BLUE, linestyle="--", linewidth=1.35)
    ax.plot(x, expected_high, color=BLUE, linestyle="--", linewidth=1.35,
            label="预期通道边界")

    ax.fill_between(x, actual_low, actual_high, color=LIGHT_ORANGE, alpha=0.60,
                    label="实际行为通道")
    ax.plot(x, actual_low, color=ORANGE, linestyle=":", linewidth=1.25)
    ax.plot(x, actual_high, color=ORANGE, linestyle=":", linewidth=1.25,
            label="实际通道边界")
    ax.plot(x, actual_center, color=DARK, linewidth=2.15, label="实际轨迹中心")

    if violation is not None and np.any(violation):
        ax.plot(x[violation], actual_center[violation], color=RED, linewidth=2.8,
                label="通道偏离区间")
        first = int(np.argmax(violation))
        ax.scatter([x[first]], [actual_center[first]], s=52, color=RED,
                   edgecolor="white", linewidth=0.8, zorder=6)

    if stop_line is not None:
        ax.axhline(stop_line, color=GRAY, linestyle="-.", linewidth=1.3)
        ax.text(x[0] + 0.12, stop_line + 0.45, "停止线", color=GRAY, fontsize=8)

    ax.set_title(title, fontsize=13, color="#123A67")
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    style_axis(ax)


def save_channel_comparisons(case, outdir):
    st_scale = 0.28 + 0.025 * case["t"]
    st_actual_low, st_actual_high = channel_actual_band(case["s_actual"], st_scale)
    sl_actual_low, sl_actual_high = channel_actual_band(case["l_actual"], 0.18)
    at_actual_low, at_actual_high = channel_actual_band(case["a_actual"], 0.14)

    configs = [
        {
            "name": "ST_expected_vs_actual_channel",
            "x": case["t"], "expected_low": case["st_lower"], "expected_high": case["st_upper"],
            "actual": case["s_actual"], "actual_low": st_actual_low, "actual_high": st_actual_high,
            "title": "S–T纵向时空通道：预期通道与实际通道对比",
            "xlabel": "时间 t (s)", "ylabel": "纵向位置 S (m)",
            "violation": case["d_st"] > 0.5, "stop_line": case["stop_line"],
        },
        {
            "name": "SL_expected_vs_actual_channel",
            "x": case["s_axis"], "expected_low": case["sl_lower"], "expected_high": case["sl_upper"],
            "actual": case["l_actual"], "actual_low": sl_actual_low, "actual_high": sl_actual_high,
            "title": "S–L横向空间通道：预期通道与实际通道对比",
            "xlabel": "纵向位置 S (m)", "ylabel": "横向偏移 L (m)",
            "violation": None, "stop_line": None,
        },
        {
            "name": "AT_expected_vs_actual_channel",
            "x": case["t"], "expected_low": case["at_lower"], "expected_high": case["at_upper"],
            "actual": case["a_actual"], "actual_low": at_actual_low, "actual_high": at_actual_high,
            "title": "A–T加速度通道：预期通道与实际通道对比",
            "xlabel": "时间 t (s)", "ylabel": "加速度 A (m/s²)",
            "violation": None, "stop_line": None,
        },
    ]

    for cfg in configs:
        fig, ax = plt.subplots(figsize=(6.4, 4.0), constrained_layout=True)
        plot_channel_comparison(
            ax, cfg["x"], cfg["expected_low"], cfg["expected_high"],
            cfg["actual"], cfg["actual_low"], cfg["actual_high"],
            cfg["title"], cfg["xlabel"], cfg["ylabel"], cfg["violation"], cfg["stop_line"]
        )
        ax.legend(loc="best", fontsize=7.6, ncol=2, frameon=True)
        fig.savefig(outdir / f"{cfg['name']}.png", dpi=300, facecolor="white")
        fig.savefig(outdir / f"{cfg['name']}.svg", facecolor="white")
        plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.3), constrained_layout=True)
    for ax, cfg in zip(axes, configs):
        short_title = cfg["title"].split("：")[0]
        plot_channel_comparison(
            ax, cfg["x"], cfg["expected_low"], cfg["expected_high"],
            cfg["actual"], cfg["actual_low"], cfg["actual_high"],
            short_title, cfg["xlabel"], cfg["ylabel"], cfg["violation"], cfg["stop_line"]
        )
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.055),
               ncol=5, fontsize=8.2, frameon=False)
    fig.suptitle("三通道预期行为边界与实际行为通道对比",
                 fontsize=16, fontweight="bold", color="#0D3B78", y=1.14)
    fig.savefig(outdir / "three_channel_expected_actual_composite.png", dpi=300,
                facecolor="white", bbox_inches="tight")
    fig.savefig(outdir / "three_channel_expected_actual_composite.svg",
                facecolor="white", bbox_inches="tight")
    fig.savefig(outdir / "three_channel_expected_actual_composite.pdf",
                facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "constraint_channel_results"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    case = build_case()
    constraints = build_constraints(case)
    plot_constraint_fusion(case, constraints, args.output_dir)
    save_channel_comparisons(case, args.output_dir)
    print(f"Generated figures in: {args.output_dir}")
    for path in sorted(args.output_dir.iterdir()):
        print(f"  {path.name} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
