#!/usr/bin/env python3
"""Generate six PPT figures for multi-source constrained behavior channels.

The data are a deterministic representative mechanism case reused from the
behavior-identification plotting utilities.  Bands denote feasible channel
domains, not statistical confidence intervals.
"""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case  # noqa: E402
from generate_constraint_channel_figures import build_constraints  # noqa: E402


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
ZH = FontProperties(fname=FONT_PATH)
ZH_BOLD = FontProperties(fname=FONT_PATH, weight="bold")

BLUE = "#1769AA"
LIGHT_BLUE = "#DCEEFF"
ORANGE = "#E68613"
LIGHT_ORANGE = "#FCE1BE"
RED = "#D62728"
GREEN = "#2A9D8F"
PURPLE = "#6F42C1"
DARK = "#202A35"
GRAY = "#6B7280"
LIGHT_GRAY = "#E9EEF4"


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def configure_style():
    plt.rcParams.update({
        "axes.unicode_minus": False,
        "svg.fonttype": "none",
        "axes.titleweight": "bold",
        "axes.linewidth": 0.8,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
    })



def build_method_constraints(case):
    """Build method-page constraint strengths with physically interpretable dynamics."""
    t = case["t"]
    traffic_control = sigmoid((t - 3.45) / 0.08)
    road_geometry = np.clip(0.72 + 0.04 * np.sin(0.65 * t), 0.0, 1.0)
    traffic_interaction = np.clip(
        0.16 + 0.34 * np.exp(-((t - 4.15) / 0.95) ** 2), 0.0, 1.0
    )
    accel_util = np.clip(np.abs(case["a_actual"]) / 3.5, 0.0, 1.0)
    jerk = np.gradient(case["a_actual"], t)
    jerk_util = np.clip(np.abs(jerk) / 4.0, 0.0, 1.0)
    dynamics_raw = 0.12 + 0.18 * accel_util + 0.12 * jerk_util
    window = 21
    padded = np.pad(dynamics_raw, (window // 2, window // 2), mode="edge")
    vehicle_dynamics = np.convolve(
        padded, np.ones(window) / window, mode="valid"
    )
    vehicle_dynamics = np.clip(vehicle_dynamics, 0.0, 1.0)
    weights = np.array([0.40, 0.25, 0.15, 0.20])
    fused = (
        weights[0] * traffic_control
        + weights[1] * road_geometry
        + weights[2] * traffic_interaction
        + weights[3] * vehicle_dynamics
    )
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


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.65, alpha=0.78)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def labels(ax, title, xlabel, ylabel):
    ax.set_xlabel(xlabel, fontproperties=ZH, fontsize=10)
    ax.set_ylabel(ylabel, fontproperties=ZH, fontsize=10)
    style_axis(ax)


def save(fig, outdir, name):
    fig.savefig(outdir / f"{name}.png", dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(outdir / f"{name}.svg", facecolor="white", bbox_inches="tight")
    plt.close(fig)


def plot_st(case, outdir):
    t = case["t"]
    free_center = case["s_actual"]
    free_half = 1.10 + 0.10 * t
    free_low, free_high = free_center - free_half, free_center + free_half
    constrained_low, constrained_high = case["st_lower"], case["st_upper"]
    violation = case["s_actual"] > constrained_high

    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.fill_between(t, free_low, free_high, color=LIGHT_GRAY, alpha=0.86,
                    label="动力学自由通道")
    ax.plot(t, free_high, color=GRAY, linestyle=":", linewidth=1.4)
    ax.plot(t, free_low, color=GRAY, linestyle=":", linewidth=1.4)
    ax.fill_between(t, constrained_low, constrained_high, color=LIGHT_BLUE, alpha=0.96,
                    label="交通控制约束后的S–T通道")
    ax.plot(t, constrained_low, color=BLUE, linestyle="--", linewidth=1.6)
    ax.plot(t, constrained_high, color=BLUE, linestyle="--", linewidth=1.6,
            label="融合通道边界")
    ax.plot(t, case["s_actual"], color=DARK, linewidth=2.2, label="实际纵向轨迹")
    ax.plot(t[violation], case["s_actual"][violation], color=RED, linewidth=3.0,
            label="约束越界段")
    ax.axhline(case["stop_line"], color=RED, linestyle="-.", linewidth=1.7,
               label="停止线约束")
    ax.axvspan(3.45, t[-1], color=ORANGE, alpha=0.08, label="红灯有效区间")
    ax.annotate("上边界受停止线截断", xy=(4.75, case["stop_line"]), xytext=(3.45, 22.8),
                arrowprops=dict(arrowstyle="->", color=RED, lw=1.3),
                fontproperties=ZH, fontsize=9, color=RED)
    labels(ax, "交通控制约束下的S–T纵向时空通道", "时间 t (s)", "纵向位置 S (m)")
    ax.set_ylim(-3.0, 31.0)
    ax.legend(loc="upper left", ncol=2, prop=ZH, fontsize=7.7, frameon=True)
    save(fig, outdir, "01_ST_traffic_control_constrained_channel")


def plot_sl(case, outdir):
    s = case["s_axis"]
    geometry_center = 0.03 * np.sin(s / 7.0)
    geometry_half = 1.75 + 0.05 * np.sin(s / 7.0)
    geometry_low = geometry_center - geometry_half
    geometry_high = geometry_center + geometry_half
    interaction = np.exp(-((s - 15.5) / 4.1) ** 2)
    shift = -0.25 * interaction
    fused_half = geometry_half - 0.90 - 0.18 - 0.20 * interaction
    fused_low = geometry_center + shift - fused_half
    fused_high = geometry_center + shift + fused_half

    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.fill_between(s, geometry_low, geometry_high, color=LIGHT_GRAY, alpha=0.88,
                    label="道路几何可行域")
    ax.plot(s, geometry_low, color=GRAY, linestyle=":", linewidth=1.5,
            label="车道几何边界")
    ax.plot(s, geometry_high, color=GRAY, linestyle=":", linewidth=1.5)
    ax.fill_between(s, fused_low, fused_high, color=LIGHT_BLUE, alpha=0.96,
                    label="几何–交互耦合后的S–L通道")
    ax.plot(s, fused_low, color=BLUE, linestyle="--", linewidth=1.6)
    ax.plot(s, fused_high, color=BLUE, linestyle="--", linewidth=1.6,
            label="融合通道边界")
    ax.plot(s, case["l_actual"] - 0.10 * interaction, color=DARK, linewidth=2.2,
            label="实际横向轨迹")
    ax.axvspan(11.2, 19.8, color=ORANGE, alpha=0.10, label="交通交互增强区间")
    ax.annotate("交互约束驱动通道偏移并收缩", xy=(15.5, fused_high[197]),
                xytext=(17.4, 1.15), arrowprops=dict(arrowstyle="->", color=ORANGE, lw=1.3),
                fontproperties=ZH, fontsize=9, color=ORANGE)
    labels(ax, "道路几何–交通交互约束下的S–L横向空间通道",
           "纵向位置 S (m)", "横向偏移 L (m)")
    ax.set_ylim(-2.10, 2.10)
    ax.legend(loc="lower left", ncol=2, prop=ZH, fontsize=7.7, frameon=True)
    save(fig, outdir, "02_SL_geometry_interaction_constrained_channel")


def plot_at(case, constraints, outdir):
    t = case["t"]
    physical_low = np.full_like(t, -3.5)
    physical_high = np.full_like(t, 3.2)
    risk = np.clip(0.70 * constraints["traffic_control"] +
                   0.30 * constraints["traffic_interaction"], 0.0, 1.0)
    fused_low = -2.15 + 0.75 * risk
    fused_high = 2.35 - 1.15 * risk

    fig, ax = plt.subplots(figsize=(7.2, 4.4), constrained_layout=True)
    ax.fill_between(t, physical_low, physical_high, color=LIGHT_GRAY, alpha=0.88,
                    label="车辆动力学极限域")
    ax.plot(t, physical_low, color=GRAY, linestyle=":", linewidth=1.5)
    ax.plot(t, physical_high, color=GRAY, linestyle=":", linewidth=1.5,
            label="动力学边界")
    ax.fill_between(t, fused_low, fused_high, color=LIGHT_BLUE, alpha=0.96,
                    label="动力学–交互耦合后的A–T通道")
    ax.plot(t, fused_low, color=BLUE, linestyle="--", linewidth=1.6)
    ax.plot(t, fused_high, color=BLUE, linestyle="--", linewidth=1.6,
            label="融合通道边界")
    ax.plot(t, case["a_actual"], color=DARK, linewidth=2.2, label="实际加速度")
    ax.axvspan(3.45, t[-1], color=ORANGE, alpha=0.09, label="控制约束激活区间")
    ax.axhline(0, color="#AAB2BD", linewidth=0.9)
    ax.annotate("风险升高触发加速度域收缩", xy=(4.25, fused_high[246]), xytext=(1.9, 2.75),
                arrowprops=dict(arrowstyle="->", color=ORANGE, lw=1.3),
                fontproperties=ZH, fontsize=9, color=ORANGE)
    labels(ax, "车辆动力学–交通交互约束下的A–T加速度通道",
           "时间 t (s)", "加速度 A (m/s²)")
    ax.set_ylim(-4.0, 3.7)
    ax.legend(loc="lower left", ncol=2, prop=ZH, fontsize=7.7, frameon=True)
    save(fig, outdir, "03_AT_dynamics_interaction_constrained_channel")


def draw_node(ax, x, y, text, color, width=0.15, height=0.075, bold=False):
    box = FancyBboxPatch((x - width / 2, y - height / 2), width, height,
                         boxstyle="round,pad=0.012,rounding_size=0.018",
                         facecolor=color, edgecolor=BLUE, linewidth=1.1)
    ax.add_patch(box)
    ax.text(x, y, text, ha="center", va="center",
            fontproperties=ZH_BOLD if bold else ZH, fontsize=8.2,
            color="#123A67")


def arrow(ax, start, end, color=BLUE, style="-", alpha=0.75, rad=0.0, lw=1.05):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=9,
                                linewidth=lw, linestyle=style, color=color,
                                alpha=alpha, connectionstyle=f"arc3,rad={rad}"))


def plot_dynamic_causal_graph(case, constraints, outdir):
    """Plot time-varying causal contributions as data, not a structure diagram."""
    t = case["t"]
    weights = constraints["weights"]
    contributions = [
        weights[0] * constraints["traffic_control"],
        weights[1] * constraints["road_geometry"],
        weights[2] * constraints["traffic_interaction"],
        weights[3] * constraints["vehicle_dynamics"],
    ]
    colors = [RED, BLUE, ORANGE, GREEN]
    labels_ = ["交通控制贡献", "道路几何贡献", "交通交互贡献", "车辆动力学贡献"]
    styles = ["-", "--", "-.", ":"]
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(7.8, 5.8), sharex=True,
        gridspec_kw={"height_ratios": [1.05, 0.95]}, constrained_layout=True
    )
    for y, color, label, ls in zip(contributions, colors, labels_, styles):
        ax1.plot(t, y, color=color, linestyle=ls, linewidth=2.2, label=label)
    ax1.axvline(3.45, color=GRAY, linestyle=":", linewidth=1.3)
    ax1.axvspan(3.45, t[-1], color=ORANGE, alpha=0.07)
    ax1.set_ylim(-0.01, 0.44)
    ax1.set_ylabel("加权因果贡献", fontproperties=ZH, fontsize=10)
    ax1.set_title("四类约束对隐状态的动态因果贡献",
                  fontproperties=ZH_BOLD, fontsize=13, color="#123A67")
    ax1.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax1)
    influence = np.vstack([
        0.95 * constraints["traffic_control"],
        0.85 * constraints["road_geometry"],
        0.45 * constraints["traffic_interaction"],
        0.75 * constraints["traffic_interaction"],
        0.90 * constraints["vehicle_dynamics"],
        0.30 * constraints["vehicle_dynamics"],
    ])
    path_labels = [
        "交通控制→S–T", "道路几何→S–L",
        "交通交互→S–T", "交通交互→S–L",
        "车辆动力学→A–T", "车辆动力学→S–T",
    ]
    mesh = ax2.pcolormesh(
        t, np.arange(len(path_labels)), influence,
        shading="nearest", cmap="viridis", vmin=0.0, vmax=1.0
    )
    ax2.set_ylim(len(path_labels) - 0.5, -0.5)
    ax2.set_yticks(np.arange(len(path_labels)))
    ax2.set_yticklabels(path_labels, fontproperties=ZH, fontsize=8.5)
    ax2.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax2.set_ylabel("约束—通道路径", fontproperties=ZH, fontsize=10)
    ax2.set_title("约束对三通道的时变影响强度",
                  fontproperties=ZH_BOLD, fontsize=12, color="#123A67")
    cax = ax2.inset_axes([1.03, 0.0, 0.028, 1.0])
    for i in range(6):
        cax.add_patch(Rectangle(
            (0, i / 6.0), 1, 1 / 6.0,
            facecolor=plt.cm.viridis((i + 0.5) / 6.0), edgecolor="none"
        ))
    cax.set_xlim(0, 1)
    cax.set_ylim(0, 1)
    cax.set_xticks([])
    cax.set_yticks([0, 0.5, 1.0])
    cax.set_yticklabels(["0.0", "0.5", "1.0"], fontsize=8)
    cax.yaxis.tick_right()
    cax.yaxis.set_label_position("right")
    cax.set_ylabel("归一化影响强度", fontproperties=ZH, fontsize=9, labelpad=8)
    save(fig, outdir, "04_dynamic_causal_effect_data")


def plot_constraint_fusion(case, constraints, outdir):
    t = case["t"]
    hard = np.maximum(constraints["traffic_control"], constraints["road_geometry"])
    soft = 0.55 * constraints["traffic_interaction"] + 0.45 * constraints["vehicle_dynamics"]
    uniform = 1.0 - (1.0 - hard) * (1.0 - 0.62 * soft)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.6, 5.6), sharex=True,
                                   gridspec_kw={"height_ratios": [1.18, 0.82]},
                                   constrained_layout=True)
    ax1.plot(t, constraints["traffic_control"], color=RED, linewidth=2.0,
             label="交通控制约束")
    ax1.plot(t, constraints["road_geometry"], color=BLUE, linestyle="--", linewidth=1.8,
             label="道路几何约束")
    ax1.plot(t, constraints["traffic_interaction"], color=ORANGE, linestyle="-.", linewidth=1.8,
             label="交通交互约束")
    ax1.plot(t, constraints["vehicle_dynamics"], color=GREEN, linestyle=":", linewidth=2.0,
             label="车辆动力学约束")
    ax1.set_ylim(-0.03, 1.05)
    ax1.set_ylabel("约束激活强度", fontproperties=ZH, fontsize=10)
    ax1.set_title("多源异构约束的时序激活", fontproperties=ZH_BOLD,
                  fontsize=13, color="#123A67")
    ax1.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax1)

    ax2.plot(t, hard, color=RED, linewidth=2.0, label="硬约束层")
    ax2.plot(t, soft, color=ORANGE, linestyle="--", linewidth=2.0, label="软约束层")
    ax2.fill_between(t, 0, uniform, color="#EAE2F8", alpha=0.78, label="统一约束域")
    ax2.plot(t, uniform, color=PURPLE, linewidth=3.0, label="约束融合强度")
    ax2.axvline(3.45, color=GRAY, linestyle=":", linewidth=1.4)
    ax2.text(3.51, 0.08, "硬约束激活", fontproperties=ZH, fontsize=8.5, color=GRAY)
    ax2.set_ylim(-0.03, 1.05)
    ax2.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax2.set_ylabel("统一约束强度", fontproperties=ZH, fontsize=10)
    ax2.set_title("硬–软约束融合与统一约束域形成", fontproperties=ZH_BOLD,
                  fontsize=13, color="#123A67")
    ax2.legend(loc="upper left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax2)
    save(fig, outdir, "05_hard_soft_constraint_fusion")


def plot_fused_channel_update(case, constraints, outdir):
    t = case["t"]
    raw = np.vstack([np.ones_like(t), np.ones_like(t), np.ones_like(t)])
    fused = np.vstack([constraints["width_st"], constraints["width_sl"], constraints["width_at"]])
    colors = [RED, BLUE, GREEN]
    labels_ = ["S–T通道", "S–L通道", "A–T通道"]
    styles = ["-", "--", "-."]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.6, 5.6), sharex=True,
                                   gridspec_kw={"height_ratios": [1.15, 0.85]},
                                   constrained_layout=True)
    for y, c, lab, ls in zip(fused, colors, labels_, styles):
        ax1.plot(t, y, color=c, linestyle=ls, linewidth=2.25, label=lab)
    ax1.axhline(1.0, color=GRAY, linestyle=":", linewidth=1.35, label="未约束通道宽度")
    ax1.axvspan(3.45, t[-1], color=ORANGE, alpha=0.08, label="控制约束激活区间")
    ax1.set_ylim(0.0, 1.08)
    ax1.set_ylabel("归一化通道宽度", fontproperties=ZH, fontsize=10)
    ax1.set_title("统一约束驱动的三通道边界协同更新", fontproperties=ZH_BOLD,
                  fontsize=13, color="#123A67")
    ax1.legend(loc="lower left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax1)

    contraction = np.clip(raw - fused, 0.0, 1.0)
    im = ax2.pcolormesh(t, np.arange(3), contraction, shading="nearest",
                        cmap="Blues", vmin=0.0, vmax=0.9)
    ax2.set_ylim(2.5, -0.5)
    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels(labels_, fontproperties=ZH)
    ax2.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax2.set_ylabel("行为通道", fontproperties=ZH, fontsize=10)
    ax2.set_title("通道边界收缩量",
                  fontproperties=ZH_BOLD, fontsize=12, color="#123A67")
    cax = ax2.inset_axes([1.03, 0.0, 0.028, 1.0])
    for i in range(6):
        cax.add_patch(Rectangle((0, i / 6.0), 1, 1 / 6.0,
                                facecolor=plt.cm.Blues((i + 0.5) / 6.0), edgecolor="none"))
    cax.set_xlim(0, 1)
    cax.set_ylim(0, 1)
    cax.set_xticks([])
    cax.set_yticks([0, 0.5, 1.0])
    cax.set_yticklabels(["0.00", "0.45", "0.90"], fontsize=8)
    cax.yaxis.tick_right()
    cax.yaxis.set_label_position("right")
    cax.set_ylabel("归一化收缩量", fontproperties=ZH, fontsize=9, labelpad=8)
    save(fig, outdir, "06_fused_three_channel_dynamic_update")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=SCRIPT_DIR.parent / "plots" / "method_page_six_figures")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    case = build_case()
    constraints = build_method_constraints(case)
    plot_st(case, args.output_dir)
    plot_sl(case, args.output_dir)
    plot_at(case, constraints, args.output_dir)
    plot_dynamic_causal_graph(case, constraints, args.output_dir)
    plot_constraint_fusion(case, constraints, args.output_dir)
    plot_fused_channel_update(case, constraints, args.output_dir)
    print(f"Generated six figures in: {args.output_dir}")
    for path in sorted(args.output_dir.iterdir()):
        print(f"{path.name}\t{path.stat().st_size}")


if __name__ == "__main__":
    main()
