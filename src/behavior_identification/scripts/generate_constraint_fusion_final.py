#!/usr/bin/env python3
"""Generate the final Chinese constraint-fusion figure with direct font loading."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case  # noqa: E402
from generate_constraint_channel_figures import build_constraints  # noqa: E402


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
ZH = FontProperties(fname=FONT_PATH)
ZH_BOLD = FontProperties(fname=FONT_PATH, weight="bold")

RED = "#D62728"
BLUE = "#1769AA"
ORANGE = "#E68613"
GREEN = "#2A9D8F"
PURPLE = "#6F42C1"


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.82)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "constraint_channel_results"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    case = build_case()
    c = build_constraints(case)
    t = case["t"]

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9.2, 6.0), sharex=True,
        gridspec_kw={"height_ratios": [1.15, 0.85]}, constrained_layout=True
    )

    ax1.plot(t, c["traffic_control"], color=RED, linewidth=2.1,
             label="交通控制约束")
    ax1.plot(t, c["road_geometry"], color=BLUE, linewidth=1.9,
             linestyle="--", label="道路几何约束")
    ax1.plot(t, c["traffic_interaction"], color=ORANGE, linewidth=1.9,
             linestyle="-.", label="交通交互约束")
    ax1.plot(t, c["vehicle_dynamics"], color=GREEN, linewidth=1.9,
             linestyle=":", label="车辆动力学约束")
    ax1.plot(t, c["fused"], color=PURPLE, linewidth=3.2,
             label="多源约束融合强度")
    ax1.axvspan(case["t0"], t[-1], color=RED, alpha=0.08)
    ax1.axvline(case["t0"], color=ORANGE, linestyle=":", linewidth=2.0)
    ax1.text(case["t0"] + 0.06, 0.91, r"首次越界 $t_0$",
             fontproperties=ZH, color=ORANGE, fontsize=9)
    ax1.set_ylabel("归一化约束强度", fontproperties=ZH, fontsize=10)
    ax1.set_ylim(-0.03, 1.05)
    ax1.set_title("(a) 多源异构约束的时序耦合结果",
                  fontproperties=ZH_BOLD, fontsize=13, color="#123A67")
    ax1.legend(loc="upper left", ncol=3, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax1)

    ax2.plot(t, c["width_st"], color=RED, linewidth=2.2, label="S–T通道宽度")
    ax2.plot(t, c["width_sl"], color=BLUE, linewidth=2.0,
             linestyle="--", label="S–L通道宽度")
    ax2.plot(t, c["width_at"], color=GREEN, linewidth=2.0,
             linestyle="-.", label="A–T通道宽度")
    ax2.axvspan(case["t0"], t[-1], color=RED, alpha=0.08, label="异常行为区间")
    ax2.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax2.set_ylabel("归一化通道宽度", fontproperties=ZH, fontsize=10)
    ax2.set_ylim(0.0, 1.05)
    ax2.set_title("(b) 约束融合驱动的预期通道动态收缩",
                  fontproperties=ZH_BOLD, fontsize=13, color="#123A67")
    ax2.legend(loc="lower left", ncol=2, prop=ZH, fontsize=8, frameon=True)
    style_axis(ax2)

    fig.suptitle("多源约束耦合及预期行为通道动态更新结果",
                 fontproperties=ZH_BOLD, fontsize=16, color="#0D3B78")

    fig.savefig(args.output_dir / "constraint_fusion_and_channel_update.png",
                dpi=300, facecolor="white")
    fig.savefig(args.output_dir / "constraint_fusion_and_channel_update.svg",
                facecolor="white")
    plt.close(fig)
    print(args.output_dir / "constraint_fusion_and_channel_update.png")


if __name__ == "__main__":
    main()
