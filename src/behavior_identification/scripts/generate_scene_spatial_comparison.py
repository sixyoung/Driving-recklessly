#!/usr/bin/env python3
"""Generate a top-down spatial comparison of expected and actual trajectories."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case  # noqa: E402


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
ZH = FontProperties(fname=FONT_PATH)
ZH_BOLD = FontProperties(fname=FONT_PATH, weight="bold")

BLUE = "#1769AA"
LIGHT_BLUE = "#DCEEFF"
ORANGE = "#E68613"
LIGHT_ORANGE = "#FCE1BE"
RED = "#D62728"
DARK = "#202A35"
ROAD = "#D7DCE2"
ROAD_EDGE = "#7B8794"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "scene_spatial_result"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    case = build_case()
    stop_line = case["stop_line"]

    s_actual = case["s_actual"]
    l_actual = np.interp(s_actual, case["s_axis"], case["l_actual"])
    expected_s = np.linspace(0.0, stop_line - 0.55, 240)
    expected_l = 0.04 * np.sin(expected_s / 5.5)

    actual_half_width = 0.26
    expected_half_width = 0.72
    crossing = s_actual > stop_line
    cross_idx = int(np.argmax(crossing))

    fig, ax = plt.subplots(figsize=(9.0, 4.8), constrained_layout=True)
    ax.set_aspect("equal", adjustable="box")

    # Main road and intersecting road.
    ax.add_patch(Rectangle((-1.0, -3.6), 32.0, 7.2,
                           facecolor=ROAD, edgecolor=ROAD_EDGE, linewidth=1.2, zorder=0))
    ax.add_patch(Rectangle((15.8, -8.0), 5.0, 16.0,
                           facecolor="#E2E6EA", edgecolor=ROAD_EDGE, linewidth=1.0, zorder=0))

    # Lane markings.
    ax.plot([-1, 31], [0, 0], color="white", linewidth=1.6,
            linestyle=(0, (7, 7)), alpha=0.95, zorder=1)
    ax.plot([-1, 31], [3.3, 3.3], color="white", linewidth=1.5, zorder=1)
    ax.plot([-1, 31], [-3.3, -3.3], color="white", linewidth=1.5, zorder=1)

    # Stop line and signal.
    ax.plot([stop_line, stop_line], [-3.2, 3.2], color="white", linewidth=5.5, zorder=2)
    ax.plot([stop_line, stop_line], [-3.2, 3.2], color="#505A66", linewidth=1.0, zorder=3)
    signal_x, signal_y = stop_line - 0.9, 2.45
    ax.add_patch(Rectangle((signal_x - 0.30, signal_y - 0.52), 0.60, 1.04,
                           facecolor="#263238", edgecolor="white", linewidth=0.8, zorder=6))
    ax.add_patch(Circle((signal_x, signal_y + 0.28), 0.14,
                        facecolor=RED, edgecolor="white", linewidth=0.7, zorder=7))
    ax.add_patch(Circle((signal_x, signal_y), 0.14,
                        facecolor="#8E8E8E", edgecolor="white", linewidth=0.7, zorder=7))
    ax.add_patch(Circle((signal_x, signal_y - 0.28), 0.14,
                        facecolor="#8E8E8E", edgecolor="white", linewidth=0.7, zorder=7))

    # Expected behavior corridor ending before the stop line.
    ax.fill_between(expected_s, expected_l - expected_half_width,
                    expected_l + expected_half_width, color=LIGHT_BLUE, alpha=0.85,
                    label="预期行为通道", zorder=2)
    ax.plot(expected_s, expected_l, color=BLUE, linewidth=2.5,
            label="预期轨迹", zorder=4)
    ax.plot(expected_s, expected_l - expected_half_width, color=BLUE,
            linestyle="--", linewidth=1.2, zorder=3)
    ax.plot(expected_s, expected_l + expected_half_width, color=BLUE,
            linestyle="--", linewidth=1.2, zorder=3)

    # Actual behavior corridor and actual trajectory.
    ax.fill_between(s_actual, l_actual - actual_half_width,
                    l_actual + actual_half_width, color=LIGHT_ORANGE, alpha=0.70,
                    label="实际行为通道", zorder=2)
    ax.plot(s_actual, l_actual, color=DARK, linewidth=2.4,
            label="实际轨迹", zorder=5)
    ax.plot(s_actual[crossing], l_actual[crossing], color=RED, linewidth=3.2,
            label="停止线越界轨迹", zorder=6)
    ax.scatter([s_actual[cross_idx]], [l_actual[cross_idx]], s=85,
               color=RED, edgecolor="white", linewidth=1.0, zorder=8)

    # Vehicle direction and annotations.
    ax.add_patch(FancyArrowPatch((4.0, -1.35), (7.0, -1.35),
                                 arrowstyle="-|>", mutation_scale=15,
                                 linewidth=1.8, color=DARK, zorder=6))
    ax.text(4.1, -1.95, "车辆行驶方向", fontproperties=ZH,
            fontsize=9, color=DARK)
    ax.annotate("红灯约束下的停止位置",
                xy=(expected_s[-1], expected_l[-1]), xytext=(10.1, 2.15),
                arrowprops=dict(arrowstyle="->", color=BLUE, linewidth=1.3),
                fontproperties=ZH, fontsize=9, color=BLUE)
    ax.annotate(r"首次越界点 $t_0$",
                xy=(s_actual[cross_idx], l_actual[cross_idx]), xytext=(20.2, -2.05),
                arrowprops=dict(arrowstyle="->", color=RED, linewidth=1.3),
                fontproperties=ZH, fontsize=9, color=RED)

    ax.text(stop_line + 0.25, 3.95, "信控路口", fontproperties=ZH_BOLD,
            fontsize=10, color="#123A67")
    ax.text(stop_line - 0.42, -4.25, "停止线", fontproperties=ZH,
            fontsize=9, color="#505A66")

    ax.set_xlim(-0.5, 30.0)
    ax.set_ylim(-5.0, 5.0)
    ax.set_xlabel("纵向位置 S (m)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("横向位置 L (m)", fontproperties=ZH, fontsize=10)
    ax.set_title("典型信控路口场景下预期轨迹与实际轨迹空间对比",
                 fontproperties=ZH_BOLD, fontsize=15, color="#0D3B78", pad=12)
    ax.legend(loc="lower right", prop=ZH, fontsize=8.5, ncol=2, frameon=True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(False)

    out = args.output_dir / "scene_expected_actual_spatial_comparison"
    fig.savefig(out.with_suffix(".png"), dpi=300, facecolor="white")
    fig.savefig(out.with_suffix(".svg"), facecolor="white")
    plt.close(fig)
    print(out.with_suffix(".png"))


if __name__ == "__main__":
    main()
