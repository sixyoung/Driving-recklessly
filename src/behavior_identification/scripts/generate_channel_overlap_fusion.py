#!/usr/bin/env python3
"""Generate expected-vs-actual channel overlap and fused consistency figure."""

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


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
ZH = FontProperties(fname=FONT_PATH)
ZH_BOLD = FontProperties(fname=FONT_PATH, weight="bold")

RED = "#D62728"
BLUE = "#1769AA"
GREEN = "#2A9D8F"
PURPLE = "#6F42C1"
ORANGE = "#E68613"
GRAY = "#6B7280"


def channel_coverage(expected_low, expected_high, actual_low, actual_high):
    intersection = np.maximum(
        0.0, np.minimum(expected_high, actual_high) - np.maximum(expected_low, actual_low)
    )
    actual_width = actual_high - actual_low
    return intersection / np.maximum(actual_width, 1e-9)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "channel_overlap_result"
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    case = build_case()
    t = case["t"]

    # S-T actual channel.
    st_half = 0.28 + 0.025 * t
    st_actual_low = case["s_actual"] - st_half
    st_actual_high = case["s_actual"] + st_half
    overlap_st = channel_coverage(
        case["st_lower"], case["st_upper"], st_actual_low, st_actual_high
    )

    # S-L data are mapped from the S axis back to the common time axis.
    sl_expected_low = np.interp(case["s_actual"], case["s_axis"], case["sl_lower"])
    sl_expected_high = np.interp(case["s_actual"], case["s_axis"], case["sl_upper"])
    sl_center = np.interp(case["s_actual"], case["s_axis"], case["l_actual"])
    overlap_sl = channel_coverage(
        sl_expected_low, sl_expected_high, sl_center - 0.18, sl_center + 0.18
    )

    # A-T actual channel.
    overlap_at = channel_coverage(
        case["at_lower"], case["at_upper"],
        case["a_actual"] - 0.14, case["a_actual"] + 0.14
    )

    weights = np.array([0.50, 0.20, 0.30])
    fused = weights[0] * overlap_st + weights[1] * overlap_sl + weights[2] * overlap_at

    red_phase_start = 3.45
    fig, ax = plt.subplots(figsize=(8.8, 4.8), constrained_layout=True)

    ax.plot(t, overlap_st, color=RED, linewidth=2.3, label="S–T通道覆盖率")
    ax.plot(t, overlap_sl, color=BLUE, linewidth=2.0, linestyle="--",
            label="S–L通道覆盖率")
    ax.plot(t, overlap_at, color=GREEN, linewidth=2.0, linestyle="-.",
            label="A–T通道覆盖率")
    ax.plot(t, fused, color=PURPLE, linewidth=3.2, label="三通道融合一致性")

    ax.axvspan(red_phase_start, t[-1], color=ORANGE, alpha=0.055,
               label="红灯约束作用区间")
    ax.axvspan(case["t0"], t[-1], color=RED, alpha=0.09,
               label="有效异常区间")
    ax.axvline(red_phase_start, color=ORANGE, linewidth=1.6, linestyle=":")
    ax.axvline(case["t0"], color=RED, linewidth=1.8, linestyle=":")
    ax.axhline(0.65, color=GRAY, linewidth=1.4, linestyle=(0, (2, 3)),
               label="一致性判定阈值")

    ax.text(red_phase_start + 0.05, 0.13, "红灯约束激活",
            fontproperties=ZH, fontsize=9, color=ORANGE, rotation=90, va="bottom")
    ax.text(case["t0"] + 0.05, 0.13, r"首次越界 $t_0$",
            fontproperties=ZH, fontsize=9, color=RED, rotation=90, va="bottom")

    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.02, 1.04)
    ax.set_xlabel("时间 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("预期通道对实际通道的覆盖率", fontproperties=ZH, fontsize=10)
    ax.set_title("三通道预期通道对实际通道的覆盖率及融合一致性",
                 fontproperties=ZH_BOLD, fontsize=15, color="#0D3B78", pad=12)
    ax.legend(loc="lower left", prop=ZH, fontsize=8, ncol=2, frameon=True)
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.82)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)

    out = args.output_dir / "channel_overlap_and_fused_consistency"
    fig.savefig(out.with_suffix(".png"), dpi=300, facecolor="white")
    fig.savefig(out.with_suffix(".svg"), facecolor="white")
    plt.close(fig)

    print(out.with_suffix(".png"))
    print(f"mean_overlap_ST={overlap_st.mean():.4f}")
    print(f"mean_overlap_SL={overlap_sl.mean():.4f}")
    print(f"mean_overlap_AT={overlap_at.mean():.4f}")
    print(f"min_fused_consistency={fused.min():.4f}")


if __name__ == "__main__":
    main()
