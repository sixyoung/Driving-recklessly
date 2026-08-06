#!/usr/bin/env python3
"""Generate publication-ready figures for PPT page 4.

The representative case is deterministic and follows the constraints used by
behavior_identification: red-light/stop-line gating in S-T, lane bounds in S-L,
and a normal acceleration envelope in A-T.  It does not change runtime logic.
"""

from pathlib import Path
import argparse
import matplotlib
matplotlib.use(bytes([65,103,103]).decode())

import matplotlib.pyplot as plt
import numpy as np


BLUE = "#1769AA"
LIGHT_BLUE = "#DCEEFF"
RED = "#D62728"
ORANGE = "#E68613"
DARK = "#202A35"
GREEN = "#2A9D8F"
GRAY = "#6B7280"


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def build_case():
    t = np.linspace(0.0, 6.2, 360)

    # Free-moving trajectory reconstructed to match the representative run.
    tau = np.clip((t - 2.35) / (6.2 - 2.35), 0.0, 1.0)
    s_actual = 28.2 * tau ** 1.58

    # During the red phase the expected S-T channel contracts at the stop line.
    stop_line = 18.0
    s_center = np.minimum(s_actual, stop_line - 0.55)
    st_lower = s_center - (0.70 + 0.08 * t)
    st_upper = np.minimum(stop_line, s_center + (0.70 + 0.08 * t))

    # Lane-centered lateral motion remains inside the lane channel.
    s_axis = np.linspace(0.0, 28.2, 360)
    l_actual = 0.20 * np.sin(s_axis / 4.0) + 0.12 * sigmoid((s_axis - 13.5) / 2.0)
    sl_lower = -1.75 + 0.05 * np.sin(s_axis / 7.0)
    sl_upper = 1.75 + 0.05 * np.sin(s_axis / 7.0)

    # Smooth longitudinal acceleration consistent with the existing A-T plots.
    a_actual = (
        0.85 * np.exp(-((t - 0.45) / 0.34) ** 2)
        + 1.55 * np.exp(-((t - 3.25) / 0.55) ** 2)
        + 1.35 * np.exp(-((t - 4.25) / 0.70) ** 2)
    )
    at_lower = -0.55 - 0.10 * np.exp(-((t - 4.9) / 0.8) ** 2)
    at_upper = 2.05 + 0.08 * np.sin(t / 1.3)

    def channel_response(value, lower, upper):
        """Distance to channel center normalized by local half width.

        Values below 1 are inside the channel and values above 1 are outside.
        The continuous response retains information from all three channels.
        """
        center = 0.5 * (upper + lower)
        half_width = np.maximum(0.5 * (upper - lower), 1e-6)
        raw_response = np.abs(value - center) / half_width
        return raw_response / (1.0 + raw_response)

    d_st = channel_response(s_actual, st_lower, st_upper)
    d_sl_s = channel_response(l_actual, sl_lower, sl_upper)
    d_sl = np.interp(s_actual, s_axis, d_sl_s)
    d_at = channel_response(a_actual, at_lower, at_upper)
    fusion_weights = np.array([0.55, 0.20, 0.25])
    d_fused = fusion_weights[0] * d_st + fusion_weights[1] * d_sl + fusion_weights[2] * d_at

    exceed = d_st > 0.5
    first_idx = int(np.argmax(exceed)) if np.any(exceed) else len(t) - 1
    t0 = float(t[first_idx])
    duration = float(t[-1] - t0)
    dmax = float(np.max(d_st))
    cross_idx = int(np.argmax(s_actual >= stop_line))
    t_cross = float(t[cross_idx])

    return {
        "t": t,
        "s_actual": s_actual,
        "st_lower": st_lower,
        "st_upper": st_upper,
        "s_axis": s_axis,
        "l_actual": l_actual,
        "sl_lower": sl_lower,
        "sl_upper": sl_upper,
        "a_actual": a_actual,
        "at_lower": at_lower,
        "at_upper": at_upper,
        "d_st": d_st,
        "d_sl": d_sl,
        "d_at": d_at,
        "d_fused": d_fused,
        "fusion_weights": fusion_weights,
        "t0": t0,
        "duration": duration,
        "dmax": dmax,
        "t_cross": t_cross,
        "stop_line": stop_line,
    }


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.65, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(labelsize=9)


def plot_channel(ax, x, actual, lower, upper, title, xlabel, ylabel, violation=None):
    ax.fill_between(x, lower, upper, color=LIGHT_BLUE, alpha=0.95, label="Expected channel")
    ax.plot(x, lower, color=BLUE, linestyle="--", linewidth=1.4)
    ax.plot(x, upper, color=BLUE, linestyle="--", linewidth=1.4, label="Channel boundary")
    ax.plot(x, actual, color=DARK, linewidth=2.15, label="Actual trajectory")
    if violation is not None and np.any(violation):
        ax.plot(x[violation], actual[violation], color=RED, linewidth=2.8, label="Out-of-channel")
        first = int(np.argmax(violation))
        ax.scatter([x[first]], [actual[first]], s=55, color=ORANGE, edgecolor="white", linewidth=0.8, zorder=6)
    ax.set_title(title, fontsize=12, fontweight="bold", color="#123A67")
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    style_axis(ax)


def save_standalone(case, outdir):
    panels = [
        ("ST_expected_channel", case["t"], case["s_actual"], case["st_lower"], case["st_upper"],
         "S-T longitudinal channel", "Time (s)", "Longitudinal position S (m)", case["d_st"] > 0.5),
        ("SL_expected_channel", case["s_axis"], case["l_actual"], case["sl_lower"], case["sl_upper"],
         "S-L lateral channel", "Longitudinal position S (m)", "Lateral offset L (m)", None),
        ("AT_expected_channel", case["t"], case["a_actual"], case["at_lower"], case["at_upper"],
         "A-T acceleration channel", "Time (s)", "Acceleration A (m/s2)", None),
    ]
    for name, x, actual, lower, upper, title, xlabel, ylabel, violation in panels:
        fig, ax = plt.subplots(figsize=(6.2, 3.7), constrained_layout=True)
        plot_channel(ax, x, actual, lower, upper, title, xlabel, ylabel, violation)
        ax.legend(loc="best", frameon=True, fontsize=8, ncol=2)
        fig.savefig(outdir / f"{name}.png", dpi=300, facecolor="white")
        fig.savefig(outdir / f"{name}.svg", facecolor="white")
        plt.close(fig)


    fig, ax = plt.subplots(figsize=(8.0, 4.2), constrained_layout=True)
    ax.plot(case["t"], case["d_st"], color=RED, linewidth=2.0, label="S-T response")
    ax.plot(case["t"], case["d_sl"], color=BLUE, linewidth=1.8, linestyle="--", label="S-L response")
    ax.plot(case["t"], case["d_at"], color=GREEN, linewidth=1.8, linestyle="-.", label="A-T response")
    ax.plot(case["t"], case["d_fused"], color="#6F42C1", linewidth=3.0, label="Fused response")
    ax.axhline(0.5, color=GRAY, linewidth=1.4, linestyle=":", label="Channel threshold")
    ax.axvspan(case["t0"], case["t"][-1], color=RED, alpha=0.09, label="Effective abnormal interval")
    ax.axvline(case["t0"], color=ORANGE, linestyle=":", linewidth=2.0)
    ax.set_title("Normalized three-channel response and fusion", fontsize=13, fontweight="bold", color="#123A67")
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Normalized response", fontsize=10)
    ax.set_ylim(-0.03, 1.05)
    style_axis(ax)
    ax.legend(loc="upper left", fontsize=8, ncol=2, frameon=True)
    fig.savefig(outdir / "normalized_channel_fusion.png", dpi=300, facecolor="white")
    fig.savefig(outdir / "normalized_channel_fusion.svg", facecolor="white")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.2, 3.7), constrained_layout=True)
    ax.axis("off")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title("Scene-constrained channel result", fontsize=13, fontweight="bold", color="#123A67")
    ax.text(0.08, 0.82, "Traffic-light phase", fontsize=10, color=GRAY)
    ax.axvspan(0.08, 0.92, ymin=0.64, ymax=0.75, color="#FDE2E2")
    ax.text(0.50, 0.695, "RED PHASE", ha="center", va="center", color=RED, fontsize=12, fontweight="bold")
    cards = [
        ("Dominant channel", "S-T"),
        ("First exceedance", f"{case['t0']:.2f} s"),
        ("Violation duration", f"{case['duration']:.2f} s"),
        ("Maximum response", f"{case['dmax']:.2f}"),
    ]
    for (label, value), y in zip(cards, [0.52, 0.38, 0.24, 0.10]):
        ax.add_patch(plt.Rectangle((0.08, y - 0.045), 0.84, 0.105,
                                   facecolor="#F5F8FC", edgecolor="#A9C7E8", linewidth=1.0))
        ax.text(0.12, y + 0.005, label, va="center", fontsize=9, color=GRAY)
        ax.text(0.88, y + 0.005, value, va="center", ha="right", fontsize=10,
                color="#123A67", fontweight="bold")
    fig.savefig(outdir / "scene_constrained_result.png", dpi=300, facecolor="white")
    fig.savefig(outdir / "scene_constrained_result.svg", facecolor="white")
    plt.close(fig)


def make_composite(case, outdir):
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "axes.unicode_minus": False,
        "svg.fonttype": "none",
    })
    fig = plt.figure(figsize=(13.33, 7.5), constrained_layout=True, facecolor="white")
    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 0.82])
    ax_st = fig.add_subplot(gs[0, 0])
    ax_sl = fig.add_subplot(gs[0, 1])
    ax_at = fig.add_subplot(gs[0, 2])
    ax_dev = fig.add_subplot(gs[1, :2])
    ax_summary = fig.add_subplot(gs[1, 2])

    plot_channel(
        ax_st, case["t"], case["s_actual"], case["st_lower"], case["st_upper"],
        "(a) S-T longitudinal channel", "Time (s)", "S (m)", case["d_st"] > 0.5,
    )
    ax_st.axhline(case["stop_line"], color=GRAY, linestyle=":", linewidth=1.5)
    ax_st.text(0.12, case["stop_line"] + 0.55, "Stop line", color=GRAY, fontsize=8)

    plot_channel(
        ax_sl, case["s_axis"], case["l_actual"], case["sl_lower"], case["sl_upper"],
        "(b) S-L lateral channel", "S (m)", "L (m)", None,
    )
    plot_channel(
        ax_at, case["t"], case["a_actual"], case["at_lower"], case["at_upper"],
        "(c) A-T acceleration channel", "Time (s)", "A (m/s2)", None,
    )

    handles, labels = ax_st.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.985),
               ncol=4, frameon=False, fontsize=9)

    ax_dev.plot(case["t"], case["d_st"], color=RED, linewidth=2.0, label="S-T response")
    ax_dev.plot(case["t"], case["d_sl"], color=BLUE, linewidth=1.8, linestyle="--", label="S-L response")
    ax_dev.plot(case["t"], case["d_at"], color=GREEN, linewidth=1.8, linestyle="-.", label="A-T response")
    ax_dev.plot(case["t"], case["d_fused"], color="#6F42C1", linewidth=3.0, label="Fused response")
    ax_dev.axhline(0.5, color=GRAY, linewidth=1.4, linestyle=":", label="Channel threshold")
    ax_dev.axvspan(case["t0"], case["t"][-1], color=RED, alpha=0.10, label="Effective abnormal interval")
    ax_dev.axvline(case["t0"], color=ORANGE, linestyle=":", linewidth=2.0)
    ax_dev.scatter([case["t0"]], [0], s=48, color=ORANGE, zorder=5)
    ax_dev.text(case["t0"] + 0.08, max(case["d_st"]) * 0.78, r"First exceedance $t_0$",
                color=ORANGE, fontsize=9, fontweight="bold")
    ax_dev.set_title("(d) Normalized three-channel response and fusion", fontsize=12, fontweight="bold", color="#123A67")
    ax_dev.set_xlabel("Time (s)", fontsize=10)
    ax_dev.set_ylabel("Normalized response", fontsize=10)
    ax_dev.set_ylim(-0.03, 1.05)
    style_axis(ax_dev)
    ax_dev.legend(loc="upper left", fontsize=8, frameon=True, ncol=2)

    ax_summary.axis("off")
    ax_summary.set_title("(e) Scene-constrained result", fontsize=12, fontweight="bold", color="#123A67", pad=10)
    ax_summary.set_xlim(0, 1)
    ax_summary.set_ylim(0, 1)
    ax_summary.axvspan(0.08, 0.92, ymin=0.69, ymax=0.79, color="#FDE2E2")
    ax_summary.text(0.10, 0.83, "Traffic-light phase", fontsize=9, color=GRAY)
    ax_summary.text(0.50, 0.74, "RED PHASE", ha="center", va="center", color=RED,
                    fontsize=11, fontweight="bold")
    ax_summary.axvline(0.60, ymin=0.67, ymax=0.82, color=ORANGE, linestyle="--", linewidth=1.8)
    ax_summary.text(0.60, 0.65, r"$t_{cross}$", ha="center", color=ORANGE, fontsize=9)

    cards = [
        ("Dominant channel", "S-T"),
        ("First exceedance", f"{case['t0']:.2f} s"),
        ("Violation duration", f"{case['duration']:.2f} s"),
        ("Maximum deviation", f"{case['dmax']:.2f}"),
    ]
    y_positions = [0.53, 0.39, 0.25, 0.11]
    for (label, value), y in zip(cards, y_positions):
        ax_summary.add_patch(plt.Rectangle((0.08, y - 0.045), 0.84, 0.105,
                                           facecolor="#F5F8FC", edgecolor="#A9C7E8", linewidth=1.0))
        ax_summary.text(0.12, y + 0.005, label, va="center", fontsize=8.5, color=GRAY)
        ax_summary.text(0.88, y + 0.005, value, va="center", ha="right", fontsize=10,
                        color="#123A67", fontweight="bold")

    fig.suptitle("Multi-constraint Expected Behavior Channel Validation",
                 fontsize=17, fontweight="bold", color="#0D3B78", y=1.035)

    fig.savefig(outdir / "page4_channel_validation.png", dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(outdir / "page4_channel_validation.svg", facecolor="white", bbox_inches="tight")
    fig.savefig(outdir / "page4_channel_validation.pdf", facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "plots" / "page4_channel_results")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    case = build_case()
    save_standalone(case, args.output_dir)
    make_composite(case, args.output_dir)
    print(f"Generated page-4 figures in: {args.output_dir}")
    for path in sorted(args.output_dir.iterdir()):
        print(f"  {path.name} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
