#!/usr/bin/env python3
"""Generate a title-free heatmap of constraint impact on three behavior channels."""

from pathlib import Path
import argparse
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_page4_figures import build_case
from generate_method_page_six_figures import (
    ZH, BLUE, ORANGE, RED, GREEN, configure_style, save,
    build_method_constraints,
)
from generate_method_page_separated_figures import add_vector_colorbar


def smooth_boundary_response(values, threshold, scale):
    """Map constraint strength to a smooth response centered on its boundary."""
    return 1.0 / (1.0 + np.exp(-(values - threshold) / scale))


def first_rising_crossing(t, values, threshold):
    indices = np.flatnonzero((values[:-1] < threshold) & (values[1:] >= threshold))
    return float(t[indices[0] + 1]) if indices.size else None


def plot_constraint_impact_heatmap(case, constraints, outdir):
    t = case["t"]
    constraint_names = [
        "\u4ea4\u901a\u63a7\u5236\u7ea6\u675f",
        "\u9053\u8def\u51e0\u4f55\u7ea6\u675f",
        "\u4ea4\u901a\u4ea4\u4e92\u7ea6\u675f",
        "\u8f66\u8f86\u52a8\u529b\u5b66\u7ea6\u675f",
    ]
    constraint_keys = [
        "traffic_control", "road_geometry",
        "traffic_interaction", "vehicle_dynamics",
    ]

    # Activation boundaries for the representative mechanism data.
    thresholds = np.array([0.50, 0.75, 0.28, 0.22])
    transition_scales = np.array([0.035, 0.004, 0.025, 0.008])

    # Rows: four constraints; columns: S-T, S-L and A-T channels.
    # Coefficients are identical to the channel-width update equations.
    coupling = np.array([
        [0.78, 0.00, 0.00],
        [0.00, 0.28, 0.00],
        [0.08, 0.18, 0.08],
        [0.00, 0.00, 0.42],
    ])

    constraint_values = np.vstack([constraints[key] for key in constraint_keys])
    boundary_response = np.vstack([
        smooth_boundary_response(values, threshold, scale)
        for values, threshold, scale in zip(
            constraint_values, thresholds, transition_scales
        )
    ])

    # Aggregate four boundary responses into exactly three channel rows.
    channel_heat = coupling.T @ boundary_response
    channel_heat /= np.maximum(coupling.sum(axis=0)[:, None], 1e-9)
    channel_heat = np.clip(channel_heat, 0.0, 1.0)

    # Explicit bin edges guarantee three complete and equally high heat bands.
    dt = np.diff(t)
    x_edges = np.r_[
        t[0] - dt[0] / 2.0,
        0.5 * (t[:-1] + t[1:]),
        t[-1] + dt[-1] / 2.0,
    ]
    y_edges = np.arange(4) - 0.5

    fig, ax = plt.subplots(figsize=(7.2, 3.8), constrained_layout=True)
    ax.pcolormesh(
        x_edges, y_edges, channel_heat,
        shading="flat", cmap="Blues", vmin=0.0, vmax=1.0,
        rasterized=False,
    )
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(2.5, -0.5)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(
        ["S\u2013T\u901a\u9053", "S\u2013L\u901a\u9053", "A\u2013T\u901a\u9053"],
        fontproperties=ZH, fontsize=9,
    )
    ax.set_xlabel("\u65f6\u95f4 t (s)", fontproperties=ZH, fontsize=10)
    ax.set_ylabel("\u884c\u4e3a\u901a\u9053", fontproperties=ZH, fontsize=10)
    ax.hlines([0.5, 1.5], t[0], t[-1], colors="white", linewidth=2.0)

    boundary_colors = [RED, BLUE, ORANGE, GREEN]
    handles = []
    crossing_times = {}
    for name, values, threshold, color in zip(
        constraint_names, constraint_values, thresholds, boundary_colors
    ):
        crossing = first_rising_crossing(t, values, threshold)
        crossing_times[name] = crossing
        if crossing is None:
            continue
        ax.axvline(
            crossing, color=color, linestyle="--",
            linewidth=1.25, alpha=0.95,
        )
        handles.append(Line2D(
            [0], [0], color=color, linestyle="--", linewidth=1.5,
            label=name + "\u8d8a\u754c",
        ))

    ax.legend(
        handles=handles, loc="upper center", bbox_to_anchor=(0.5, 1.15),
        ncol=4, prop=ZH, fontsize=7.6, frameon=False,
        handlelength=1.8, columnspacing=1.2,
    )
    add_vector_colorbar(
        ax, plt.cm.Blues, [0, 0.5, 1.0], ["0.00", "0.50", "1.00"],
        "\u7ea6\u675f\u5f71\u54cd\u5f3a\u5ea6",
    )
    save(fig, outdir, "09_constraint_impact_on_three_channels")
    return channel_heat, crossing_times


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path,
        default=SCRIPT_DIR.parent / "plots" / "method_page_separated_figures",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    case = build_case()
    constraints = build_method_constraints(case)
    heat, crossings = plot_constraint_impact_heatmap(
        case, constraints, args.output_dir
    )
    print("shape:", heat.shape)
    print("row ranges:", [
        (float(row.min()), float(row.max())) for row in heat
    ])
    print("crossings:", crossings)


if __name__ == "__main__":
    main()
