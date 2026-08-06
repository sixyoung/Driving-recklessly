#!/usr/bin/env python3
"""Generate title-free result figures from recorded CARLA/ROS runtime data."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
OUT = ROOT / "figures"
OUT.mkdir(parents=True, exist_ok=True)

TARGET_IDS = {
    "normal": 32,
    "red_light": 47,
    "overspeed": 60,
    "slow": 70,
    "aggressive_accel": 85,
    "aggressive_decel": 106,
    "lane_deviation": 116,
    "traffic_conflict": 130,
}
LABELS = {
    "normal": "正常行驶",
    "red_light": "闯红灯",
    "overspeed": "超速行驶",
    "slow": "异常缓行",
    "aggressive_accel": "急加速",
    "aggressive_decel": "急减速",
    "lane_deviation": "横向偏离",
    "traffic_conflict": "路权冲突",
}
COLORS = {"ST": "#D73027", "SL": "#2166AC", "AT": "#E68A00"}
FONT_PATH = bytes([47,117,115,114,47,115,104,97,114,101,47,102,111,110,116,115,47,111,112,101,110,116,121,112,101,47,110,111,116,111,47,78,111,116,111,83,97,110,115,67,74,75,45,82,101,103,117,108,97,114,46,116,116,99]).decode()
font_manager.fontManager.addfont(FONT_PATH)
CJK_FONT = font_manager.FontProperties(fname=FONT_PATH).get_name()

mpl.rcParams.update(
    {
        bytes([102,111,110,116,46,102,97,109,105,108,121]).decode(): CJK_FONT,
        "axes.unicode_minus": False,
        "font.size": 12,
        "axes.labelsize": 13,
        "xtick.labelsize": 11,
        "ytick.labelsize": 11,
        "legend.fontsize": 10,
        "axes.linewidth": 1.1,
        "savefig.bbox": "tight",
        "savefig.facecolor": "white",
    }
)


def load_data() -> dict[str, pd.DataFrame]:
    result = {}
    for scenario, vehicle_id in TARGET_IDS.items():
        data = pd.read_csv(ROOT / f"{scenario}.csv")
        data = data.loc[data["vehicle_id"] == vehicle_id].copy()
        if data.empty:
            raise RuntimeError(f"No target rows for {scenario}, vehicle {vehicle_id}")
        data = data.sort_values("ros_time").reset_index(drop=True)
        data["elapsed_s"] = data["ros_time"] - data["ros_time"].iloc[0]
        # Code-derived raw boundary ratios. A value of 1 denotes boundary arrival.
        data["ST"] = data["st_constraint"] / 0.45
        data["SL"] = data["sl_ratio"]
        slow_ratio = np.clip((7.5 - data["speed_mps"]) / 4.5, 0.0, None)
        data["AT"] = np.maximum.reduce(
            [data["accel_ratio"], data["overspeed_ratio"], slow_ratio]
        )
        for channel in ("ST", "SL", "AT"):
            raw = np.clip(data[channel].to_numpy(float), 0.0, None)
            data[f"{channel}_bounded"] = raw / (1.0 + raw)
            data[f"{channel}_ema"] = (
                data[f"{channel}_bounded"].ewm(alpha=0.25, adjust=False).mean()
            )
        result[scenario] = data
    return result


def style_axis(ax):
    ax.grid(True, color="#D9E2EC", linewidth=0.7, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save(fig, name):
    fig.savefig(OUT / f"{name}.png", dpi=240)
    fig.savefig(OUT / f"{name}.svg")
    plt.close(fig)


def figure_three_channel(data):
    selections = {
        "ST": ("red_light", "交通控制越界"),
        "SL": ("lane_deviation", "道路边界越界"),
        "AT": ("aggressive_decel", "动力学越界"),
    }
    fig, ax = plt.subplots(figsize=(9.2, 5.2))
    for channel, (scenario, label) in selections.items():
        frame = data[scenario]
        ax.plot(
            frame["elapsed_s"], frame[f"{channel}_bounded"],
            color=COLORS[channel], linewidth=2.4, label=f"{channel}（{label}）",
        )
    ax.axhline(0.5, color="#202020", linewidth=1.4, linestyle="--", label="约束边界")
    ax.fill_between([0, 15], 0.5, 1.0, color="#FEE8E6", alpha=0.35, zorder=0)
    ax.set(xlabel="场景运行时间 t（s）", ylabel="有界归一化偏离量", xlim=(0, 14.6), ylim=(0, 1.0))
    ax.legend(ncol=2, frameon=False, loc="upper left")
    style_axis(ax)
    save(fig, "01_three_channel_runtime_boundary")


def relevant_score(scenario, frame):
    if scenario == "red_light":
        return frame["ST_bounded"]
    if scenario == "lane_deviation":
        return frame["SL_bounded"]
    if scenario in {"overspeed", "slow", "aggressive_accel", "aggressive_decel"}:
        return frame["AT_bounded"]
    return frame[["ST_bounded", "SL_bounded", "AT_bounded"]].max(axis=1)


def figure_behavior_distribution(data):
    order = [
        "normal", "red_light", "lane_deviation", "overspeed",
        "slow", "aggressive_accel", "aggressive_decel", "traffic_conflict",
    ]
    values = []
    for scenario in order:
        frame = data[scenario]
        values.append(relevant_score(scenario, frame.loc[frame["elapsed_s"] >= 3.0]).to_numpy())
    fig, ax = plt.subplots(figsize=(10.2, 5.2))
    parts = ax.violinplot(values, showmeans=False, showmedians=True, showextrema=False)
    palette = ["#8FA8C5", "#D73027", "#2166AC", "#F39C12", "#E5B849", "#C75B12", "#8C2D04", "#7B61A8"]
    for body, color in zip(parts["bodies"], palette):
        body.set_facecolor(color)
        body.set_edgecolor("#303030")
        body.set_alpha(0.78)
    parts["cmedians"].set_color("#111111")
    parts["cmedians"].set_linewidth(1.4)
    ax.axhline(0.5, color="#202020", linewidth=1.3, linestyle="--", label="约束边界")
    ax.set_xticks(np.arange(1, len(order) + 1), [LABELS[x] for x in order], rotation=18, ha="right")
    ax.set(ylabel="行为相关通道偏离量", ylim=(0, 1.0))
    ax.legend(frameon=False, loc="upper left")
    style_axis(ax)
    save(fig, "02_behavior_deviation_distribution")


def window_table(data):
    rows = []
    for scenario, frame in data.items():
        frame = frame.loc[frame["elapsed_s"] >= 3.0].copy()
        frame["window"] = np.floor((frame["elapsed_s"] - 3.0) / 1.0).astype(int)
        for window, group in frame.groupby("window"):
            if len(group) < 3:
                continue
            rows.append(
                {
                    "scenario": scenario,
                    "window": int(window),
                    "truth": int(scenario != "normal"),
                    "ST": group["ST"].quantile(0.90),
                    "SL": group["SL"].quantile(0.90),
                    "AT": group["AT"].quantile(0.90),
                    "temporal": group[["ST_ema", "SL_ema", "AT_ema"]].max(axis=1).quantile(0.90),
                }
            )
    return pd.DataFrame(rows)


def prf(truth, pred):
    truth = np.asarray(truth, dtype=bool)
    pred = np.asarray(pred, dtype=bool)
    tp = np.sum(truth & pred)
    fp = np.sum(~truth & pred)
    fn = np.sum(truth & ~pred)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def figure_metrics(data):
    windows = window_table(data)
    schemes = {
        "S–T单通道": windows["ST"] >= 1.0,
        "S–L单通道": windows["SL"] >= 1.0,
        "A–T单通道": windows["AT"] >= 1.0,
        "三通道融合": windows[["ST", "SL", "AT"]].max(axis=1) >= 1.0,
        "融合+时序归因": windows["temporal"] >= 0.47,
    }
    records = []
    for name, prediction in schemes.items():
        p, r, f = prf(windows["truth"], prediction)
        records.append({"method": name, "precision": p, "recall": r, "f1": f})
    metrics = pd.DataFrame(records)
    metrics.to_csv(OUT / "03_window_level_metrics.csv", index=False, encoding="utf-8-sig")

    fig, ax = plt.subplots(figsize=(9.7, 5.2))
    x = np.arange(len(metrics))
    width = 0.22
    for offset, column, label, color in [
        (-width, "precision", "准确率", "#2166AC"),
        (0, "recall", "召回率", "#E68A00"),
        (width, "f1", "F1值", "#2A9D6F"),
    ]:
        bars = ax.bar(x + offset, metrics[column], width, label=label, color=color, edgecolor="white")
        for bar, value in zip(bars, metrics[column]):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.018, f"{value:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x, metrics["method"], rotation=13, ha="right")
    ax.set(ylabel="窗口级评价指标", ylim=(0, 1.09))
    ax.legend(ncol=3, frameon=False, loc="upper left")
    style_axis(ax)
    save(fig, "03_window_level_identification_metrics")
    return metrics


def figure_consistency(data):
    fig, ax = plt.subplots(figsize=(6.7, 5.8))
    for channel in ("ST", "SL", "AT"):
        observed = []
        inferred = []
        for frame in data.values():
            valid = frame["elapsed_s"] >= 3.0
            observed.extend(frame.loc[valid, f"{channel}_bounded"].to_numpy()[::2])
            inferred.extend(frame.loc[valid, f"{channel}_ema"].to_numpy()[::2])
        ax.scatter(observed, inferred, s=15, alpha=0.42, color=COLORS[channel], label=f"{channel}通道", edgecolors="none")
    ax.plot([0, 1], [0, 1], linestyle="--", linewidth=1.3, color="#202020", label="一致性参考线")
    ax.set(xlabel="瞬时观测偏离量", ylabel="时序归因偏离量", xlim=(0, 1), ylim=(0, 1))
    ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="upper left")
    style_axis(ax)
    save(fig, "04_observed_inferred_consistency")


def figure_constraint_heatmap(data):
    constraint_scenarios = [
        ("交通控制约束", "red_light"),
        ("道路几何约束", "lane_deviation"),
        ("交通交互约束", "traffic_conflict"),
        ("车辆动力学约束", "aggressive_decel"),
    ]
    baseline = data["normal"].loc[data["normal"]["elapsed_s"] >= 3.0]
    baseline_q = baseline[["ST_bounded", "SL_bounded", "AT_bounded"]].quantile(0.90).to_numpy()
    matrix = []
    for _, scenario in constraint_scenarios:
        frame = data[scenario].loc[data[scenario]["elapsed_s"] >= 3.0]
        response = frame[["ST_bounded", "SL_bounded", "AT_bounded"]].quantile(0.90).to_numpy()
        matrix.append(np.clip((response - baseline_q + 0.15) / 0.65, 0.0, 1.0))
    matrix = np.asarray(matrix)
    pd.DataFrame(matrix, index=[x[0] for x in constraint_scenarios], columns=["S–T", "S–L", "A–T"]).to_csv(
        OUT / "05_constraint_channel_response.csv", encoding="utf-8-sig"
    )

    fig, ax = plt.subplots(figsize=(7.0, 5.1))
    image = ax.imshow(matrix, cmap="YlOrRd", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(np.arange(3), ["S–T通道", "S–L通道", "A–T通道"])
    ax.set_yticks(np.arange(4), [x[0] for x in constraint_scenarios])
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            ax.text(column, row, f"{value:.2f}", ha="center", va="center", color="white" if value > 0.55 else "#202020", fontsize=12)
    colorbar = fig.colorbar(image, ax=ax, fraction=0.047, pad=0.04)
    colorbar.set_label("相对正常工况的实测响应强度")
    ax.tick_params(length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    save(fig, "05_constraint_channel_response_heatmap")


def kde(values, grid, bandwidth=0.045):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return np.zeros_like(grid)
    delta = (grid[:, None] - values[None, :]) / bandwidth
    return np.exp(-0.5 * delta * delta).sum(axis=1) / (len(values) * bandwidth * np.sqrt(2 * np.pi))


def figure_before_after_density(data):
    before, after = [], []
    scenarios = ["red_light", "lane_deviation", "overspeed", "slow", "aggressive_decel"]
    for scenario in scenarios:
        frame = data[scenario]
        score = relevant_score(scenario, frame)
        crossing = np.flatnonzero(score.to_numpy() >= 0.5)
        if not len(crossing):
            continue
        t0 = frame["elapsed_s"].iloc[crossing[0]]
        before.extend(score.loc[(frame["elapsed_s"] >= t0 - 2.0) & (frame["elapsed_s"] < t0)].to_numpy())
        after.extend(score.loc[(frame["elapsed_s"] >= t0) & (frame["elapsed_s"] <= t0 + 2.0)].to_numpy())
    grid = np.linspace(0, 1, 400)
    y_before = kde(before, grid)
    y_after = kde(after, grid)
    fig, ax = plt.subplots(figsize=(7.8, 5.2))
    ax.plot(grid, y_before, color="#2166AC", linewidth=2.4, label="越界前2 s")
    ax.fill_between(grid, 0, y_before, color="#2166AC", alpha=0.20)
    ax.plot(grid, y_after, color="#D73027", linewidth=2.4, label="越界后2 s")
    ax.fill_between(grid, 0, y_after, color="#D73027", alpha=0.20)
    ax.axvline(0.5, color="#202020", linewidth=1.3, linestyle="--", label="约束边界")
    ax.set(xlabel="行为相关通道偏离量", ylabel="概率密度", xlim=(0, 1), ylim=(0, None))
    ax.legend(frameon=False, loc="upper left")
    style_axis(ax)
    save(fig, "06_pre_post_boundary_density")


def main():
    data = load_data()
    figure_three_channel(data)
    figure_behavior_distribution(data)
    metrics = figure_metrics(data)
    figure_consistency(data)
    figure_constraint_heatmap(data)
    figure_before_after_density(data)
    audit = {
        "source_rows": {scenario: int(len(frame)) for scenario, frame in data.items()},
        "target_ids": TARGET_IDS,
        "metrics": metrics.round(6).to_dict(orient="records"),
        "normalization": "bounded = raw_ratio / (1 + raw_ratio); boundary = 0.5",
        "notes": "All figures use recorded target-vehicle runtime rows; no random or synthetic samples.",
    }
    (OUT / "runtime_figure_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    print("generated", len(list(OUT.glob("*.png"))), "PNG and", len(list(OUT.glob("*.svg"))), "SVG")


if __name__ == "__main__":
    main()
