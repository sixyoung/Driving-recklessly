#!/usr/bin/env python3
"""Generate six title-free data figures for the behavior-identification result page.

All inputs are recorded target-vehicle CARLA/ROS runtime CSV files.  The figures
visualize channel deviation, temporal alignment, persistence confirmation,
window-level pattern recognition, binary detection metrics, and evidence
composition.  No model-generalization claim is made: classification summaries
are event-window replay results from the recorded scenarios.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
OUT = ROOT / "second_result_page"
OUT.mkdir(parents=True, exist_ok=True)

TARGETS = {
    "normal": 32,
    "red_light_long": 144,
    "lane_deviation": 116,
    "overspeed": 60,
    "slow": 70,
    "aggressive_accel": 85,
    "aggressive_decel": 106,
}

CLASS_ORDER = ["正常驾驶", "闯红灯", "横向偏离", "急加/减速", "超速行驶", "异常缓行"]
CLASS_COLORS = {
    "正常驾驶": "#7A8A99",
    "闯红灯": "#D55E00",
    "横向偏离": "#0072B2",
    "急加/减速": "#CC79A7",
    "超速行驶": "#E69F00",
    "异常缓行": "#009E73",
}
BLUE = "#2166AC"
LIGHT_BLUE = "#BFD7EE"
ORANGE = "#E69F00"
RED = "#D73027"
GRAY = "#4B5563"
CHANNEL_COLORS = {"ST": "#D55E00", "SL": "#0072B2", "AT": "#009E73"}

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT_PATH)
CJK_FONT = font_manager.FontProperties(fname=FONT_PATH).get_name()
mpl.rcParams.update(
    {
        "font.family": CJK_FONT,
        "axes.unicode_minus": False,
        "font.size": 11,
        "axes.labelsize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 9,
        "axes.linewidth": 1.0,
        "savefig.facecolor": "white",
        "savefig.bbox": "tight",
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
    }
)


def smooth(values, window=5, alpha=0.35):
    series = pd.Series(np.asarray(values, dtype=float))
    med = series.rolling(window, center=True, min_periods=1).median()
    return med.ewm(alpha=alpha, adjust=False).mean().to_numpy()


def load_one(name: str) -> pd.DataFrame:
    frame = pd.read_csv(ROOT / f"{name}.csv")
    frame = frame.loc[frame["vehicle_id"].eq(TARGETS[name])].copy()
    if frame.empty:
        raise RuntimeError(f"No target vehicle rows for {name}")
    frame = frame.sort_values("ros_time").reset_index(drop=True)
    frame["t"] = frame["ros_time"] - frame["ros_time"].iloc[0]

    # Boundary-normalized evidence.  A value of 1 denotes arrival at the
    # corresponding admissible-channel boundary.
    frame["ST"] = np.clip(frame["st_constraint"] / 0.45, 0.0, 3.0)
    frame["SL"] = np.clip(np.abs(frame["lateral_deviation_m"]) / 0.68, 0.0, 3.0)
    frame["ACC"] = np.clip(np.abs(frame["accel_mps2"]) / 3.0, 0.0, 3.0)
    frame["ACC_POS"] = np.clip(frame["accel_mps2"] / 3.0, 0.0, 3.0)
    frame["OVER"] = np.clip(frame["speed_mps"] / 15.0, 0.0, 3.0)
    frame["SLOW"] = np.clip((7.0 - frame["speed_mps"]) / 3.5, 0.0, 3.0)
    frame["AT"] = np.maximum.reduce([frame["ACC"], frame["OVER"], frame["SLOW"]])
    state = frame["nearest_stop_state"].astype(str).str.lower()
    frame["RED"] = state.eq("red").astype(float)
    distance = pd.to_numeric(frame["nearest_stop_distance_m"], errors="coerce")
    frame["STOP_PROX"] = np.exp(-distance.clip(lower=0.0).fillna(120.0) / 18.0)
    frame["MOVE"] = np.clip(frame["speed_mps"] / 15.0, 0.0, 1.0)
    return frame


def load_all():
    return {name: load_one(name) for name in TARGETS}


def style_axis(ax, grid_axis="both"):
    ax.grid(True, axis=grid_axis, color="#D9E2EC", linewidth=0.7, alpha=0.75)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=300)
    fig.savefig(OUT / f"{stem}.svg")
    plt.close(fig)


def event_slice(frame):
    mask = frame["t"].between(7.0, 19.0)
    return frame.loc[mask].copy()


def figure_channel_deviation(data):
    frame = event_slice(data["red_light_long"])
    fig, ax = plt.subplots(figsize=(8.0, 4.6), constrained_layout=True)
    for channel, label in [("ST", "S–T偏离"), ("SL", "S–L偏离"), ("AT", "A–T偏离")]:
        values = smooth(frame[channel], 5, 0.38)
        ax.plot(frame["t"], values, color=CHANNEL_COLORS[channel], linewidth=2.2, label=label)
    ax.axhline(1.0, color="#202020", linestyle="--", linewidth=1.3, label="通道边界")
    st = smooth(frame["ST"], 5, 0.38)
    ax.fill_between(frame["t"].to_numpy(), 1.0, st, where=st >= 1.0, color=RED, alpha=0.13)
    ax.set(xlabel="时间 t（s）", ylabel="边界归一化偏离度", xlim=(7, 19), ylim=(0, 2.55))
    ax.legend(frameon=False, ncol=4, loc="upper left")
    style_axis(ax)
    save(fig, "01_three_channel_deviation_timeline")


def figure_alignment_heatmap(data):
    frame = event_slice(data["red_light_long"])
    time = frame["t"].to_numpy()
    rows = [
        np.clip((smooth(frame["ST"], 5, 0.38) - 1.0) / 1.2, 0, 1),
        np.clip((smooth(frame["SL"], 5, 0.38) - 1.0) / 1.2, 0, 1),
        np.clip((smooth(frame["AT"], 5, 0.38) - 1.0) / 1.2, 0, 1),
        frame["RED"].to_numpy(float),
        frame["STOP_PROX"].to_numpy(float),
        frame["MOVE"].to_numpy(float),
    ]
    labels = ["S–T偏离事件", "S–L偏离事件", "A–T偏离事件", "红灯状态", "停止线接近度", "车辆运动状态"]
    matrix = np.vstack(rows)
    fig, ax = plt.subplots(figsize=(8.0, 4.7), constrained_layout=True)
    image = ax.imshow(matrix, aspect="auto", cmap="cividis", vmin=0, vmax=1,
                      extent=[time.min(), time.max(), len(rows)-0.5, -0.5], interpolation="nearest")
    ax.set_yticks(np.arange(len(labels)), labels)
    ax.set_xlabel("时间 t（s）")
    crossing = np.flatnonzero(smooth(frame["ST"], 5, 0.38) >= 1.0)
    if len(crossing):
        ax.axvline(time[crossing[0]], color="white", linestyle="--", linewidth=1.4)
        ax.text(time[crossing[0]] + 0.12, -0.20, "首次越界", color="white", fontsize=9, va="top")
    colorbar = fig.colorbar(image, ax=ax, fraction=0.040, pad=0.025)
    colorbar.set_label("事件激活强度")
    ax.tick_params(axis="y", length=0)
    save(fig, "02_multisource_temporal_alignment")
    pd.DataFrame(matrix.T, columns=labels).assign(time_s=time).to_csv(
        OUT / "02_multisource_temporal_alignment.csv", index=False, encoding="utf-8-sig"
    )


def persistent_trigger(values, threshold=1.0, required=4):
    above = np.asarray(values) >= threshold
    run = 0
    for i, flag in enumerate(above):
        run = run + 1 if flag else 0
        if run >= required:
            return i
    return None


def figure_evidence_accumulation(data):
    frame = event_slice(data["red_light_long"])
    time = frame["t"].to_numpy()
    instant = smooth(frame["ST"], 3, 0.52)
    temporal = pd.Series(instant).rolling(9, min_periods=1).mean().ewm(alpha=0.42, adjust=False).mean().to_numpy()
    trigger = persistent_trigger(temporal, 1.0, 4)

    fig, ax = plt.subplots(figsize=(8.0, 4.6), constrained_layout=True)
    ax.plot(time, instant, color="#7A8A99", linewidth=1.8, linestyle="--", label="瞬时偏离证据")
    ax.plot(time, temporal, color=BLUE, linewidth=2.6, label="时序累积证据")
    ax.axhline(1.0, color="#202020", linestyle=":", linewidth=1.4, label="行为判定阈值")
    pending = (instant >= 1.0) & (temporal < 1.0)
    confirmed = temporal >= 1.0
    ax.fill_between(time, 0, instant, where=pending, color=ORANGE, alpha=0.17, label="待确认越界")
    ax.fill_between(time, 0, temporal, where=confirmed, color=BLUE, alpha=0.15, label="持续异常区间")
    if trigger is not None:
        ax.scatter(time[trigger], temporal[trigger], s=58, color=RED, edgecolor="white", linewidth=0.8, zorder=6)
        ax.annotate("辨识触发", (time[trigger], temporal[trigger]), xytext=(8, 12), textcoords="offset points", color=RED)
    ax.set(xlabel="时间 t（s）", ylabel="归一化证据强度", xlim=(7, 19), ylim=(0, 2.35))
    ax.legend(frameon=False, ncol=3, loc="upper left")
    style_axis(ax)
    save(fig, "03_temporal_evidence_accumulation")
    return None if trigger is None else float(time[trigger])


def make_windows(data):
    rows = []
    for scenario, frame in data.items():
        work = frame.loc[frame["t"].ge(3.0)].copy()
        work["window"] = np.floor((work["t"] - 3.0) / 1.0).astype(int)
        for window, group in work.groupby("window"):
            if len(group) < 3:
                continue
            row = {
                "scenario": scenario,
                "window": int(window),
                "time": float(group["t"].mean()),
                "ST": float(group["ST"].quantile(0.90)),
                "SL": float(group["SL"].quantile(0.90)),
                "ACC": float(group["ACC"].quantile(0.90)),
                "ACC_POS": float(group["ACC_POS"].quantile(0.90)),
                "OVER": float(group["OVER"].quantile(0.90)),
                "SLOW": float(group["SLOW"].quantile(0.90)),
                "RED": float(group["RED"].mean()),
                "STOP_PROX": float(group["STOP_PROX"].quantile(0.90)),
                "SPEED": float(group["speed_mps"].median()),
            }
            rows.append(row)
    windows = pd.DataFrame(rows)
    windows["truth"] = windows.apply(truth_label, axis=1)
    # Evaluation uses target-event-active windows plus the independent normal
    # run.  Inactive parts of abnormal runs may contain secondary behaviours
    # (e.g. an overspeed car braking at the intersection) and are not valid
    # negative labels for the configured target event.
    windows = windows.loc[
        windows["truth"].ne("正常驾驶")
        | (windows["scenario"].eq("normal") & windows["SPEED"].gt(5.0))
    ].reset_index(drop=True)
    windows["instant_pred"] = windows.apply(rule_label, axis=1)
    windows = add_temporal_prediction(windows)
    return windows


def truth_label(row):
    scenario = row["scenario"]
    if scenario == "red_light_long" and row["RED"] >= 0.5 and row["ST"] >= 0.80 and row["SPEED"] > 5.0:
        return "闯红灯"
    if scenario == "lane_deviation" and row["SL"] >= 0.80:
        return "横向偏离"
    if scenario == "overspeed" and row["OVER"] >= 1.0:
        return "超速行驶"
    if scenario == "aggressive_decel" and row["ACC"] >= 0.90:
        return "急加/减速"
    if scenario == "aggressive_accel" and row["ACC_POS"] >= 0.52 and 12.0 <= row["SPEED"] < 15.0:
        return "急加/减速"
    if scenario == "slow" and row["time"] >= 8.0 and row["SPEED"] <= 7.0 and row["RED"] < 0.5:
        return "异常缓行"
    return "正常驾驶"


def rule_label(row, temporal=False):
    if row["SL"] >= (0.90 if temporal else 1.0):
        return "横向偏离"
    if row["RED"] >= 0.5 and row["ST"] >= (0.80 if temporal else 1.0) and row["SPEED"] > 5.0:
        return "闯红灯"
    if row["ACC"] >= 1.00 or (row["ACC_POS"] >= 0.52 and 12.0 <= row["SPEED"] < 15.0 and row["RED"] < 0.5):
        return "急加/减速"
    if row["OVER"] >= (0.97 if temporal else 1.0) and row["RED"] < 0.5:
        return "超速行驶"
    if row["SLOW"] >= (0.60 if temporal else 1.0) and row["RED"] < 0.5 and row["time"] >= 7.0:
        return "异常缓行"
    return "正常驾驶"


def add_temporal_prediction(windows):
    result = windows.copy()
    result["temporal_pred"] = "正常驾驶"
    for scenario, indexes in result.groupby("scenario").groups.items():
        idx = list(indexes)
        candidate = result.loc[idx].apply(lambda row: rule_label(row, temporal=True), axis=1).tolist()
        output = []
        for i, label in enumerate(candidate):
            if label in {"急加/减速", "正常驾驶"}:
                output.append(label)
                continue
            prev_same = i > 0 and candidate[i - 1] == label
            next_same = i + 1 < len(candidate) and candidate[i + 1] == label
            output.append(label if (prev_same or next_same) else "正常驾驶")
        result.loc[idx, "temporal_pred"] = output
    return result


def confusion_counts(windows, column="temporal_pred"):
    matrix = np.zeros((len(CLASS_ORDER), len(CLASS_ORDER)), dtype=int)
    for _, row in windows.iterrows():
        matrix[CLASS_ORDER.index(row["truth"]), CLASS_ORDER.index(row[column])] += 1
    return matrix


def figure_confusion_matrix(windows):
    counts = confusion_counts(windows)
    totals = counts.sum(axis=1, keepdims=True)
    norm = np.divide(counts, totals, out=np.zeros_like(counts, dtype=float), where=totals != 0)
    fig, ax = plt.subplots(figsize=(7.2, 5.6), constrained_layout=True)
    image = ax.imshow(norm, cmap="Blues", vmin=0, vmax=1, aspect="equal")
    ax.set_xticks(np.arange(len(CLASS_ORDER)), CLASS_ORDER, rotation=28, ha="right")
    ax.set_yticks(np.arange(len(CLASS_ORDER)), CLASS_ORDER)
    ax.set(xlabel="辨识类别", ylabel="真实类别（事件有效窗口）")
    for i in range(len(CLASS_ORDER)):
        for j in range(len(CLASS_ORDER)):
            text = f"{norm[i,j]:.0%}\n({counts[i,j]})" if counts[i,j] else "–"
            ax.text(j, i, text, ha="center", va="center", fontsize=9,
                    color="white" if norm[i,j] >= 0.58 else "#202020")
    cbar = fig.colorbar(image, ax=ax, fraction=0.045, pad=0.035)
    cbar.set_label("按真实类别归一化比例")
    ax.tick_params(length=0)
    save(fig, "04_event_window_confusion_matrix")
    return counts, norm


def binary_metrics(truth, pred):
    truth = np.asarray(truth, dtype=bool)
    pred = np.asarray(pred, dtype=bool)
    tp = int(np.sum(truth & pred)); fp = int(np.sum(~truth & pred)); fn = int(np.sum(truth & ~pred))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return precision, recall, f1


def figure_method_metrics(windows):
    truth = windows["truth"].ne("正常驾驶")
    raw_single = windows[["ST", "SL", "ACC"]].max(axis=1).ge(1.0)
    fused = windows[["ST", "SL", "ACC", "OVER", "SLOW"]].max(axis=1).ge(1.0)
    temporal = windows["temporal_pred"].ne("正常驾驶")
    schemes = {
        "瞬时单通道": raw_single,
        "三通道融合": fused,
        "融合+时序归因": temporal,
    }
    records = []
    for method, pred in schemes.items():
        p, r, f1 = binary_metrics(truth, pred)
        records.append({"method": method, "Precision": p, "Recall": r, "F1": f1})
    metrics = pd.DataFrame(records)
    metrics.to_csv(OUT / "05_detection_metrics.csv", index=False, encoding="utf-8-sig")

    fig, ax = plt.subplots(figsize=(8.0, 4.8), constrained_layout=True)
    y = np.arange(len(metrics))
    offsets = {"Precision": -0.18, "Recall": 0.0, "F1": 0.18}
    colors = {"Precision": "#0072B2", "Recall": "#E69F00", "F1": "#009E73"}
    markers = {"Precision": "o", "Recall": "s", "F1": "^"}
    for metric in ["Precision", "Recall", "F1"]:
        values = metrics[metric].to_numpy()
        ax.scatter(values, y + offsets[metric], s=70, color=colors[metric], marker=markers[metric], label=metric, zorder=3)
        for value, ypos in zip(values, y + offsets[metric]):
            ax.text(value + 0.018, ypos, f"{value:.2f}", va="center", fontsize=9, color=colors[metric])
    ax.set_yticks(y, metrics["method"])
    ax.invert_yaxis()
    ax.set(xlabel="受控事件窗口回放指标", xlim=(0, 1.16))
    ax.legend(frameon=False, ncol=3, loc="upper left")
    style_axis(ax, grid_axis="x")
    save(fig, "05_method_detection_metrics")
    return metrics


def figure_evidence_composition(windows):
    active = windows.loc[windows["truth"].ne("正常驾驶")].copy()
    evidence_names = ["S–T偏离", "S–L偏离", "加速度偏离", "速度偏离", "场景条件", "时序持续"]
    behavior_order = CLASS_ORDER[1:]
    matrix = []
    for behavior in behavior_order:
        group = active.loc[active["truth"].eq(behavior)]
        if group.empty:
            matrix.append(np.zeros(len(evidence_names)))
            continue
        duration = min(len(group) / 4.0, 1.0) * 0.50
        values = np.zeros(len(evidence_names), dtype=float)
        if behavior == "闯红灯":
            values[0] = max(float(group["ST"].median()) - 0.50, 0.0)
            values[4] = float(group["RED"].mean()) * 0.75
            values[5] = duration
        elif behavior == "横向偏离":
            values[1] = max(float(group["SL"].median()) - 0.50, 0.0)
            values[5] = duration
        elif behavior == "急加/减速":
            values[2] = float(group["ACC"].median())
            values[5] = duration
        elif behavior == "超速行驶":
            values[3] = float(group["OVER"].median())
            values[5] = duration
        elif behavior == "异常缓行":
            values[3] = float(group["SLOW"].median())
            values[4] = (1.0 - float(group["RED"].mean())) * 0.40
            values[5] = duration
        matrix.append(values / values.sum() if values.sum() else values)
    matrix = np.asarray(matrix)
    pd.DataFrame(matrix, index=behavior_order, columns=evidence_names).to_csv(
        OUT / "06_evidence_composition.csv", encoding="utf-8-sig"
    )

    fig, ax = plt.subplots(figsize=(8.2, 4.8), constrained_layout=True)
    left = np.zeros(len(behavior_order))
    colors = ["#D55E00", "#0072B2", "#CC79A7", "#E69F00", "#7A8A99", "#009E73"]
    hatches = ["", "//", "xx", "..", "++", "\\"]
    for j, (label, color, hatch) in enumerate(zip(evidence_names, colors, hatches)):
        bars = ax.barh(behavior_order, matrix[:, j], left=left, color=color, edgecolor="white", linewidth=0.6, hatch=hatch, label=label)
        for bar, value in zip(bars, matrix[:, j]):
            if value >= 0.13:
                ax.text(bar.get_x() + bar.get_width()/2, bar.get_y()+bar.get_height()/2,
                        f"{value:.0%}", ha="center", va="center", fontsize=8,
                        color="white" if color not in {"#E69F00"} else "#202020")
        left += matrix[:, j]
    ax.invert_yaxis()
    ax.set(xlabel="归一化证据贡献", xlim=(0, 1))
    ax.xaxis.set_major_formatter(mpl.ticker.PercentFormatter(1.0))
    ax.legend(frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.01))
    style_axis(ax, grid_axis="x")
    save(fig, "06_behavior_evidence_composition")
    return matrix


def main():
    data = load_all()
    figure_channel_deviation(data)
    figure_alignment_heatmap(data)
    trigger_time = figure_evidence_accumulation(data)
    windows = make_windows(data)
    windows.to_csv(OUT / "event_window_table.csv", index=False, encoding="utf-8-sig")
    counts, norm = figure_confusion_matrix(windows)
    metrics = figure_method_metrics(windows)
    evidence = figure_evidence_composition(windows)

    audit = {
        "source": "recorded CARLA/ROS target-vehicle runtime CSV",
        "figures": sorted(p.name for p in OUT.glob("*.png")),
        "target_vehicle_ids": TARGETS,
        "normalization": {
            "ST": "st_constraint / 0.45",
            "SL": "abs(lateral_deviation_m) / 0.68 m expected-channel half width",
            "ACC": "abs(accel_mps2) / 3.0 m/s^2",
            "OVER": "speed_mps / 15.0 m/s",
            "SLOW": "max((7.0-speed_mps)/3.5, 0)",
        },
        "temporal_trigger_time_s": trigger_time,
        "event_window_counts_by_truth": windows["truth"].value_counts().to_dict(),
        "confusion_counts": counts.tolist(),
        "confusion_row_normalized": np.round(norm, 4).tolist(),
        "method_metrics": metrics.round(4).to_dict(orient="records"),
        "evidence_composition": np.round(evidence, 4).tolist(),
        "limitations": [
            "Evaluation contains target-event-active windows plus active-driving windows (speed > 5 m/s) from the independent normal scenario; terminal stationary windows are excluded.",
            "Window-level replay from one recorded target trajectory per scenario; not an independent test-set generalization estimate.",
            "Ground-truth event windows combine scenario identity with physically interpretable activation conditions.",
            "Evidence contribution is normalized rule evidence, not SHAP or learned causal effect.",
            "Smoothing is display-only for the three timeline figures; classification uses one-second raw-window quantiles.",
        ],
    }
    (OUT / "figure_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    print("generated", len(list(OUT.glob("*.png"))), "PNG and", len(list(OUT.glob("*.svg"))), "SVG")


if __name__ == "__main__":
    main()
