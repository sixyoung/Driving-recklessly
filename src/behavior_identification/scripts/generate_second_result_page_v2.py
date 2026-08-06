#!/usr/bin/env python3
"""Generate seven evidence-chain figures for the second result slide.

Data source: recorded CARLA/ROS runtime CSV files.  Figures are independent,
title-free PNG/SVG assets for PowerPoint.  Rule attribution and window replay
metrics are explicitly audited; they are not presented as independent test-set
generalisation results.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch, Rectangle
import numpy as np
import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
OUT = ROOT / "second_result_page_v2"
OUT.mkdir(parents=True, exist_ok=True)

TARGETS = {
    "normal": 32,
    "red_light_long": 144,
    "lane_deviation": 116,
    "aggressive_accel": 85,
    "aggressive_decel": 106,
    "overspeed": 60,
    "slow": 70,
    "traffic_conflict": 130,
}
BEHAVIORS = ["闯红灯", "横向通道偏离", "急加速/急减速", "超速行驶", "异常缓行", "复合异常行为"]
EVENT_COLUMNS = ["S–T事件", "S–L事件", "A–T事件", "场景条件", "持续时间"]

GRAY = "#C7CDD4"
DARK_GRAY = "#5D6772"
ORANGE = "#E69F00"
RED = "#D73027"
BLUE = "#2166AC"
GREEN = "#009E73"
PURPLE = "#CC79A7"
STATE_CMAP = ListedColormap([GRAY, ORANGE, RED])
STATE_NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], STATE_CMAP.N)

FONT_DIR = Path(__file__).resolve().parents[1] / "assets" / "fonts"
SONG_FONT_PATH = FONT_DIR / "simsun.ttc"
TIMES_REGULAR_PATH = FONT_DIR / "times.ttf"
TIMES_BOLD_PATH = FONT_DIR / "timesbd.ttf"
TIMES_ITALIC_PATH = FONT_DIR / "timesi.ttf"
TIMES_BOLD_ITALIC_PATH = FONT_DIR / "timesbi.ttf"
for font_path in (SONG_FONT_PATH, TIMES_REGULAR_PATH, TIMES_BOLD_PATH,
                  TIMES_ITALIC_PATH, TIMES_BOLD_ITALIC_PATH):
    if not font_path.exists():
        raise FileNotFoundError(f"Required publication font is missing: {font_path}")
    font_manager.fontManager.addfont(str(font_path))
SONG_FONT = font_manager.FontProperties(fname=str(SONG_FONT_PATH)).get_name()
TIMES_FONT = font_manager.FontProperties(fname=str(TIMES_REGULAR_PATH)).get_name()
mpl.rcParams.update({
    # Times New Roman is tried first for Latin letters, numbers and symbols;
    # SimSun supplies all CJK glyphs absent from Times New Roman.
    "font.family": [TIMES_FONT, SONG_FONT],
    "font.serif": [TIMES_FONT, SONG_FONT],
    "mathtext.fontset": "custom",
    "mathtext.rm": TIMES_FONT,
    "mathtext.it": f"{TIMES_FONT}:italic",
    "mathtext.bf": f"{TIMES_FONT}:bold",
    "axes.unicode_minus": False,
    "font.size": 11,
    "axes.labelsize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "axes.linewidth": 1.0,
    "savefig.bbox": "tight",
    "savefig.facecolor": "white",
    "pdf.fonttype": 42,
    "svg.fonttype": "none",
})


def load_raw(name: str) -> pd.DataFrame:
    return pd.read_csv(ROOT / f"{name}.csv")


def load_target(name: str) -> pd.DataFrame:
    raw = load_raw(name)
    frame = raw.loc[raw["vehicle_id"].eq(TARGETS[name])].sort_values("ros_time").copy()
    if frame.empty:
        raise RuntimeError(f"No target rows for {name}")
    frame["t"] = frame["ros_time"] - frame["ros_time"].iloc[0]
    frame["ST"] = np.clip(frame["st_constraint"] / 0.45, 0, 3)
    frame["SL"] = np.clip(np.abs(frame["lateral_deviation_m"]) / 0.68, 0, 3)
    frame["ACC"] = np.clip(np.abs(frame["accel_mps2"]) / 3.0, 0, 3)
    frame["ACC_POS"] = np.clip(frame["accel_mps2"] / 3.0, 0, 3)
    frame["OVER"] = np.clip(frame["speed_mps"] / 15.0, 0, 3)
    frame["SLOW"] = np.clip((7.0 - frame["speed_mps"]) / 3.5, 0, 3)
    frame["AT"] = np.maximum.reduce([frame["ACC"], frame["OVER"], frame["SLOW"]])
    frame["RED_SIGNAL"] = frame["nearest_stop_state"].astype(str).str.lower().eq("red").astype(float)
    return frame.reset_index(drop=True)


def smooth(values, window=5, alpha=0.38):
    s = pd.Series(np.asarray(values, dtype=float))
    return s.rolling(window, center=True, min_periods=1).median().ewm(alpha=alpha, adjust=False).mean().to_numpy()


def bridge_short_gaps(mask, max_gap=1):
    out = np.asarray(mask, dtype=bool).copy()
    n = len(out)
    i = 0
    while i < n:
        if out[i]:
            i += 1
            continue
        j = i
        while j < n and not out[j]:
            j += 1
        if i > 0 and j < n and (j - i) <= max_gap:
            out[i:j] = True
        i = j
    return out


def event_states(values, time, threshold=1.0, persistence_s=0.75):
    values = np.asarray(values, dtype=float)
    time = np.asarray(time, dtype=float)
    active = bridge_short_gaps(values >= threshold, max_gap=1)
    state = np.zeros(len(active), dtype=int)
    i = 0
    while i < len(active):
        if not active[i]:
            i += 1
            continue
        j = i + 1
        while j < len(active) and active[j]:
            j += 1
        state[i:j] = 1
        sustained_start = np.flatnonzero(time[i:j] - time[i] >= persistence_s)
        if len(sustained_start):
            state[i + int(sustained_start[0]):j] = 2
        i = j
    return state


def save(fig, stem):
    fig.savefig(OUT / f"{stem}.png", dpi=300)
    fig.savefig(OUT / f"{stem}.svg")
    plt.close(fig)


def style_axis(ax, axis="both"):
    ax.grid(True, axis=axis, color="#D9E2EC", linewidth=0.7, alpha=0.72)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def resample_event(frame, channel, start, duration=8.0, n=220):
    part = frame.loc[frame["t"].between(start, start + duration)]
    rel = part["t"].to_numpy() - start
    grid = np.linspace(0, duration, n)
    values = np.interp(grid, rel, smooth(part[channel], 5, 0.42))
    return grid, event_states(values, grid)


def figure_1_discrete_event_encoding(data):
    configs = [
        ("red_light_long", "ST", 10.5, "S–T偏离事件"),
        ("lane_deviation", "SL", 4.5, "S–L偏离事件"),
        ("aggressive_decel", "AT", 4.5, "A–T偏离事件"),
    ]
    signals = []
    for scenario, channel, start, _ in configs:
        _, state = resample_event(data[scenario], channel, start)
        signals.append(state)
    matrix = np.vstack(signals)
    fig, ax = plt.subplots(figsize=(8.0, 3.2), constrained_layout=True)
    ax.imshow(matrix, aspect="auto", interpolation="nearest", cmap=STATE_CMAP, norm=STATE_NORM,
              extent=[0, 8, 2.5, -0.5])
    ax.set_yticks(range(3), [x[3] for x in configs])
    ax.set_xlabel("事件相对时间 τ（s）")
    ax.set_xticks(np.arange(0, 9, 1))
    ax.tick_params(axis="y", length=0)
    ax.legend(handles=[Patch(color=GRAY, label="无偏离"), Patch(color=ORANGE, label="瞬时偏离"), Patch(color=RED, label="持续偏离")],
              frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.01))
    for y in [0.5, 1.5]:
        ax.axhline(y, color="white", linewidth=2)
    save(fig, "01_continuous_to_discrete_event_sequence")
    pd.DataFrame(matrix.T, columns=[x[3] for x in configs]).assign(relative_time_s=np.linspace(0, 8, matrix.shape[1])).to_csv(
        OUT / "01_discrete_event_sequence.csv", index=False, encoding="utf-8-sig")


def frontal_interaction(raw, target):
    dx = np.gradient(target["x"].to_numpy(float))
    dy = np.gradient(target["y"].to_numpy(float))
    direction = np.array([np.nanmedian(dx), np.nanmedian(dy)])
    direction = direction / (np.linalg.norm(direction) + 1e-9)
    result = []
    for _, row in target.iterrows():
        others = raw.loc[raw["ros_time"].eq(row["ros_time"]) & raw["vehicle_id"].ne(row["vehicle_id"])]
        if others.empty:
            result.append(np.nan)
            continue
        rel = others[["x", "y"]].to_numpy(float) - np.array([row["x"], row["y"]])
        distance = np.linalg.norm(rel, axis=1)
        projection = rel @ direction
        lateral = np.abs(rel[:, 0] * (-direction[1]) + rel[:, 1] * direction[0])
        mask = (projection > 0) & (lateral < 15.0)
        result.append(float(distance[mask].min()) if mask.any() else np.nan)
    return np.asarray(result)


def first_active_time(time, state):
    idx = np.flatnonzero(np.asarray(state) > 0)
    return None if not len(idx) else float(np.asarray(time)[idx[0]])


def active_intervals(time, state, level=1):
    mask = np.asarray(state) >= level
    time = np.asarray(time)
    intervals = []
    i = 0
    while i < len(mask):
        if not mask[i]:
            i += 1; continue
        j = i + 1
        while j < len(mask) and mask[j]:
            j += 1
        intervals.append((float(time[i]), float(time[j-1])))
        i = j
    return intervals


def figure_2_multisource_timeline(data):
    name = "red_light_long"
    raw = load_raw(name)
    frame = data[name].copy()
    frame["front_distance"] = frontal_interaction(raw, frame)
    part = frame.loc[frame["t"].between(8.0, 21.0)].copy().reset_index(drop=True)
    time = part["t"].to_numpy()

    st = event_states(smooth(part["ST"], 5), time)
    sl = event_states(smooth(part["SL"], 5), time)
    at = event_states(smooth(part["AT"], 5), time)
    signal = np.zeros(len(part), dtype=int)
    states = part["nearest_stop_state"].astype(str).str.lower()
    signal[states.eq("yellow").to_numpy()] = 1
    signal[states.eq("red").to_numpy()] = 2
    stop_score = np.where(part["nearest_stop_distance_m"].notna(), 10.0 / np.maximum(part["nearest_stop_distance_m"].to_numpy(float), 0.2), 0)
    stop = event_states(stop_score, time, threshold=1.0, persistence_s=0.45)
    interaction_score = 22.0 / np.maximum(part["front_distance"].fillna(1e6).to_numpy(float), 0.2)
    interaction = event_states(interaction_score, time, threshold=1.0, persistence_s=0.55)
    motion = np.zeros(len(part), dtype=int)
    transition = (np.abs(part["accel_mps2"].to_numpy(float)) >= 0.40) | (part["speed_mps"].to_numpy(float) < 3.0)
    motion[transition] = 1
    motion[part["speed_mps"].to_numpy(float) < 0.8] = 2

    labels = ["S–T偏离事件", "S–L偏离事件", "A–T偏离事件", "信号灯状态", "停止线事件", "前车交互事件", "启停状态"]
    matrix = np.vstack([st, sl, at, signal, stop, interaction, motion])
    fig, (ax_note, ax) = plt.subplots(
        2, 1, figsize=(9.0, 6.1), sharex=True, constrained_layout=True,
        gridspec_kw={"height_ratios": [2.6, 7.0]},
    )
    ax.imshow(matrix, aspect="auto", interpolation="nearest", cmap=STATE_CMAP, norm=STATE_NORM,
              extent=[time.min(), time.max(), 6.5, -0.5])
    ax.set_yticks(range(7), labels)
    ax.set_xlabel("统一时间轴 t（s）")
    ax.tick_params(axis="y", length=0)
    for y in np.arange(0.5, 6.5, 1):
        ax.axhline(y, color="white", linewidth=1.8)

    st_first = first_active_time(time, st)
    signal_first = first_active_time(time, signal > 0)
    st_sustained = active_intervals(time, st, level=2)
    st_active = active_intervals(time, st, level=1)
    overlap = (st > 0) & (signal == 2) & (stop > 0) & (interaction > 0)
    idx = np.flatnonzero(overlap)

    # Five independent annotation tracks prevent labels from competing for the
    # same horizontal space. Event names are y-axis labels; only marks occupy time.
    annotation_labels = ["时序先后", "首次触发", "同步关联", "持续区间", "异常终止"]
    annotation_y = np.arange(5)[::-1]
    ax_note.set_ylim(-0.65, 4.65)
    ax_note.set_yticks(annotation_y, annotation_labels)
    ax_note.tick_params(axis="y", length=0, pad=8, labelsize=9)
    ax_note.tick_params(axis="x", which="both", bottom=False, labelbottom=False)
    for spine in ax_note.spines.values():
        spine.set_visible(False)
    for y in annotation_y:
        ax_note.hlines(y, time.min(), time.max(), color="#E8EDF2", linewidth=1.0, zorder=0)

    handles = [Patch(color=GRAY, label="未激活/正常"),
               Patch(color=ORANGE, label="瞬时/过渡"),
               Patch(color=RED, label="持续/异常")]
    ax_note.legend(handles=handles, frameon=False, ncol=3, loc="lower center",
                   bbox_to_anchor=(0.5, 1.01), borderaxespad=0.0,
                   handlelength=1.6, columnspacing=2.0)

    if signal_first is not None and st_first is not None:
        ax_note.annotate("", xy=(st_first, 4), xytext=(signal_first, 4),
                         arrowprops=dict(arrowstyle="->", color=DARK_GRAY,
                                         linewidth=1.5, shrinkA=0, shrinkB=0))
        ax_note.scatter([signal_first, st_first], [4, 4], s=24,
                        color=[DARK_GRAY, BLUE], zorder=4)
    if st_first is not None:
        ax.axvline(st_first, color=BLUE, linestyle="--", linewidth=1.2)
        ax_note.scatter(st_first, 3, marker="v", s=70, color=BLUE, zorder=4)
    if len(idx):
        sync_a, sync_b = time[idx[0]], time[idx[-1]]
        ax.axvspan(sync_a, sync_b, color=BLUE, alpha=0.10)
        ax_note.hlines(2, sync_a, sync_b, color=BLUE, linewidth=6, alpha=.72)
        ax_note.vlines([sync_a, sync_b], 1.82, 2.18, color=BLUE, linewidth=1.2)
    if st_sustained:
        sustained_a, sustained_b = max(st_sustained, key=lambda z: z[1] - z[0])
        ax_note.hlines(1, sustained_a, sustained_b, color=RED, linewidth=6)
        ax_note.vlines([sustained_a, sustained_b], .82, 1.18, color=RED, linewidth=1.2)
    if st_active:
        end_t = st_active[-1][1]
        ax.axvline(end_t, color=DARK_GRAY, linestyle=":", linewidth=1.2)
        ax_note.scatter(end_t, 0, marker="s", s=58, color=DARK_GRAY, zorder=4)

    save(fig, "02_multisource_event_alignment_timeline")
    pd.DataFrame(matrix.T, columns=labels).assign(time_s=time, front_distance_m=part["front_distance"]).to_csv(
        OUT / "02_multisource_event_alignment.csv", index=False, encoding="utf-8-sig")


def q90_features(frame):
    f = frame.loc[frame["t"].ge(3.0)]
    return np.array([f["ST"].quantile(.9), f["SL"].quantile(.9), f["AT"].quantile(.9),
                     (f["ST"]*f["RED_SIGNAL"]).quantile(.9), f["SL"].quantile(.9),
                     f["ACC"].quantile(.9), f["OVER"].quantile(.9), f["SLOW"].quantile(.9)])


def figure_3_attribution_heatmap(data):
    condition_names = ["交通控制条件", "道路几何条件", "交通交互条件", "车辆动力学条件"]
    control = q90_features(data["red_light_long"])
    geometry = q90_features(data["lane_deviation"])
    interaction = q90_features(data["traffic_conflict"])
    dynamics = np.mean(np.vstack([q90_features(data[x]) for x in ["aggressive_accel", "aggressive_decel", "overspeed", "slow"]]), axis=0)
    raw = np.vstack([control, geometry, interaction, dynamics])
    gates = np.array([
        [1.00, .10, .25, 1.00, .05, .10, .15, .10],
        [.15, 1.00, .25, .10, 1.00, .10, .10, .10],
        [.55, .25, .45, .35, .20, .30, .25, .20],
        [.20, .10, 1.00, .10, .05, 1.00, 1.00, 1.00],
    ])
    attribution = raw * gates
    attribution = attribution / np.maximum(attribution.max(axis=0, keepdims=True), 1e-9)
    columns = ["S–T偏离", "S–L偏离", "A–T偏离", "闯红灯", "横向偏离", "急加/急减速", "超速", "异常缓行"]
    fig, ax = plt.subplots(figsize=(9.2, 4.3), constrained_layout=True)
    image = ax.imshow(attribution, cmap="Reds", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(np.arange(len(columns)), columns, rotation=25, ha="right")
    ax.set_yticks(np.arange(len(condition_names)), condition_names)
    for i in range(attribution.shape[0]):
        for j in range(attribution.shape[1]):
            value = attribution[i, j]
            ax.text(j, i, f"{value:.2f}", ha="center", va="center", fontsize=9,
                    color="white" if value >= .58 else "#202020")
    cbar = fig.colorbar(image, ax=ax, fraction=.035, pad=.025)
    cbar.set_label("归因强度")
    ax.tick_params(length=0)
    save(fig, "03_scene_condition_attribution_heatmap")
    pd.DataFrame(attribution, index=condition_names, columns=columns).to_csv(
        OUT / "03_scene_condition_attribution.csv", encoding="utf-8-sig")
    return attribution


PATTERN = np.array([
    [1.0, 0.0, 0.0, 1.0, .8],
    [0.0, 1.0, 0.0, .8, .7],
    [0.0, 0.0, 1.0, .6, .4],
    [0.0, 0.0, 1.0, .6, .8],
    [0.0, 0.0, 1.0, .7, .8],
    [1.0, 1.0, 1.0, 1.0, 1.0],
])


def current_case_evidence(data):
    f = data["red_light_long"]
    active = f.loc[(f["RED_SIGNAL"] > .5) & (f["ST"] >= .8) & (f["speed_mps"] > 5)]
    duration = float(active["t"].max() - active["t"].min()) if len(active) else 0
    return np.array([
        float((active["ST"] >= 1.0).any()),
        float((active["SL"] >= 1.0).any()),
        float((active["AT"] >= 1.0).any()),
        float(active["RED_SIGNAL"].mean() >= .5),
        min(duration / 2.0, 1.0),
    ])


def pattern_match_scores(observed):
    scene_compatibility = np.array([1.0, .15, .08, .08, .08, .45])
    scores = []
    for i, req in enumerate(PATTERN):
        required_channels = req[:3] > 0
        channel_coverage = float(observed[:3][required_channels].mean()) if required_channels.any() else 0.0
        duration_coverage = min(float(observed[4] / max(req[4], 1e-9)), 1.0)
        score = .62 * channel_coverage + .23 * scene_compatibility[i] + .15 * duration_coverage
        scores.append(np.clip(score, 0, 1))
    return np.asarray(scores)


def figure_4_pattern_matching(data):
    observed = current_case_evidence(data)
    scores = pattern_match_scores(observed)
    winner = int(np.argmax(scores))
    fig, ax = plt.subplots(figsize=(8.8, 5.2), constrained_layout=True)
    ax.set_xlim(-.55, 6.25); ax.set_ylim(len(BEHAVIORS)-.55, -.55)
    ax.set_xticks(np.arange(5), EVENT_COLUMNS)
    ax.set_yticks(np.arange(6), BEHAVIORS)
    ax.xaxis.tick_top(); ax.tick_params(length=0)
    for x in np.arange(-.5, 5.5, 1): ax.axvline(x, color="#D9E2EC", linewidth=.8, zorder=0)
    for y in np.arange(-.5, 6.5, 1): ax.axhline(y, color="#D9E2EC", linewidth=.8, zorder=0)
    for i in range(PATTERN.shape[0]):
        for j in range(PATTERN.shape[1]):
            value = PATTERN[i, j]
            if value <= 0:
                ax.scatter(j, i, s=28, color=GRAY, zorder=2)
            else:
                color = RED if value >= .75 else ORANGE
                ax.scatter(j, i, s=90 + 220*value, color=color, edgecolor="white", linewidth=.8, zorder=3)
        ax.text(5.25, i, f"{scores[i]:.0%}", va="center", ha="center", color=DARK_GRAY)
    ax.text(5.25, -.72, "匹配度", ha="center", va="bottom", fontweight="bold")
    ax.legend(handles=[mpl.lines.Line2D([], [], marker='o', linestyle='', color=RED, markersize=9, label='主导条件'),
                       mpl.lines.Line2D([], [], marker='o', linestyle='', color=ORANGE, markersize=7, label='辅助条件')],
              frameon=False, ncol=2, loc="lower center", bbox_to_anchor=(.48, -.18))
    for spine in ax.spines.values(): spine.set_visible(False)
    save(fig, "04_behavior_pattern_matching_matrix")
    pd.DataFrame(PATTERN, index=BEHAVIORS, columns=EVENT_COLUMNS).assign(match_score=scores).to_csv(
        OUT / "04_pattern_matching_scores.csv", encoding="utf-8-sig")
    return observed, scores, winner


def build_event_windows(data):
    scenario_names = ["red_light_long", "lane_deviation", "aggressive_accel", "aggressive_decel", "overspeed", "slow"]
    rows = []
    for name in scenario_names:
        d = data[name].loc[data[name]["t"].ge(3)].copy()
        d["window"] = np.floor((d["t"] - 3) / .75).astype(int)
        for window, g in d.groupby("window"):
            if len(g) < 3: continue
            r = {
                "scenario": name, "window": int(window), "time": float(g["t"].mean()),
                "ST": float(g["ST"].quantile(.9)), "SL": float(g["SL"].quantile(.9)),
                "ACC": float(g["ACC"].quantile(.9)), "ACC_POS": float(g["ACC_POS"].quantile(.9)),
                "OVER": float(g["OVER"].quantile(.9)), "SLOW": float(g["SLOW"].quantile(.9)),
                "RED": float(g["RED_SIGNAL"].mean()), "SPEED": float(g["speed_mps"].median()),
            }
            truth = None
            if name == "red_light_long" and r["RED"] >= .5 and r["ST"] >= .8 and r["SPEED"] > 5: truth = "闯红灯"
            elif name == "lane_deviation" and r["SL"] >= 1 and r["ST"] >= 1: truth = "复合异常行为"
            elif name == "lane_deviation" and r["SL"] >= 1: truth = "横向通道偏离"
            elif name == "aggressive_decel" and r["ACC"] >= .8: truth = "急加速/急减速"
            elif name == "aggressive_accel" and r["ACC_POS"] >= .5 and 12 <= r["SPEED"] < 15: truth = "急加速/急减速"
            elif name == "overspeed" and r["OVER"] >= 1: truth = "超速行驶"
            elif name == "slow" and r["time"] >= 8 and r["SLOW"] >= .55 and r["RED"] < .5: truth = "异常缓行"
            if truth:
                r["truth"] = truth; rows.append(r)
    result = pd.DataFrame(rows)
    result["pred"] = result.apply(predict_behavior, axis=1)
    return result


def predict_behavior(r):
    scores = {
        "闯红灯": .58*min(r.ST/1.1,1)+.32*r.RED+.10*min(r.OVER,1),
        "横向通道偏离": .78*min(r.SL/1.2,1)+.12*(1-r.RED)+.10*min(r.OVER,1),
        "急加速/急减速": .75*min(r.ACC/1.0,1)+.15*min(r.SLOW,1)+.10*(1-r.RED),
        "超速行驶": .80*min(r.OVER/1.05,1)+.10*(1-r.RED)+.10*min(r.ACC,1),
        "异常缓行": .78*min(r.SLOW/1.0,1)+.12*(1-r.RED)+.10*(1-min(r.OVER,1)),
        "复合异常行为": .45*min(r.ST,1)+.45*min(r.SL,1)+.10*max(r.RED,min(r.OVER,1)),
    }
    return max(scores, key=scores.get)


def confusion(event_windows):
    matrix = np.zeros((len(BEHAVIORS), len(BEHAVIORS)), dtype=int)
    for _, r in event_windows.iterrows():
        matrix[BEHAVIORS.index(r["truth"]), BEHAVIORS.index(r["pred"])] += 1
    return matrix


def class_metrics(matrix):
    records = []
    for i, behavior in enumerate(BEHAVIORS):
        tp = matrix[i, i]; fp = matrix[:, i].sum()-tp; fn = matrix[i, :].sum()-tp
        p = tp/(tp+fp) if tp+fp else 0; r = tp/(tp+fn) if tp+fn else 0
        f1 = 2*p*r/(p+r) if p+r else 0
        records.append({"behavior": behavior, "Precision": p, "Recall": r, "F1-score": f1, "n": int(matrix[i,:].sum())})
    return pd.DataFrame(records)


def figure_5_classification_bars(event_windows, matrix):
    metrics = class_metrics(matrix)
    metrics.to_csv(OUT / "05_classification_metrics.csv", index=False, encoding="utf-8-sig")
    fig, ax = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    x = np.arange(len(metrics)); width=.23
    specs = [("Precision", -width, BLUE), ("Recall", 0, ORANGE), ("F1-score", width, GREEN)]
    for metric, offset, color in specs:
        bars = ax.bar(x+offset, metrics[metric], width, color=color, edgecolor="white", label=metric)
        for bar, value in zip(bars, metrics[metric]):
            ax.text(bar.get_x()+bar.get_width()/2, value+.025, f"{value:.2f}", ha="center", va="bottom", fontsize=8)
    labels = [f"{b}\n(n={n})" for b,n in zip(metrics.behavior, metrics.n)]
    ax.set_xticks(x, labels)
    ax.set_ylabel("分类性能")
    ax.set_ylim(0, 1.13)
    ax.legend(frameon=False, ncol=3, loc="upper center")
    style_axis(ax, axis="y")
    save(fig, "05_per_class_precision_recall_f1")
    return metrics


def figure_6_confusion_matrix(matrix):
    totals = matrix.sum(axis=1, keepdims=True)
    norm = np.divide(matrix, totals, out=np.zeros_like(matrix, dtype=float), where=totals != 0)
    fig, ax = plt.subplots(figsize=(7.2, 5.8), constrained_layout=True)
    image = ax.imshow(norm, cmap="Blues", vmin=0, vmax=1, aspect="equal")
    ax.set_xticks(np.arange(6), BEHAVIORS, rotation=28, ha="right")
    ax.set_yticks(np.arange(6), BEHAVIORS)
    ax.set(xlabel="预测类别", ylabel="真实类别")
    for i in range(6):
        for j in range(6):
            text = f"{norm[i,j]:.0%}\n({matrix[i,j]})" if matrix[i,j] else "–"
            ax.text(j, i, text, ha="center", va="center", fontsize=9,
                    color="white" if norm[i,j] >= .58 else "#202020")
    cbar = fig.colorbar(image, ax=ax, fraction=.045, pad=.035)
    cbar.set_label("按真实类别归一化比例")
    ax.tick_params(length=0)
    save(fig, "06_six_class_confusion_matrix")


def figure_7_channel_contribution(event_windows):
    rows = []
    for behavior in BEHAVIORS:
        g = event_windows.loc[event_windows["truth"].eq(behavior)]
        st = max(float(g["ST"].median())-.80, 0)
        sl = max(float(g["SL"].median())-.50, 0)
        at_raw = np.maximum.reduce([g["ACC"].to_numpy(), g["OVER"].to_numpy(), g["SLOW"].to_numpy()])
        at = max(float(np.median(at_raw))-.50, 0)
        values = np.array([st, sl, at])
        if values.sum() == 0: values = np.array([.01,.01,.01])
        values = values/values.sum()
        rows.append(values)
    matrix = np.vstack(rows)
    pd.DataFrame(matrix, index=BEHAVIORS, columns=["S–T主导", "S–L主导", "A–T主导"]).to_csv(
        OUT / "07_channel_contribution.csv", encoding="utf-8-sig")
    fig, ax = plt.subplots(figsize=(8.5, 5.0), constrained_layout=True)
    left = np.zeros(len(BEHAVIORS))
    colors = [RED, ORANGE, "#8A8F98"]
    for j, (label,color) in enumerate(zip(["S–T主导", "S–L主导", "A–T主导"], colors)):
        bars=ax.barh(BEHAVIORS, matrix[:,j], left=left, color=color, edgecolor="white", label=label)
        text_color = "white" if j == 0 else "#202020"
        for bar,value in zip(bars,matrix[:,j]):
            if value>=.10:
                ax.text(bar.get_x()+bar.get_width()/2, bar.get_y()+bar.get_height()/2, f"{value:.0%}",
                        ha="center",va="center",fontsize=8,color=text_color)
        left += matrix[:,j]
    ax.invert_yaxis(); ax.set_xlim(0,1); ax.set_xlabel("通道归一化贡献")
    ax.xaxis.set_major_formatter(mpl.ticker.PercentFormatter(1.0))
    ax.legend(frameon=False,ncol=3,loc="lower center",bbox_to_anchor=(.5,1.01))
    style_axis(ax, axis="x")
    save(fig, "07_behavior_channel_contribution_statistics")
    return matrix


def main():
    data = {name: load_target(name) for name in TARGETS}
    figure_1_discrete_event_encoding(data)
    figure_2_multisource_timeline(data)
    attribution = figure_3_attribution_heatmap(data)
    observed, match_scores, winner = figure_4_pattern_matching(data)
    windows = build_event_windows(data)
    windows.to_csv(OUT / "event_windows.csv", index=False, encoding="utf-8-sig")
    matrix = confusion(windows)
    metrics = figure_5_classification_bars(windows, matrix)
    figure_6_confusion_matrix(matrix)
    contributions = figure_7_channel_contribution(windows)
    audit = {
        "source": "recorded CARLA/ROS target-vehicle runtime CSV",
        "figures": sorted(p.name for p in OUT.glob("*.png")),
        "event_encoding": {"0":"无偏离", "1":"瞬时偏离", "2":"持续偏离", "persistence_threshold_s":.75},
        "timeline_case": "red_light_long target vehicle 144; frontal interaction derived from synchronous other-vehicle positions",
        "pattern_case": "red_light_long target vehicle 144",
        "pattern_observed_evidence": np.round(observed,4).tolist(),
        "pattern_match_scores": {k:round(float(v),4) for k,v in zip(BEHAVIORS,match_scores)},
        "pattern_winner": BEHAVIORS[winner],
        "event_window_counts": windows["truth"].value_counts().reindex(BEHAVIORS).fillna(0).astype(int).to_dict(),
        "confusion_counts": matrix.tolist(),
        "classification_metrics": metrics.round(4).to_dict(orient="records"),
        "channel_contribution": np.round(contributions,4).tolist(),
        "limitations": [
            "Classification is a controlled 0.75 s event-window replay from one recorded target trajectory per scenario, not an independent test-set generalisation estimate.",
            "Composite-abnormal windows are real lane-deviation-run windows where both S-L and S-T exceed their boundaries.",
            "Scene attribution is rule-normalized recorded response intensity, not a learned causal effect or SHAP value.",
            "Figure 1 aligns representative real channel segments by relative event time; it is not a single multi-behavior trajectory.",
        ],
    }
    (OUT/"figure_audit.json").write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(audit,ensure_ascii=False,indent=2))
    print("generated",len(list(OUT.glob('*.png'))),"PNG and",len(list(OUT.glob('*.svg'))),"SVG")


if __name__ == "__main__":
    main()
