import matplotlib.pyplot as plt
import matplotlib
import matplotlib.font_manager as fm

# 指定字体文件路径
font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

# 注册并设置为默认字体
fm.fontManager.addfont(font_path)
prop = fm.FontProperties(fname=font_path)
matplotlib.rcParams["font.family"] = prop.get_name()
matplotlib.rcParams["axes.unicode_minus"] = False  # 允许负号正常显示

print(f"✅ 已加载中文字体: {prop.get_name()}")

import time
import numpy as np
import re

# ================== 各加载函数 ==================
def load_path_waypoints_sample(filename):
    """读取采样点 (s, l)，支持 '# Frame ID ID:' 格式"""
    frames = {}
    current_points = []
    current_id = None

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("="):
                continue

            # ✅ 支持 '# Frame ID ID:' 格式
            if line.startswith("# Frame ID ID:"):
                if current_id is not None and current_points:
                    frames[current_id] = current_points
                m = re.search(r"# Frame ID ID:\s*(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                current_points = []
                continue

            # 跳过分隔线
            if line.startswith("----"):
                continue

            # ✅ 你的格式是: idx sub_idx s l x y
            parts = line.split()
            if len(parts) >= 4:
                try:
                    s = float(parts[2])  # 第3列: s
                    l = float(parts[3])  # 第4列: l
                    current_points.append((s, l))
                except ValueError:
                    continue

    # ✅ 最后一帧也要保存
    if current_id is not None and current_points:
        frames[current_id] = current_points

    # ✅ 防止StopIteration崩溃
    first_len = len(next(iter(frames.values()))) if frames else 0
    print(f"✅ 采样路径读取完成，共 {len(frames)} 帧，每帧约 {first_len} 个采样点")

    return frames

def load_dp_path(filename):
    """读取 DP Path 中的 Frenet Path (s, d)"""
    frames = {}
    current_frenet = []
    current_id = None
    mode = None
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# Frame ID:"):
                if current_id is not None and current_frenet:
                    frames[current_id] = current_frenet
                m = re.search(r"# Frame ID:\s*(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                current_frenet = []
                mode = None
                continue
            if line.startswith("# Frenet Path"):
                mode = "frenet"
                continue
            if mode == "frenet":
                parts = line.split()
                if len(parts) >= 2:
                    try:
                        s = float(parts[0])
                        d = float(parts[1])
                        current_frenet.append((s, d))
                    except ValueError:
                        continue
    if current_id is not None and current_frenet:
        frames[current_id] = current_frenet
    print(f"✅ DP 结果读取完成，共 {len(frames)} 帧，每帧约 {len(next(iter(frames.values())))} 个点")
    return frames


def load_sl_obstacles(filename):
    """读取每帧静态障碍物的 s-l 范围"""
    frames = {}
    current_obs = []
    current_id = None
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("==================== Frame"):
                if current_id is not None and current_obs:
                    frames[current_id] = current_obs
                m = re.search(r"Frame\s+(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                current_obs = []
                continue
            m = re.search(r"s:\[([\d\.\-eE]+), ([\d\.\-eE]+)\]\s+l:\[([\d\.\-eE]+), ([\d\.\-eE]+)\]", line)
            if m:
                s_min, s_max, l_max, l_min = map(float, m.groups())
                l_low, l_high = min(l_min, l_max), max(l_min, l_max)
                current_obs.append((s_min, s_max, l_low, l_high))
    if current_id is not None and current_obs:
        frames[current_id] = current_obs
    print(f"✅ 障碍物读取完成，共 {len(frames)} 帧")
    return frames

def load_lateral_bounds(filename):
    """读取每帧的横向约束范围 (s, low, high)"""
    frames = {}
    current_id = None
    current_bounds = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# Frame ID ID:"):
                # 保存上一帧
                if current_id is not None and current_bounds:
                    frames[current_id] = current_bounds
                current_bounds = []
                m = re.search(r"# Frame ID ID:\s*(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                continue
            if re.match(r"^[\d\.\-eE]+\s+[\d\.\-eE]+\s+[\d\.\-eE]+$", line):
                try:
                    s, low, high = map(float, line.split())
                    current_bounds.append((s, low, high))
                except ValueError:
                    continue
    if current_id is not None and current_bounds:
        frames[current_id] = current_bounds
    print(f"✅ 横向约束读取完成，共 {len(frames)} 帧")
    return frames

def load_dp_node_cost(filename):
    """读取 DP 节点代价 (s, l, safety_cost, smoothness_cost)"""
    frames = {}
    current_nodes = []
    current_id = None
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("==================== Frame"):
                if current_id is not None and current_nodes:
                    frames[current_id] = current_nodes
                m = re.search(r"Frame\s+(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                current_nodes = []
                continue
            if re.match(r"^# Level", line) or line.startswith("s\tl\t") or line.startswith("--------------------------------------"):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    s = float(parts[0])
                    l = float(parts[1])
                    safety = float(parts[2])
                    smooth = float(parts[3])
                    current_nodes.append((s, l, safety, smooth))
                except ValueError:
                    continue
    if current_id is not None and current_nodes:
        frames[current_id] = current_nodes
    print(f"✅ DP 节点代价读取完成，共 {len(frames)} 帧，每帧约 {len(next(iter(frames.values())))} 个节点")
    return frames


def load_qp_path(filename):
    """读取 QP 优化结果的 Frenet Path (s, d)"""
    frames = {}
    current_points = []
    current_id = None
    mode = None
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("="):
                continue
            if line.startswith("# Frame ID:"):
                if current_id is not None and current_points:
                    frames[current_id] = current_points
                m = re.search(r"# Frame ID:\s*(\d+)", line)
                current_id = int(m.group(1)) if m else len(frames)
                current_points = []
                mode = None
                continue
            if line.startswith("# Frenet Path"):
                mode = "frenet"
                continue
            elif line.startswith("# Discretized Path"):
                mode = "discretized"
                continue
            if mode == "frenet":
                parts = line.split()
                if len(parts) >= 2:
                    try:
                        s = float(parts[0])
                        d = float(parts[1])
                        current_points.append((s, d))
                    except ValueError:
                        continue
    if current_id is not None and current_points:
        frames[current_id] = current_points
    print(f"✅ QP 结果读取完成，共 {len(frames)} 帧，每帧约 {len(next(iter(frames.values())))} 个点")
    return frames


# ================== 播放函数 ==================
def play_sl_animation(frames_samp, frames_dp=None, frames_obs=None, frames_cost=None, frames_qp=None,
                      frames_bound=None,  # ✅ 新增：横向约束帧
                      start=0, end=None, speed=1.0, aspect_ratio=0.1):

    # ✅ 按帧号取并集
    all_ids = set(frames_samp.keys())
    if frames_dp: all_ids |= set(frames_dp.keys())
    if frames_obs: all_ids |= set(frames_obs.keys())
    if frames_cost: all_ids |= set(frames_cost.keys())
    if frames_qp: all_ids |= set(frames_qp.keys())
    valid_ids = sorted(all_ids)

    if end is None:
        end = len(valid_ids)
    print(f"✅ 有效帧号: {valid_ids[:10]}{'...' if len(valid_ids)>10 else ''} (共 {len(valid_ids)} 帧)")

    plt.rcParams["figure.dpi"] = 150
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("s (纵向距离)", fontsize=10)
    ax.set_ylabel("l / d (横向偏移)", fontsize=10)
    ax.set_title(f"SL 采样点 + DP + QP + 障碍物（aspect={aspect_ratio}）", fontsize=12, fontweight="bold")

    scatter_samp = ax.scatter([], [], s=12, c="royalblue", alpha=0.8, label="采样点 (s, l)")
    dp_line, = ax.plot([], [], "g-", linewidth=1.3, alpha=0.8, label="DP 优化路径 (s, d)")
    dp_scatter = ax.scatter([], [], s=18, c="limegreen", edgecolors="black", linewidths=0.4, alpha=0.9, label="DP 点")
    qp_line, = ax.plot([], [], color="orange", linewidth=1.8, alpha=0.8, label="QP 优化路径")
    qp_scatter = ax.scatter([], [], s=18, c="orange", edgecolors="black", linewidths=0.4, alpha=0.9, label="QP 点")
    obs_rects = []
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
                   bbox=dict(facecolor="white", alpha=0.8, edgecolor="black", boxstyle="round"))
    ax.legend(loc="upper right", fontsize=8, framealpha=0.8)

    all_s, all_l = [], []
    for f in frames_samp.values():
        all_s.extend([p[0] for p in f])
        all_l.extend([p[1] for p in f])
    for dic in [frames_dp, frames_qp, frames_obs]:
        if dic:
            for f in dic.values():
                if isinstance(f[0], tuple):
                    if len(f[0]) == 2:
                        all_s.extend([p[0] for p in f])
                        all_l.extend([p[1] for p in f])
                    elif len(f[0]) == 4:
                        for (s_min, s_max, l_min, l_max) in f:
                            all_s.extend([s_min, s_max])
                            all_l.extend([l_min, l_max])

    if all_s and all_l:
        s_min, s_max = min(all_s), max(all_s)
        l_min, l_max = min(all_l), max(all_l)
        s_margin = (s_max - s_min) * 0.1 + 0.5
        l_margin = (l_max - l_min) * 0.5 + 0.2
        ax.set_xlim(s_min - s_margin, s_max + s_margin)
        ax.set_ylim(l_min - l_margin, l_max + l_margin)
        ax.set_aspect(aspect_ratio)

    # === 控制变量 ===
    frame_cursor = start
    paused = False

    # === 键盘控制 ===
    def on_key(event):
        nonlocal paused, frame_cursor, speed
        if event.key == " ":
            paused = not paused
            print("⏸ 暂停" if paused else "▶ 继续")
        elif event.key == "escape":
            plt.close()
        elif event.key == "up":
            speed *= 1.5
            print(f"⚡ 播放速度: {speed:.2f}x")
        elif event.key == "down":
            speed /= 1.5
            print(f"🐢 播放速度: {speed:.2f}x")
        elif event.key == "left" and paused:
            frame_cursor = max(0, frame_cursor - 1)
            update_plot(valid_ids[frame_cursor])
        elif event.key == "right" and paused:
            frame_cursor = min(len(valid_ids) - 1, frame_cursor + 1)
            update_plot(valid_ids[frame_cursor])

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update_plot(frame_id):
        # === 清除旧帧内容 ===
        # 1️⃣ 清除障碍物矩形
        for p in list(ax.patches):
            p.remove()

        # 2️⃣ 清除 cost 散点（但保留采样点、DP点、QP点）
        for coll in list(ax.collections):
            if coll not in [scatter_samp, dp_scatter, qp_scatter]:
                coll.remove()

        # 3️⃣ 清除旧文字（但保留左上角 info）
        for t in list(ax.texts):
            if t is not info:
                t.remove()
        frame_samp = frames_samp.get(frame_id, [])
        scatter_samp.set_offsets(np.c_[[p[0] for p in frame_samp], [p[1] for p in frame_samp]])
        frame_dp = frames_dp.get(frame_id, []) if frames_dp else []
        if frame_dp:
            s_dp, d_dp = np.array(frame_dp).T
            dp_line.set_data(s_dp, d_dp)
            dp_scatter.set_offsets(np.c_[s_dp, d_dp])
        else:
            dp_line.set_data([], [])
            dp_scatter.set_offsets(np.empty((0, 2)))
        frame_qp = frames_qp.get(frame_id, []) if frames_qp else []
        if frame_qp:
            s_qp, l_qp = np.array(frame_qp).T
            qp_line.set_data(s_qp, l_qp)
            qp_scatter.set_offsets(np.c_[s_qp, l_qp])
        else:
            qp_line.set_data([], [])
            qp_scatter.set_offsets(np.empty((0, 2)))
        if frames_obs and frame_id in frames_obs:
            for (s_min, s_max, l_min, l_max) in frames_obs[frame_id]:
                rect = plt.Rectangle((s_min, l_min), s_max - s_min, l_max - l_min,
                                     color="red", alpha=0.3, lw=0.8, zorder=1)
                ax.add_patch(rect)
        # === 绘制横向约束浅绿色区域 ===
        if frames_bound and frame_id in frames_bound:
            bounds = np.array(frames_bound[frame_id])
            s_vals = bounds[:, 0]
            low_vals = bounds[:, 1]
            high_vals = bounds[:, 2]
            ax.fill_between(
                s_vals, low_vals, high_vals,
                color="lightgreen", alpha=0.3, zorder=0, label="横向约束"
            )

        # === 绘制 cost 节点 ===
        if frames_cost and frame_id in frames_cost:
            cost_nodes = np.array(frames_cost[frame_id])
            if len(cost_nodes) > 0:
                s_vals = cost_nodes[:, 0]
                l_vals = cost_nodes[:, 1]
                smooth_cost = cost_nodes[:, 3]

                # 使用 smoothness_cost 上色
                scatter_cost = ax.scatter(
                    s_vals, l_vals,
                    c=smooth_cost,
                    cmap="plasma",
                    s=25,
                    alpha=0.8,
                    edgecolors="none",
                    zorder=5,
                )

                # 标注 safety_cost + smooth_cost
                for (s, l, safety, smooth) in cost_nodes:
                    ax.text(s, l + 0.05,
                            f"S:{safety:.1f}\nSm:{smooth:.1f}",
                            fontsize=6, color="blue",
                            alpha=0.7, ha='center', va='bottom')

        ax.set_title(f"Frame {frame_id}", fontsize=11)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    for frame_id in valid_ids[start:end]:
        has_samp = frame_id in frames_samp
        has_dp = frames_dp and frame_id in frames_dp
        has_qp = frames_qp and frame_id in frames_qp
        has_obs = frames_obs and frame_id in frames_obs
        has_cost = frames_cost and frame_id in frames_cost
        if not (has_samp or has_dp or has_qp or has_obs or has_cost):
            print(f"⚠️ 跳过空帧（帧号 {frame_id}）")
            continue
        update_plot(frame_id)
        t0 = time.time()
        # === 播放循环 ===
        while frame_cursor < len(valid_ids):
            frame_id = valid_ids[frame_cursor]

            # 判断空帧
            has_samp = frame_id in frames_samp
            has_dp = frames_dp and frame_id in frames_dp
            has_qp = frames_qp and frame_id in frames_qp
            has_obs = frames_obs and frame_id in frames_obs
            has_cost = frames_cost and frame_id in frames_cost
            if not (has_samp or has_dp or has_qp or has_obs or has_cost):
                print(f"⚠️ 跳过空帧（帧号 {frame_id}）")
                frame_cursor += 1
                continue

            update_plot(frame_id)
            t0 = time.time()
            while paused:
                plt.pause(0.1)
            frame_cursor += 1
            elapsed = time.time() - t0
            plt.pause(max(0.02, 0.15 / speed - elapsed))

    print("✅ 播放结束")


# ================== 主程序入口 ==================
if __name__ == "__main__":
    samp_file = "/home/bob/文档/备份/demo05/src/test/txt/path_waypoints_samp/path_waypoints_samp_0.txt"
    dp_file = "/home/bob/文档/备份/demo05/src/test/txt/dp_path/dp_path_0.txt"
    obs_file = "/home/bob/文档/备份/demo05/src/test/txt/trajectory_cost_obstacles/trajectory_cost_obstacles_0.txt"
    cost_file = "/home/bob/文档/备份/demo05/src/test/txt/dp_node_cost/dp_node_cost_0.txt"
    qp_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_path/qp_path_0.txt"
    bound_file = "/home/bob/文档/备份/demo05/src/test/txt/lateral_bounds_log/lateral_bounds_log_0.txt"  # ✅ 新增
    frames_samp = load_path_waypoints_sample(samp_file)
    frames_dp = load_dp_path(dp_file)
    frames_obs = load_sl_obstacles(obs_file)
    frames_cost = load_dp_node_cost(cost_file)
    frames_qp = load_qp_path(qp_file)
    frames_bound = load_lateral_bounds(bound_file)  # ✅ 新增


    start = int(input("请输入开始帧 (默认0): ") or 0)
    end = input(f"请输入结束帧 (最大{len(frames_samp)}): ")
    end = int(end) if end else len(frames_samp)
    speed = float(input("请输入播放速度 (默认1.0): ") or 1.0)
    aspect = float(input("请输入视觉比例 aspect (默认1=横向放大1倍): ") or 1)

    play_sl_animation(frames_samp,
                    frames_dp=frames_dp,
                    frames_obs=frames_obs,
                    frames_cost=frames_cost,
                    frames_qp=frames_qp,
                    frames_bound=frames_bound,   # ✅ 一定要有这一行
                    start=start, end=end,
                    speed=speed, aspect_ratio=aspect)
