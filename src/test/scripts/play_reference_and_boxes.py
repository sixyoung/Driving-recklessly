#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.font_manager as fm
import numpy as np
import re
import random

# ========== 字体设置 ==========
font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(font_path)
prop = fm.FontProperties(fname=font_path)
matplotlib.rcParams["font.family"] = prop.get_name()
matplotlib.rcParams["axes.unicode_minus"] = False
print(f"✅ 已加载中文字体: {prop.get_name()}")


# ========== 数据加载 ==========
def load_reference_line(filename):
    """读取参考线坐标 (x, y)，按帧组织"""
    frames = {}
    current_frame = None
    current_points = []

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("# Frame:"):
                if current_frame is not None and current_points:
                    frames[current_frame] = np.array(current_points)
                m = re.search(r"# Frame:\s*(\d+)", line)
                current_frame = int(m.group(1)) if m else len(frames)
                current_points = []
                continue

            if re.match(r"^[\d\.\-eE]+\s+[\d\.\-eE]+\s+[\d\.\-eE]+", line):
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        _, x, y = map(float, parts[:3])
                        current_points.append((x, y))
                    except ValueError:
                        continue

    if current_frame is not None and current_points:
        frames[current_frame] = np.array(current_points)

    print(f"✅ 参考线读取完成，共 {len(frames)} 帧，每帧约 {len(next(iter(frames.values())))} 个点")
    return frames


def load_obstacle_boxes(filename):
    """读取每帧的障碍物 Box"""
    frames = {}
    current_frame = None
    current_boxes = []
    current_box = []
    current_obs_id = None
    current_t = 0.0

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("# Frame ID:"):
                if current_frame is not None and current_boxes:
                    frames[current_frame] = current_boxes
                m = re.search(r"# Frame ID:\s*(\d+)", line)
                current_frame = int(m.group(1)) if m else len(frames)
                current_boxes = []
                continue

            if line.startswith("Obstacle "):
                m = re.match(r"Obstacle\s+(\S+)\s+t=([\d\.eE\-\+]+)", line)
                if m:
                    current_obs_id = m.group(1)
                    current_t = float(m.group(2))
                    current_box = []
                continue

            if line.startswith("----"):
                if current_box and current_obs_id:
                    current_boxes.append({
                        "id": current_obs_id,
                        "t": current_t,
                        "box": np.array(current_box)
                    })
                    current_box = []
                    current_obs_id = None
                continue

            if re.match(r"^[\-\d\.eE]+\s+[\-\d\.eE]+$", line):
                try:
                    x, y = map(float, line.split())
                    current_box.append((x, y))
                except ValueError:
                    continue

    if current_frame is not None and current_boxes:
        frames[current_frame] = current_boxes

    print(f"✅ 障碍物 Box 读取完成，共 {len(frames)} 帧")
    return frames


# ========== 播放器 ==========
def play_animation(frames_ref, frames_box, start=0, end=None, speed=1.0):
    all_frames = sorted(set(frames_ref.keys()) | set(frames_box.keys()))
    if end is None:
        end = len(all_frames)

    # ==== 计算全局坐标范围 ====
    all_x, all_y = [], []
    for ref in frames_ref.values():
        if len(ref) > 0:
            all_x.extend(ref[:, 0])
            all_y.extend(ref[:, 1])
    for frame in frames_box.values():
        for obs in frame:
            if len(obs["box"]) > 0:
                all_x.extend(obs["box"][:, 0])
                all_y.extend(obs["box"][:, 1])

    if all_x and all_y:
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        dx, dy = (x_max - x_min) * 0.1, (y_max - y_min) * 0.1
    else:
        x_min, x_max, y_min, y_max, dx, dy = -10, 10, -10, 10, 0, 0

    # 随机为每个障碍物分配固定颜色
    color_map = {}

    def get_color(obs_id):
        if obs_id not in color_map:
            color_map[obs_id] = (random.random(), random.random(), random.random())
        return color_map[obs_id]

    plt.rcParams["figure.dpi"] = 150
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("参考线与障碍物包围盒播放", fontsize=12)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim(x_min - dx, x_max + dx)
    ax.set_ylim(y_min - dy, y_max + dy)
    ax.set_aspect("equal", adjustable="box")

    paused = False
    frame_idx = start

    # ---- 键盘控制 ----
    def on_key(event):
        nonlocal paused, frame_idx, speed
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
            frame_idx = max(start, frame_idx - 1)
            update_plot(all_frames[frame_idx])
        elif event.key == "right" and paused:
            frame_idx = min(end - 1, frame_idx + 1)
            update_plot(all_frames[frame_idx])

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update_plot(fid):
        # ❌ 删除 ax.cla()（这会重置缩放）
        # ax.cla()

        # ✅ 改为清除旧图元（不破坏缩放）
        for line in ax.lines[:]:
            line.remove()
        for text in ax.texts[:]:
            text.remove()
        for patch in ax.patches[:]:
            patch.remove()

        # ✅ 保留原有标题、比例和坐标范围，不重设
        ax.set_title("参考线与障碍物包围盒播放", fontsize=12)

        # 绘制参考线
        ref = frames_ref.get(fid)
        if ref is not None and len(ref) > 0:
            ax.plot(ref[:, 0], ref[:, 1], "b-", lw=2, label="参考线")

        # 绘制障碍物
        boxes = frames_box.get(fid, [])
        if boxes:
            max_t = max(obs["t"] for obs in boxes)
            for obs in boxes:
                box = obs["box"]
                if len(box) >= 4:
                    xs, ys = box[:, 0], box[:, 1]
                    color = get_color(obs["id"])
                    alpha = max(0.1, 1.0 - obs["t"] / (max_t + 1e-6))
                    ax.plot(np.append(xs, xs[0]), np.append(ys, ys[0]),
                            color=color, alpha=alpha, lw=1.2)
                    cx, cy = np.mean(xs), np.mean(ys)
                    ax.text(cx, cy, f"{obs['id']}\n{obs['t']:.1f}",
                            fontsize=6, color=color, ha="center", va="center",
                            alpha=alpha, zorder=5)

        handles, labels = ax.get_legend_handles_labels()
        if labels:
            ax.legend(loc="upper right")

        ax.text(0.02, 0.98, f"Frame {fid} | Box数量={len(boxes)} | 速度={speed:.2f}x | 空格暂停",
                transform=ax.transAxes, va="top", ha="left",
                bbox=dict(facecolor="white", alpha=0.8, edgecolor="black"))

        fig.canvas.draw_idle()

    # ---- 播放循环 ----
    while frame_idx < end:
        update_plot(all_frames[frame_idx])
        if not paused:
            frame_idx += 1
        plt.pause(0.25 / speed)

    plt.show()


# ========== 主程序 ==========
if __name__ == "__main__":
    ref_file = "/home/bob/文档/备份/demo05/src/test/txt/reference_line_log/reference_line_log_0.txt"
    box_file = "/home/bob/文档/备份/demo05/src/test/txt/obstacle_box/obstacle_box_0.txt"

    frames_ref = load_reference_line(ref_file)
    frames_box = load_obstacle_boxes(box_file)

    start = int(input("请输入起始帧 (默认0): ") or 0)
    end = input(f"请输入结束帧 (最大{len(frames_ref)}): ")
    end = int(end) if end else len(frames_ref)
    speed = float(input("请输入播放速度 (默认1.0): ") or 1.0)

    play_animation(frames_ref, frames_box, start=start, end=end, speed=speed)
