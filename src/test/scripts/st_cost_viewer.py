import matplotlib.pyplot as plt
import numpy as np
import time
import os

# ==========================
# 读取 ST-Cost 文件（含速度）
# ==========================
def load_st_cost_file(filename):
    frames = []
    current_frame_id = None
    current_points = []

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # 读取 Frame ID
            if line.startswith("# Frame ID:"):
                if current_points:
                    frames.append({
                        "frame_id": current_frame_id,
                        "points": current_points
                    })
                    current_points = []
                try:
                    current_frame_id = int(line.split(":")[1].strip())
                except:
                    current_frame_id = len(frames)
                continue

            # 跳过注释行
            if line.startswith("#") or line.startswith("="):
                continue

            # t_idx s_idx t_val s_val cost v_opt
            parts = line.split()
            if len(parts) == 6:
                try:
                    t_idx = int(parts[0])
                    s_idx = int(parts[1])
                    t_val = float(parts[2])
                    s_val = float(parts[3])
                    cost = float(parts[4]) if parts[4] != "inf" else np.inf
                    v_opt = float(parts[5])
                    current_points.append((t_idx, s_idx, t_val, s_val, cost, v_opt))
                except:
                    continue

    if current_points:
        frames.append({
            "frame_id": current_frame_id,
            "points": current_points
        })

    return frames


# ==========================
#   播放并绘制 ST-Cost + Speed
# ==========================
def play_cost_frames(frames, start=0, end=None, speed=1.0):

    if end is None:
        end = len(frames)

    fig, ax = plt.subplots(figsize=(8, 6), dpi=120)
    paused = False
    frame_idx = start
    state = {"speed": speed}

    # -------------------------
    # 更新绘图函数
    # -------------------------
    def update_plot(idx):
        ax.clear()

        frame = frames[idx]
        points = frame["points"]

        t_vals = []
        s_vals = []

        for (t_idx, s_idx, t, s, cost, v_opt) in points:
            t_vals.append(t)
            s_vals.append(s)

            # 绘制点
            if np.isinf(cost):
                ax.scatter(t, s, c="red", marker="x", s=40)
                ax.text(t + 0.02, s + 0.02, "inf", fontsize=6, color="red")
            else:
                ax.scatter(t, s, c="blue", marker="o", s=25)
                ax.text(t + 0.02, s + 0.02, f"{cost:.1f}", fontsize=6, color="blue")

            # 绘制速度（绿色）
            ax.text(t + 0.02, s - 0.10, f"v={v_opt:.2f}", fontsize=6, color="green")

        # 自动坐标轴范围
        if t_vals:
            ax.set_xlim(min(t_vals) - 0.5, max(t_vals) + 0.5)
        if s_vals:
            ax.set_ylim(min(s_vals) - 0.5, max(s_vals) + 0.5)

        ax.set_xlabel("t (seconds)")
        ax.set_ylabel("s (meters)")
        ax.set_title(f"ST-Cost + Speed  (Frame {idx+1}/{end}, Frame ID {frame['frame_id']})")

        ax.grid(True)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # -------------------------
    # 按键事件
    # -------------------------
    def on_key(event):
        nonlocal paused, frame_idx

        if event.key == " ":
            paused = not paused
            print("▶ 继续" if not paused else "⏸ 暂停")

        elif event.key == "escape":
            plt.close()

        elif event.key == "up":
            state["speed"] *= 1.5
            print(f"⚡ 播放速度: {state['speed']:.2f}x")

        elif event.key == "down":
            state["speed"] /= 1.5
            print(f"🐢 播放速度: {state['speed']:.2f}x")

        elif event.key == "left" and paused:
            frame_idx = max(0, frame_idx - 1)
            update_plot(frame_idx)

        elif event.key == "right" and paused:
            frame_idx = min(end - 1, frame_idx + 1)
            update_plot(frame_idx)

    fig.canvas.mpl_connect("key_press_event", on_key)

    # -------------------------
    # 播放主循环
    # -------------------------
    while frame_idx < end:
        update_plot(frame_idx)

        t0 = time.time()
        while paused:
            plt.pause(0.1)

        frame_idx += 1
        elapsed = time.time() - t0
        plt.pause(max(0.01, 0.2 / state["speed"] - elapsed))


# ==========================
# 主程序入口
# ==========================
if __name__ == "__main__":
    base = "/home/bob/文档/备份/demo05/src/test/txt/"
    cost_file = base + "speed_cost/speed_cost_0.txt"

    if not os.path.exists(cost_file):
        print("❌ 文件不存在:", cost_file)
        exit()

    frames = load_st_cost_file(cost_file)
    print(f"加载到 {len(frames)} 帧 ST-Cost 数据")

    play_cost_frames(frames, start=0, end=None, speed=1.0)
