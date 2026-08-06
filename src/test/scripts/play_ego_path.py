import matplotlib.pyplot as plt
import numpy as np
import math
import time

# ================== 读取 Ego Path ==================
def load_ego_path(filename):
    frames = []
    current = {"frame": None, "time": None, "all_local": [], "final_path": []}
    mode = None

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("============================="):
                if current["all_local"] or current["final_path"]:
                    frames.append(current)
                current = {"frame": None, "time": None, "all_local": [], "final_path": []}
                mode = None
                continue

            if line.startswith("# Frame:"):
                parts = line.split()
                if len(parts) >= 4:
                    current["frame"] = int(parts[2])
                    current["time"] = parts[-1]
                continue

            if line.startswith("# All Local Path Points"):
                mode = "all_local"
                continue
            if line.startswith("# Final Selected Path"):
                mode = "final"
                continue
            if line.startswith("----"):
                continue

            parts = line.split()
            if len(parts) >= 4:
                try:
                    x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
                    if mode == "all_local":
                        current["all_local"].append((x, y, yaw))
                    elif mode == "final":
                        current["final_path"].append((x, y, yaw))
                except ValueError:
                    continue

    if current["all_local"] or current["final_path"]:
        frames.append(current)
    print(f"✅ 读取 ego_path: {len(frames)} 帧")
    return frames


# ================== 读取 Vehicle Odom ==================
import math

def load_vehicle_odom(filename):
    frames = []
    current = {"frame": None, "time": None, "pose": None}

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # 分帧分隔
            if line.startswith("============================="):
                if current["pose"]:
                    frames.append(current)
                current = {"frame": None, "time": None, "pose": None}
                continue

            # 帧号与时间
            if line.startswith("# Frame:"):
                parts = line.split()
                if len(parts) >= 4:
                    current["frame"] = int(parts[2])
                    current["time"] = parts[-1]
                continue

            # 位置信息
            if line.startswith("position"):
                parts = line.split()
                if len(parts) >= 4:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    current["pose"] = [x, y, 0.0]  # 暂存 yaw = 0
                continue

            # 四元数 -> 欧拉角 (yaw)
            if line.startswith("orientation"):
                parts = line.split()
                if len(parts) >= 5:
                    qx, qy, qz, qw = map(float, parts[1:5])
                    # 根据 ROS 四元数计算 yaw
                    yaw = math.atan2(
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)
                    )
                    if current["pose"]:
                        current["pose"][2] = yaw
                continue

    # 添加最后一帧
    if current["pose"]:
        frames.append(current)

    print(f"✅ 读取 vehicle_odom: {len(frames)} 帧（含真实 yaw）")
    return frames


# ================== 播放函数 ==================
def play_ego_path(frames_path, frames_odom, start=0, end=None, speed=1.0, aspect_ratio=1.0):
    if end is None:
        end = min(len(frames_path), len(frames_odom))

    plt.rcParams["figure.dpi"] = 150
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlabel("X", fontsize=10)
    ax.set_ylabel("Y", fontsize=10)
    ax.set_title("Ego Path + Vehicle Odom Playback", fontsize=12, fontweight="bold")

    scatter_local = ax.scatter([], [], s=10, c="royalblue", alpha=0.8, label="All Local Path")
    line_final, = ax.plot([], [], "r-", linewidth=2.0, alpha=0.8, label="Final Selected Path")
    scatter_final = ax.scatter([], [], s=20, c="red", marker="o", alpha=0.8)
    ego_arrow = None
    ego_history_line, = ax.plot([], [], "g--", linewidth=1.5, alpha=0.6, label="Ego History")

    info = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                   va="top", ha="left",
                   bbox=dict(facecolor="white", alpha=0.8, edgecolor="black", boxstyle="round"))
    ax.legend(loc="upper right", fontsize=8, framealpha=0.8)

    frame_idx = start
    paused = False
    ego_history = []       # 保存历史轨迹
    path_arrows = []       # 保存路径方向箭头

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
            update_plot(frame_idx)
        elif event.key == "right" and paused:
            frame_idx = min(end - 1, frame_idx + 1)
            update_plot(frame_idx)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update_plot(idx):
        nonlocal ego_arrow, ego_history, path_arrows
        frame_path = frames_path[idx]
        frame_odom = frames_odom[idx]

        all_pts = np.array(frame_path["all_local"])
        final_pts = np.array(frame_path["final_path"])

        # 更新采样路径和最终路径
        scatter_local.set_offsets(all_pts[:, :2] if len(all_pts) > 0 else [])
        line_final.set_data(final_pts[:, 0], final_pts[:, 1] if len(final_pts) > 0 else [])
        scatter_final.set_offsets(final_pts[:, :2] if len(final_pts) > 0 else [])

        # 清除旧箭头
        for a in path_arrows:
            a.remove()
        path_arrows.clear()

        # =====================================================
        # 🟦 1️⃣ 绘制 All Local Path 的方向箭头（蓝色）+ 下标标注
        # =====================================================
        if len(all_pts) > 0:
            n_skip = 1   # 每隔多少个点画一次箭头
            label_skip = 1  # 每隔多少个点显示编号

            # 清除旧箭头与旧文字
            for a in path_arrows:
                a.remove()
            path_arrows.clear()
            for t in ax.texts[:]:  # 删除旧编号文字（不删除 info 文本）
                if t != info:
                    t.remove()

            for i in range(0, len(all_pts), n_skip):
                x, y, yaw = all_pts[i]
                arrow_len = 0.6
                arrow = ax.arrow(
                    x, y,
                    arrow_len * math.cos(yaw),
                    arrow_len * math.sin(yaw),
                    head_width=0.18,
                    head_length=0.25,
                    fc="skyblue",
                    ec="royalblue",
                    alpha=0.5,
                    length_includes_head=True
                )
                path_arrows.append(arrow)

                # 显示下标编号
                if i % label_skip == 0:
                    ax.text(
                        x + 0.2, y + 0.2,  # 文字稍微偏移避免遮挡箭头
                        str(i),
                        fontsize=7,
                        color="blue",
                        alpha=0.8
                    )


        # =====================================================
        # 🔴 2️⃣ 绘制 Final Path 的方向箭头（红色）
        # =====================================================
        if len(final_pts) > 0:
            n_skip = max(1, len(final_pts) // 20)
            for i in range(0, len(final_pts), n_skip):
                x, y, yaw = final_pts[i]
                arrow_len = 0.8
                arrow = ax.arrow(
                    x, y,
                    arrow_len * math.cos(yaw),
                    arrow_len * math.sin(yaw),
                    head_width=0.25,
                    head_length=0.4,
                    fc="salmon",
                    ec="red",
                    alpha=0.6,
                    length_includes_head=True
                )
                path_arrows.append(arrow)

        # =====================================================
        # 🟢 3️⃣ 绘制自车箭头 + 历史轨迹
        # =====================================================
        if ego_arrow:
            ego_arrow.remove()

        if frame_odom["pose"]:
            ex, ey, yaw = frame_odom["pose"]  # ✅ 真实姿态
            arrow_len = 1.5
            ego_arrow = ax.arrow(
                ex, ey,
                arrow_len * math.cos(yaw),
                arrow_len * math.sin(yaw),
                head_width=0.5,
                head_length=0.8,
                fc="limegreen",
                ec="green",
                alpha=0.9,
                label="Ego Pose"
            )

            # 记录历史轨迹
            ego_history.append((ex, ey))
            hx, hy = zip(*ego_history)
            ego_history_line.set_data(hx, hy)

        # =====================================================
        # 🔍 4️⃣ 自动缩放视野 + 信息更新
        # =====================================================
        all_x, all_y = [], []
        if len(all_pts) > 0:
            all_x.extend(all_pts[:, 0])
            all_y.extend(all_pts[:, 1])
        if len(final_pts) > 0:
            all_x.extend(final_pts[:, 0])
            all_y.extend(final_pts[:, 1])
        if frame_odom["pose"]:
            all_x.append(frame_odom["pose"][0])
            all_y.append(frame_odom["pose"][1])

        if len(all_x) > 0:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            dx = (x_max - x_min) * 0.1 + 1.0
            dy = (y_max - y_min) * 0.1 + 1.0
            ax.set_xlim(x_min - dx, x_max + dx)
            ax.set_ylim(y_min - dy, y_max + dy)

        ax.set_aspect(aspect_ratio)
        ax.set_title(f"Frame {idx+1}/{end} | Time: {frame_path['time']}", fontsize=11)
        info.set_text(f"Ego: ({frame_odom['pose'][0]:.2f}, {frame_odom['pose'][1]:.2f})\nSpeed={speed:.2f}x")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()


    # ---- 播放循环 ----
    while frame_idx < end:
        update_plot(frame_idx)
        t0 = time.time()
        while paused:
            plt.pause(0.1)
        frame_idx += 1
        elapsed = time.time() - t0
        plt.pause(max(0.02, 0.15 / speed - elapsed))

    print("✅ 播放结束")


# ================== 主程序入口 ==================
if __name__ == "__main__":
    ego_path_file = "/home/bob/文档/备份/demo05/src/test/txt/ego_path/ego_path_0.txt"
    odom_file = "/home/bob/文档/备份/demo05/src/test/txt/vehicle_odom/vehicle_odom_0.txt"

    frames_path = load_ego_path(ego_path_file)
    frames_odom = load_vehicle_odom(odom_file)

    end_max = min(len(frames_path), len(frames_odom))
    print(f"🎯 可播放帧数: {end_max}")

    start = int(input("请输入开始帧 (默认0): ") or 0)
    end = input(f"请输入结束帧 (最大{end_max}): ")
    end = int(end) if end else end_max
    speed = float(input("请输入播放速度 (默认1.0): ") or 1.0)
    aspect = float(input("请输入视觉比例 aspect (默认1.0): ") or 1.0)

    play_ego_path(frames_path, frames_odom, start=start, end=end, speed=speed, aspect_ratio=aspect)
