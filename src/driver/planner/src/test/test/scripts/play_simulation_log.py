import matplotlib.pyplot as plt
import time
import numpy as np

# ================== 日志解析 ==================
def load_simulation_log(filename):
    frames = []
    last_time = None
    with open(filename, "r") as f:
        lines = f.readlines()

    frame = {"time": 0.0,
             "ego": None, "ego_yaw": 0.0,
             "planning_init": None,
             "last_traj": [],
             "reference_line": [],
             "frame_header_time": None,
             "last_header_time": None}

    in_last = False
    in_ref = False

    for line in lines:
        line = line.strip()
        if not line:
            continue

        if line.startswith("time(ms):"):
            t = float(line.split(":")[1])
            if last_time is not None and abs(t - last_time) < 1e-3:
                continue
            last_time = t

            if frame["ego"] or frame["last_traj"] or frame["planning_init"] or frame["reference_line"]:
                frames.append(frame)

            frame = {"time": t,
                     "ego": None, "ego_yaw": 0.0,
                     "planning_init": None,
                     "last_traj": [],
                     "reference_line": [],
                     "frame_header_time": None,
                     "last_header_time": None}
            in_last = False
            in_ref = False

        elif line.startswith("position") and not in_ref:
            x, y, _ = map(float, line.split()[1:4])
            frame["ego"] = (x, y)

        elif line.startswith("# Reference Line"):
            in_ref = True
            continue
        elif line.startswith("# Ego Odom"):
            in_ref = False
            continue
        elif in_ref:
            if not line or line.startswith("#") or set(line) <= {"=", "-"}:
                continue
            parts = line.split()
            try:
                vals = list(map(float, parts))
            except ValueError:
                continue
            if len(vals) >= 4:
                s, x, y, theta = vals[:4]
                frame["reference_line"].append((x, y, theta))

        elif line.startswith("# Planning Init Point"):
            in_last = False
        elif line.startswith("x "):
            px = float(line.split()[1])
            if frame["planning_init"] is None:
                frame["planning_init"] = [px, 0.0]
            else:
                frame["planning_init"][0] = px
        elif line.startswith("y "):
            py = float(line.split()[1])
            if frame["planning_init"] is None:
                frame["planning_init"] = [0.0, py]
            else:
                frame["planning_init"][1] = py

        elif line.startswith("header_time"):
            frame["last_header_time"] = float(line.split()[1])
        elif line.startswith("frame_header_time"):
            frame["frame_header_time"] = float(line.split()[1])
        elif line.startswith("# Last Optimized Trajectory"):
            in_last = True
            continue
        elif line.startswith("# Current Optimized Trajectory"):
            in_last = False
            continue
        elif line.startswith("-----------------------------------"):
            in_last = False
            continue

        elif in_last:
            vals = list(map(float, line.split()))
            if len(vals) >= 2:
                frame["last_traj"].append((vals[0], vals[1]))

    if frame["ego"] or frame["last_traj"] or frame["planning_init"] or frame["reference_line"]:
        frames.append(frame)

    return frames


# ================== 播放 ==================
def play_simulation(frames, start=0, end=None, speed=1.0):
    if end is None:
        end = len(frames)

    # ==== 计算所有点的范围 ====
    all_x, all_y = [], []
    for fr in frames[start:end]:
        if fr["ego"]:
            all_x.append(fr["ego"][0]); all_y.append(fr["ego"][1])
        if fr["planning_init"]:
            all_x.append(fr["planning_init"][0]); all_y.append(fr["planning_init"][1])
        for (x, y) in fr["last_traj"]:
            all_x.append(x); all_y.append(y)
        for (x, y, _) in fr["reference_line"]:
            all_x.append(x); all_y.append(y)

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.grid(True)

    if all_x and all_y:
        margin = 10.0
        ax.set_xlim(min(all_x)-margin, max(all_x)+margin)
        ax.set_ylim(min(all_y)-margin, max(all_y)+margin)

    ego_dot, = ax.plot([], [], "bo", label="Ego")
    init_dot, = ax.plot([], [], "ro", label="Planning Init")
    last_traj_line, = ax.plot([], [], "c--", alpha=0.7, label="Last Traj")
    last_traj_points, = ax.plot([], [], "co", markersize=3, alpha=0.6, label="Traj Points")
    ref_line, = ax.plot([], [], "k-", alpha=0.5, label="Reference Line")
    ref_arrows = []

    plt.legend().set_draggable(True)

    # ==== 信息文本框 ====
    info_text = ax.annotate("",
                            xy=(0.02, 0.98),
                            xycoords="axes fraction",
                            ha="left", va="top",
                            fontsize=9, color="black",
                            bbox=dict(facecolor="white", alpha=0.7, edgecolor="black"))
    info_text.draggable(True)

    paused = False
    frame_idx = start

    # ---- 键盘事件 ----
    def on_key(event):
        nonlocal paused, speed, frame_idx
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

    # ---- 更新帧 ----
    def update_plot(idx):
        nonlocal ref_arrows
        frame = frames[idx]

        if frame["ego"]:
            ego_dot.set_data([frame["ego"][0]], [frame["ego"][1]])

        if frame["planning_init"]:
            init_dot.set_data([frame["planning_init"][0]], [frame["planning_init"][1]])

        lx, ly = zip(*frame["last_traj"]) if frame["last_traj"] else ([], [])
        last_traj_line.set_data(lx, ly)
        last_traj_points.set_data(lx, ly)

        # 参考线绘制
        if frame["reference_line"]:
            rx = [p[0] for p in frame["reference_line"]]
            ry = [p[1] for p in frame["reference_line"]]
            ref_line.set_data(rx, ry)

            # 清除旧箭头
            for arr in ref_arrows:
                arr.remove()
            ref_arrows = []

            # 添加方向箭头
            for (x, y, theta) in frame["reference_line"]:  # 每个点都画
                arr = ax.arrow(x, y,
                            0.5*np.cos(theta), 0.5*np.sin(theta),
                            head_width=0.3, head_length=0.5, fc="k", ec="k")
                ref_arrows.append(arr)

        ax.set_title(f"Frame {idx+1}/{end}")

        fht = frame["frame_header_time"] if frame["frame_header_time"] else 0.0
        lht = frame["last_header_time"] if frame["last_header_time"] else 0.0
        dt = fht - lht if (fht and lht) else 0.0

        info_text.set_text(
            f"t={frame['time']:.0f} ms\n"
            f"FrameHeader: {fht:.3f}\n"
            f"LastHeader: {lht:.3f}\n"
            f"Δt={dt:.3f}"
        )

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # ---- 播放循环 ----
    while frame_idx < end:
        update_plot(frame_idx)
        t_start = time.time()
        while paused:
            plt.pause(0.1)
        frame_idx += 1
        elapsed = time.time() - t_start
        plt.pause(max(0.01, 0.1 / speed - elapsed))


# ================== 主程序入口 ==================
if __name__ == "__main__":
    frames = load_simulation_log(
        "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/frame_data.txt"
    )
    print(f"共 {len(frames)} 帧")
    start = int(input("请输入开始帧 (默认0): ") or 0)
    end = input(f"请输入结束帧 (最大{len(frames)}): ")
    end = int(end) if end else len(frames)
    speed = float(input("请输入播放速度 (1.0=正常): ") or 1.0)

    play_simulation(frames, start=start, end=end, speed=speed)
