import matplotlib.pyplot as plt
import numpy as np
import time

# ================== 读取日志文件 ==================
def load_frames(filename):
    frames = []
    with open(filename, "r") as f:
        lines = f.readlines()

    # 单帧模板
    def new_frame():
        return {
            "time": 0.0,
            "ego": None, "ego_yaw": 0.0,
            "target": None, "target_yaw": 0.0,
            "traj": [],
            "global_plan": [],
            "reference_line": [],
            "control": {"throttle": 0.0, "brake": 0.0, "steer": 0.0},
            "speed": {"ego": 0.0, "target": 0.0},
            "lon_pid": {"error": 0.0, "error_integral": 0.0, "error_derivative": 0.0,
                        "p_term": 0.0, "i_term": 0.0, "d_term": 0.0, "pid_output": 0.0},
            "lat_pid": {"heading_error": 0.0, "e_y": 0.0,
                        "cross_term": 0.0, "stanley_term": 0.0,
                        "delta_raw": 0.0, "output": 0.0}
        }

    frame = new_frame()

    # 状态机标志
    in_traj = in_ego = in_target = in_control = False
    in_speed = in_lon_pid = in_lat_pid = False
    in_global = False
    in_reference = False

    for line in lines:
        line = line.strip()
        if not line:
            continue

        # ==== 时间戳 ====
        if line.startswith("time(ms):"):
            if frame["ego"] or frame["target"] or frame["traj"]:
                frames.append(frame)
            frame = new_frame()
            frame["time"] = float(line.split(":")[1])

            in_traj = in_ego = in_target = in_control = in_speed = False
            in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue

        # ==== 区块切换 ====
        if line.startswith("# Ego Pose"):
            in_ego = True
            in_target = in_traj = in_control = in_speed = in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue
        elif line.startswith("# Target Pose"):
            in_target = True
            in_ego = in_traj = in_control = in_speed = in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue
        elif line.startswith("# Control Command"):
            in_control = True
            in_ego = in_target = in_traj = in_speed = in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue
        elif line.startswith("# Speed Info"):
            in_speed = True
            in_ego = in_target = in_traj = in_control = in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue
        elif line.startswith("# Planned Trajectory"):
            in_traj = True
            in_ego = in_target = in_control = in_speed = in_lon_pid = in_lat_pid = in_global = in_reference = False
            continue
        elif line.startswith("# Global Plan"):
            in_global = True
            in_ego = in_target = in_traj = in_control = in_speed = in_lon_pid = in_lat_pid = in_reference = False
            continue
        elif line.startswith("# Reference Line"):
            in_reference = True
            in_ego = in_target = in_traj = in_control = in_speed = in_lon_pid = in_lat_pid = in_global = False
            continue
        elif line.startswith("# PID Debug Info"):
            in_lon_pid = True
            in_lat_pid = in_ego = in_target = in_traj = in_control = in_speed = in_global = in_reference = False
            continue
        elif line.startswith("# Lateral Debug Info"):
            in_lat_pid = True
            in_lon_pid = in_ego = in_target = in_traj = in_control = in_speed = in_global = in_reference = False
            continue

        # ==== Ego Pose ====
        if in_ego and line.startswith("position"):
            x, y, _ = map(float, line.split()[1:4])
            frame["ego"] = (x, y)
            continue
        if in_ego and line.startswith("yaw"):
            frame["ego_yaw"] = float(line.split()[1])
            continue

        # ==== Target Pose ====
        if in_target and line.startswith("position"):
            x, y, _ = map(float, line.split()[1:4])
            frame["target"] = (x, y)
            continue
        if in_target and line.startswith("yaw"):
            frame["target_yaw"] = float(line.split()[1])
            continue

        # ==== Control ====
        if in_control:
            k, v = line.split()
            frame["control"][k] = float(v)
            continue

        # ==== Speed ====
        if in_speed:
            k, v = line.split()
            frame["speed"][k.split("_")[0]] = float(v)
            continue

        # ==== PID Long Debug ====
        if in_lon_pid:
            k, v = line.split()
            frame["lon_pid"][k] = float(v)
            continue

        # ==== PID Lat Debug（最小修改） ====
        if in_lat_pid:
            k, v = line.split()
            if k == "lat_output":
                frame["lat_pid"]["output"] = float(v)
            else:
                frame["lat_pid"][k] = float(v)
            continue

        # ==== Global Plan ====
        if in_global:
            if line.startswith("-----------------------------------"):
                in_global = False
                continue
            vals = list(map(float, line.split()))
            if len(vals) >= 2:
                frame["global_plan"].append((vals[0], vals[1]))
            continue

        # ==== Reference Line ====
        if in_reference:
            if line.startswith("-----------------------------------"):
                in_reference = False
                continue
            vals = list(map(float, line.split()))
            if len(vals) >= 2:
                frame["reference_line"].append((vals[0], vals[1]))
            continue

        # ==== Trajectory ====
        if in_traj:
            if line.startswith("-----------------------------------"):
                in_traj = False
                continue
            vals = list(map(float, line.split()))
            if len(vals) >= 4:
                frame["traj"].append((vals[0], vals[1], vals[3]))
            elif len(vals) >= 2:
                frame["traj"].append((vals[0], vals[1], 0.0))
            continue

    if frame["ego"] or frame["target"] or frame["traj"]:
        frames.append(frame)

    return frames


# ================== 动画播放 ==================
def play_frames(frames, start=0, end=None, speed=1.0, zoom_y=1.0):
    if end is None:
        end = len(frames)

    # ---- 收集坐标范围 ----
    all_x, all_y = [], []
    for fr in frames[start:end]:
        if fr["ego"]: all_x.append(fr["ego"][0]); all_y.append(fr["ego"][1])
        if fr["target"]: all_x.append(fr["target"][0]); all_y.append(fr["target"][1])
        for x, y, _ in fr["traj"]: all_x.append(x); all_y.append(y)
        for x, y in fr["global_plan"]: all_x.append(x); all_y.append(y)
        for x, y in fr["reference_line"]: all_x.append(x); all_y.append(y)

    margin = 0.5
    x_min, x_max = min(all_x)-margin, max(all_x)+margin
    y_min, y_max = min(all_y)-margin, max(all_y)+margin
    cx, cy = (x_min+x_max)/2, (y_min+y_max)/2

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_aspect("equal")
    ax.grid(True)

    def apply_zoom_y():
        ax.set_xlim(x_min, x_max)
        height = (y_max - y_min) / zoom_y
        ax.set_ylim(cy - height/2, cy + height/2)
        fig.canvas.draw_idle()

    apply_zoom_y()

    ego_dot, = ax.plot([], [], "bo", markersize=6)
    target_dot, = ax.plot([], [], "rx", markersize=6)
    traj_line, = ax.plot([], [], "g-", linewidth=2)
    traj_pts, = ax.plot([], [], "go", markersize=3)

    global_line, = ax.plot([], [], "k--", alpha=0.4)
    reference_line_plot, = ax.plot([], [], color="yellow", linewidth=3, alpha=0.9)

    ego_arrow = target_arrow = None
    traj_arrows = []

    # ======================
    # ⭐ 新增：Lateral Debug 文本
    # ======================
    lat_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top",
        fontsize=10, bbox=dict(boxstyle="round", fc="white", alpha=0.6)
    )

    # ======================
    # ⭐ 新增：PID Longitudinal Debug 文本
    # ======================
    pid_text = ax.text(
        0.80, 0.98, "", transform=ax.transAxes, va="top",
        fontsize=10, bbox=dict(boxstyle="round", fc="white", alpha=0.6)
    )

    paused = False
    frame_idx = start

    # ---- 键盘控制 ----
    def on_key(event):
        nonlocal paused, speed, frame_idx, zoom_y
        if event.key == " ":
            paused = not paused
        elif event.key == "escape":
            plt.close()
        elif event.key == "up":
            speed *= 1.5
        elif event.key == "down":
            speed /= 1.5
        elif event.key == "left" and paused:
            frame_idx = max(start, frame_idx-1)
            update_plot(frame_idx)
        elif event.key == "right" and paused:
            frame_idx = min(end-1, frame_idx+1)
            update_plot(frame_idx)
        elif event.key in ["+", "="]:
            zoom_y *= 1.2; apply_zoom_y()
        elif event.key in ["-", "_"]:
            zoom_y /= 1.2; apply_zoom_y()

    fig.canvas.mpl_connect("key_press_event", on_key)

    # ---- 更新绘图 ----
    def update_plot(idx):
        nonlocal ego_arrow, target_arrow, traj_arrows

        fr = frames[idx]

        # Ego
        if fr["ego"]:
            ex, ey = fr["ego"]
            ego_dot.set_data([ex], [ey])
            yaw = fr["ego_yaw"]
            if ego_arrow: ego_arrow.remove()
            ego_arrow = ax.arrow(ex, ey, 0.3*np.cos(yaw), 0.3*np.sin(yaw),
                                 head_width=0.2, color="b")

        # Target
        if fr["target"]:
            tx, ty = fr["target"]
            target_dot.set_data([tx], [ty])
            yaw = fr["target_yaw"]
            if target_arrow: target_arrow.remove()
            target_arrow = ax.arrow(tx, ty, 0.3*np.cos(yaw), 0.3*np.sin(yaw),
                                    head_width=0.2, color="r")

        # Local Trajectory
        xs = [p[0] for p in fr["traj"]]
        ys = [p[1] for p in fr["traj"]]
        vs = [p[2] for p in fr["traj"]]

        traj_line.set_data(xs, ys)
        traj_pts.set_data(xs, ys)

        for a in traj_arrows:
            a.remove()
        traj_arrows = []
        for x, y, v in zip(xs, ys, vs):
            arr = ax.arrow(x, y, 0.2 if v >= 0 else -0.2, 0,
                           head_width=0.1, alpha=0.7, color="g")
            traj_arrows.append(arr)

        # Global Plan
        gxs = [p[0] for p in fr["global_plan"]]
        gys = [p[1] for p in fr["global_plan"]]
        global_line.set_data(gxs, gys)

        # Reference Line
        rxs = [p[0] for p in fr["reference_line"]]
        rys = [p[1] for p in fr["reference_line"]]
        reference_line_plot.set_data(rxs, rys)

        # ===========================
        # ⭐ 新增：显示 Lateral Debug
        # ===========================
        lat = fr["lat_pid"]
        lat_text.set_text(
            "Lateral Debug:\n"
            f"heading_error = {lat['heading_error']:.4f}\n"
            f"e_y = {lat['e_y']:.4f}\n"
            f"cross_term = {lat['cross_term']:.4f}\n"
            f"stanley_term = {lat['stanley_term']:.4f}\n"
            f"delta_raw = {lat['delta_raw']:.4f}\n"
            f"output = {lat['output']:.4f}"
        )

        # ===========================
        # ⭐ 新增：显示 PID Long Debug
        # ===========================
        pid = fr["lon_pid"]
        pid_text.set_text(
            "PID Debug:\n"
            f"error = {pid['error']:.4f}\n"
            f"error_integral = {pid['error_integral']:.4f}\n"
            f"error_derivative = {pid['error_derivative']:.4f}\n"
            f"p_term = {pid['p_term']:.4f}\n"
            f"i_term = {pid['i_term']:.4f}\n"
            f"d_term = {pid['d_term']:.4f}\n"
            f"pid_output = {pid['pid_output']:.4f}"
        )

        ax.set_title(f"Frame {idx+1}/{end}")
        fig.canvas.draw_idle()

    # ---- 主循环 ----
    while frame_idx < end:
        update_plot(frame_idx)
        t0 = time.time()

        if paused:
            plt.waitforbuttonpress(timeout=-1)
            continue

        frame_idx += 1
        dt = time.time() - t0
        plt.pause(max(0.01, 0.1/speed - dt))


# ================== 主程序入口 ==================
if __name__ == "__main__":
    frames = load_frames("/home/bob/文档/备份/demo05/src/test/txt/frame_log/frame_log_0.txt")
    print(f"共 {len(frames)} 帧")

    start = int(input("开始帧(默认0): ") or 0)
    end = input(f"结束帧(最大 {len(frames)}): ")
    end = int(end) if end else len(frames)

    speed = float(input("播放速度(1.0=正常): ") or 1.0)
    zoom_y = float(input("Y轴放大倍数(1.0=正常): ") or 1.0)

    play_frames(frames, start=start, end=end, speed=speed, zoom_y=zoom_y)
