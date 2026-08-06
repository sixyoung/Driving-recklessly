import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ============ 工具函数 ============

def load_multi_frame_xy(filename, mode="path"):
    frames = []
    cur_x, cur_y = [], []
    with open(filename) as f:
        for line in f:
            if line.startswith("="):
                if cur_x:
                    frames.append((cur_x, cur_y))
                cur_x, cur_y = [], []
                continue
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            if mode == "traj":
                cur_x.append(float(parts[2]))
                cur_y.append(float(parts[3]))
            else:
                cur_x.append(float(parts[1]))
                cur_y.append(float(parts[2]))
    if cur_x:
        frames.append((cur_x, cur_y))
    return frames


def load_multi_frame_speed(filename):
    frames = []
    cur_t, cur_v = [], []
    with open(filename) as f:
        for line in f:
            if line.startswith("="):
                if cur_t:
                    frames.append((cur_t, cur_v))
                cur_t, cur_v = [], []
                continue
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            cur_t.append(float(parts[0]))
            cur_v.append(float(parts[2]))
    if cur_t:
        frames.append((cur_t, cur_v))
    return frames


def load_multi_frame_speed_ts(filename):
    frames = []
    cur_t, cur_s = [], []
    with open(filename) as f:
        for line in f:
            if line.startswith("="):
                if cur_t:
                    frames.append((cur_t, cur_s))
                cur_t, cur_s = [], []
                continue
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            cur_t.append(float(parts[0]))
            cur_s.append(float(parts[1]))
    if cur_t:
        frames.append((cur_t, cur_s))
    return frames


def load_multi_frame_init(filename):
    frames = []
    cur_x, cur_y = None, None
    with open(filename) as f:
        for line in f:
            if line.startswith("="):
                if cur_x is not None:
                    frames.append((cur_x, cur_y))
                cur_x, cur_y = None, None
                continue
            if line.startswith("#") or not line.strip():
                continue
            if line.startswith("x "):
                cur_x = float(line.split()[1])
            elif line.startswith("y "):
                cur_y = float(line.split()[1])
    if cur_x is not None:
        frames.append((cur_x, cur_y))
    return frames

import math

def load_multi_frame_vehicle_odom(filename):
    """
    读取车辆里程计文件 (pose + twist)，解析每帧的 x, y, yaw。
    yaw 由四元数转换得到：yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    """
    frames = []
    cur_x, cur_y, cur_yaw = None, None, None
    qx = qy = qz = qw = 0.0

    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("="):
                # 遇到新的帧分隔符，保存上一帧
                if cur_x is not None:
                    # 计算yaw
                    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                                     1.0 - 2.0 * (qy * qy + qz * qz))
                    frames.append((cur_x, cur_y, yaw))
                cur_x = cur_y = cur_yaw = None
                qx = qy = qz = qw = 0.0
                continue

            if line.startswith("position"):
                parts = line.split()
                cur_x = float(parts[1])
                cur_y = float(parts[2])
            elif line.startswith("orientation"):
                parts = line.split()
                qx = float(parts[1])
                qy = float(parts[2])
                qz = float(parts[3])
                qw = float(parts[4])

    # 文件末尾再保存最后一帧
    if cur_x is not None:
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        frames.append((cur_x, cur_y, yaw))

    return frames



def load_multi_frame_st(filename):
    frames = []
    cur_frame = []
    cur_upper, cur_lower = [], []
    in_upper, in_lower = False, False

    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("="):
                if cur_upper or cur_lower:
                    cur_frame.append((cur_upper, cur_lower))
                if cur_frame:
                    frames.append(cur_frame)
                cur_frame = []
                cur_upper, cur_lower = [], []
                continue
            if line.startswith("Obstacle"):
                if cur_upper or cur_lower:
                    cur_frame.append((cur_upper, cur_lower))
                cur_upper, cur_lower = [], []
                in_upper, in_lower = False, False
                continue
            if line.startswith("# Upper"):
                in_upper, in_lower = True, False
                continue
            if line.startswith("# Lower"):
                in_upper, in_lower = False, True
                continue
            parts = line.split()
            if len(parts) == 2:
                t, s = float(parts[0]), float(parts[1])
                if in_upper:
                    cur_upper.append((t, s))
                elif in_lower:
                    cur_lower.append((t, s))

    if cur_upper or cur_lower:
        cur_frame.append((cur_upper, cur_lower))
    if cur_frame:
        frames.append(cur_frame)

    return frames


# ============ 动画播放器 ============

def main():
    base = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/"

    path_frames = load_multi_frame_xy(base + "qp_path/qp_path_0.txt", "path")
    traj_frames = load_multi_frame_xy(base + "optim_traj/optim_traj_0.txt", "traj")
    ref_frames  = load_multi_frame_xy(base + "reference_line_log/reference_line_log_0.txt", "path")
    init_points = load_multi_frame_init(base + "init_point_log/init_point_log_0.txt")
    speed_frames = load_multi_frame_speed(base + "qp_speed/qp_speed_0.txt")       # (t,v)
    speed_ts_frames = load_multi_frame_speed_ts(base + "qp_speed/qp_speed_0.txt") # (t,s)
    dp_speed_ts_frames = load_multi_frame_speed_ts(base + "dp_speed/dp_speed_0.txt") # ✅ 新增 (t,s)
    st_frames = load_multi_frame_st(base + "st_boundaries/st_boundaries_0.txt")
    vehicle_odom_frames = load_multi_frame_vehicle_odom(base + "vehicle_odom/vehicle_odom_0.txt")
    n_frames = max(len(path_frames), len(traj_frames), len(ref_frames),
                   len(init_points), len(speed_frames),
                   len(speed_ts_frames), len(dp_speed_ts_frames), len(st_frames))

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

    # 图1: 轨迹
    line_ref,  = ax1.plot([], [], "k--", label="Reference Line")
    dots_ref,  = ax1.plot([], [], "ko", markersize=6)
    line_path, = ax1.plot([], [], "b-",  label="QP Path")
    dots_path, = ax1.plot([], [], "bx", markersize=10)
    line_traj, = ax1.plot([], [], "r-",  label="Optim Trajectory")
    dots_traj, = ax1.plot([], [], "ro", markersize=2)
    point_init,= ax1.plot([], [], "go",  label="Init Point")
    # ✅ 自车位置与朝向
    ego_marker, = ax1.plot([], [], marker="*", color="blue", markersize=12, label="Ego Vehicle")
    ego_arrow = None
    ax1.set_title("Trajectory Playback")
    ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
    ax1.axis("equal"); ax1.legend(); ax1.grid()

    # 图2: 速度 (t-v)
    line_speed, = ax2.plot([], [], "g-", label="Velocity")
    ax2.set_title("Speed Profile Playback")
    ax2.set_xlabel("t [s]"); ax2.set_ylabel("v [m/s]")
    ax2.legend(); ax2.grid()

    # 图3: ST 图 (t-s)
    ax3.set_title("ST Boundaries & Planned Speed (t–s)")
    ax3.set_xlabel("t [s]"); ax3.set_ylabel("s [m]")
    ax3.grid()

    # 初始化 ST 边界
    st_lines = []
    max_obs = max(len(frame) for frame in st_frames) if st_frames else 0
    for _ in range(max_obs * 2):
        line, = ax3.plot([], [], "m--", linewidth=1)
        st_lines.append(line)

    # 初始化 (t,s) 曲线
    line_speed_ts, = ax3.plot([], [], "g-", linewidth=2, label="QP t–s")
    line_dp_speed_ts, = ax3.plot([], [], "b-", linewidth=2, label="DP t–s")  # ✅ 新增
    ax3.legend()

    # 状态
    state = {"paused": False, "speed": 500, "frame": 0}

    def init():
        for artist in [line_ref, dots_ref, line_path, dots_path,
                       line_traj, dots_traj, point_init,
                       line_speed, line_speed_ts, line_dp_speed_ts]:
            artist.set_data([], [])
        for line in st_lines:
            line.set_data([], [])
        return [line_ref, dots_ref, line_path, dots_path,
                line_traj, dots_traj, point_init,
                line_speed, line_speed_ts, line_dp_speed_ts,
                ego_marker] + st_lines

    def update(_):
        f = state["frame"]

        # 图1: 轨迹
        if f < len(ref_frames):
            xs, ys = ref_frames[f]; line_ref.set_data(xs, ys); dots_ref.set_data(xs, ys)
        if f < len(path_frames):
            xs, ys = path_frames[f]; line_path.set_data(xs, ys); dots_path.set_data(xs, ys)
        if f < len(traj_frames):
            xs, ys = traj_frames[f]; line_traj.set_data(xs, ys); dots_traj.set_data(xs, ys)
        if f < len(init_points):
            point_init.set_data([init_points[f][0]], [init_points[f][1]])
        # ✅ 自车位置与朝向
        if f < len(vehicle_odom_frames):
            x, y, yaw = vehicle_odom_frames[f]
            ego_marker.set_data([x], [y])

            nonlocal ego_arrow
            if ego_arrow:
                ego_arrow.remove()

            arrow_len = 0.8  # 可调箭头长度
            ego_arrow = ax1.arrow(
                x, y,
                arrow_len * math.cos(yaw),
                arrow_len * math.sin(yaw),
                head_width=0.2, head_length=0.3,
                fc="blue", ec="blue", alpha=0.8
            )

        # 图2: 速度 (t-v)
        if f < len(speed_frames):
            t, v = speed_frames[f]
            line_speed.set_data(t, v)
            ax2.relim(); ax2.autoscale_view()

        # 图3: ST (t-s)
        for line in st_lines:
            line.set_data([], [])
        if f < len(st_frames):
            frame_obs = st_frames[f]
            for j, (upper, lower) in enumerate(frame_obs):
                if upper:
                    t_vals, s_vals = zip(*upper)
                    st_lines[2*j].set_data(t_vals, s_vals)
                if lower:
                    t_vals, s_vals = zip(*lower)
                    st_lines[2*j+1].set_data(t_vals, s_vals)
        if f < len(speed_ts_frames):
            t_vals, s_vals = speed_ts_frames[f]
            line_speed_ts.set_data(t_vals, s_vals)
        if f < len(dp_speed_ts_frames):  # ✅ 新增 DP 曲线
            t_vals, s_vals = dp_speed_ts_frames[f]
            line_dp_speed_ts.set_data(t_vals, s_vals)

        ax1.relim(); ax1.autoscale_view()
        ax3.relim(); ax3.autoscale_view()

        # 显示帧号
        ax1.set_title(f"Trajectory Playback (Frame {f+1}/{n_frames})")
        ax2.set_title(f"Speed Profile Playback (Frame {f+1}/{n_frames})")
        ax3.set_title(f"ST Boundaries & Planned Speed (Frame {f+1}/{n_frames})")

        if not state["paused"]:
            state["frame"] = (state["frame"] + 1) % n_frames

        return [line_ref, dots_ref, line_path, dots_path,
                line_traj, dots_traj, point_init,
                line_speed, line_speed_ts, line_dp_speed_ts,
                ego_marker] + st_lines

    ani = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init,
        blit=False, interval=state["speed"], repeat=True
    )

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "right":
            state["frame"] = (state["frame"] + 1) % n_frames
            update(state["frame"]); fig.canvas.draw()
        elif event.key == "left":
            state["frame"] = (state["frame"] - 1) % n_frames
            update(state["frame"]); fig.canvas.draw()
        elif event.key == "up":
            state["speed"] = max(50, state["speed"] - 50)
            ani.event_source.interval = state["speed"]
            print(f"⏩ Speed up: {state['speed']} ms")
        elif event.key == "down":
            state["speed"] += 50
            ani.event_source.interval = state["speed"]
            print(f"⏬ Slow down: {state['speed']} ms")

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.tight_layout(); plt.show()


if __name__ == "__main__":
    main()
