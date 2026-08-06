import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# ============ 工具函数 ============

def load_multi_frame_xy(filename, mode="path"):
    frames = []
    cur_x, cur_y, cur_theta = [], [], []
    with open(filename) as f:
        for line in f:
            if line.startswith("="):
                if cur_x:
                    frames.append((cur_x, cur_y, cur_theta))
                cur_x, cur_y, cur_theta = [], [], []
                continue
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            if mode == "traj":
                cur_x.append(float(parts[2]))
                cur_y.append(float(parts[3]))
                cur_theta.append(0.0)  # traj没有theta
            elif mode == "path":  # QP Path 有 theta
                cur_x.append(float(parts[1]))
                cur_y.append(float(parts[2]))
                cur_theta.append(float(parts[3]))  # ✅ 直接读 theta
            elif mode == "ref":  # Reference Line 有 theta
                cur_x.append(float(parts[1]))
                cur_y.append(float(parts[2]))
                cur_theta.append(float(parts[3]))
    if cur_x:
        frames.append((cur_x, cur_y, cur_theta))
    return frames


# ============ 动画播放器 ============

def main():
    base = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/"

    path_frames = load_multi_frame_xy(base + "qp_path.txt", "path")
    ref_frames  = load_multi_frame_xy(base + "reference_line_log.txt", "ref")

    n_frames = max(len(path_frames), len(ref_frames))

    fig, ax = plt.subplots(figsize=(8, 8))

    # 图: QP Path + Reference Line
    line_ref,  = ax.plot([], [], "k--", label="Reference Line")
    dots_ref,  = ax.plot([], [], "ko", markersize=4)
    line_path, = ax.plot([], [], "b-",  label="QP Path")
    dots_path, = ax.plot([], [], "bx", markersize=6)

    ax.set_title("QP Path & Reference Line Playback")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.axis("equal"); ax.legend(); ax.grid()

    ref_arrows = []
    path_arrows = []

    state = {"paused": False, "speed": 500, "frame": 0}

    def init():
        line_ref.set_data([], [])
        dots_ref.set_data([], [])
        line_path.set_data([], [])
        dots_path.set_data([], [])
        return [line_ref, dots_ref, line_path, dots_path]

    def update(_):
        nonlocal ref_arrows, path_arrows
        f = state["frame"]

        # QP Path
        if f < len(path_frames):
            xs, ys, thetas = path_frames[f]
            line_path.set_data(xs, ys)
            dots_path.set_data(xs, ys)

            # 清旧箭头
            for arr in path_arrows:
                arr.remove()
            path_arrows = []

            # 每个点都画箭头
            for x, y, theta in zip(xs, ys, thetas):
                arr = ax.arrow(x, y,
                               1.0*np.cos(theta), 1.0*np.sin(theta),  # 箭头长度
                               head_width=0.2, head_length=0.3, fc="b", ec="b")
                path_arrows.append(arr)

        # Reference Line
        if f < len(ref_frames):
            xs, ys, thetas = ref_frames[f]
            line_ref.set_data(xs, ys)
            dots_ref.set_data(xs, ys)

            # 清旧箭头
            for arr in ref_arrows:
                arr.remove()
            ref_arrows = []

            # 每个点都画箭头
            for x, y, theta in zip(xs, ys, thetas):
                arr = ax.arrow(x, y,
                               1.0*np.cos(theta), 1.0*np.sin(theta),
                               head_width=0.2, head_length=0.3, fc="k", ec="k")
                ref_arrows.append(arr)

        ax.set_title(f"Frame {f+1}/{n_frames}")

        if not state["paused"]:
            state["frame"] = (state["frame"] + 1) % n_frames

        return [line_ref, dots_ref, line_path, dots_path] + ref_arrows + path_arrows

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
        elif event.key == "down":
            state["speed"] += 50
            ani.event_source.interval = state["speed"]

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.tight_layout(); plt.show()


if __name__ == "__main__":
    main()
