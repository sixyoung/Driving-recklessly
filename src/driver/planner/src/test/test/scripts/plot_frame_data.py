import matplotlib.pyplot as plt
import numpy as np

def load_frames(filename):
    """逐帧读取 frame_data.txt，保存 (x,y,v,a)"""
    frames, frame = [], {"ego": None, "init": None,
                         "last_traj": [], "current_traj": []}
    mode = None  # 当前轨迹类型

    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("time"):
                if frame["ego"] or frame["init"] or frame["last_traj"] or frame["current_traj"]:
                    frames.append(frame)
                frame = {"ego": None, "init": None,
                         "last_traj": [], "current_traj": []}
                frame["time"] = int(line.split(":")[1])
                mode = None
            elif line.startswith("position") and not frame["ego"]:
                parts = line.split()
                frame["ego"] = (float(parts[1]), float(parts[2]))
            elif line.startswith("x ") and not frame["init"]:
                x = float(line.split()[1])
                frame["init"] = [x, None, None]
            elif line.startswith("y ") and frame["init"][1] is None:
                y = float(line.split()[1])
                frame["init"][1] = y
            elif line.startswith("theta") and frame["init"][2] is None:
                theta = float(line.split()[1])
                frame["init"][2] = theta
            elif line.startswith("# Last Optimized Trajectory"):
                mode = "last"
                continue
            elif line.startswith("# Current Optimized Trajectory"):
                mode = "current"
                continue
            elif line.startswith("-") or line[0].isdigit():
                parts = line.split()
                if len(parts) >= 5:  # x, y, s, v, a
                    try:
                        x, y, s, v, a = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                        if mode == "last":
                            frame["last_traj"].append((x, y, v, a))
                        elif mode == "current":
                            frame["current_traj"].append((x, y, v, a))
                    except ValueError:
                        continue
        if frame["ego"] or frame["init"] or frame["last_traj"] or frame["current_traj"]:
            frames.append(frame)
    return frames


def plot_range_frames(frames, start_offset=10, end_offset=5):
    """绘制 [len(frames)-start_offset, len(frames)-end_offset) 之间的帧，并标注前3个点的 v,a + 历史 ego 位置"""
    if len(frames) < start_offset:
        print("❌ 数据不足，无法绘制该范围")
        return

    selected_frames = frames[-start_offset:-end_offset]

    for i, frame in enumerate(selected_frames):
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111)

        # --- 绘制 Last Trajectory ---
        if frame.get("last_traj"):
            xs, ys = zip(*[(p[0], p[1]) for p in frame["last_traj"]])
            ax.plot(xs, ys, "b--", linewidth=1.5, label="Last Traj")
            ax.scatter(xs, ys, c="blue", s=10, alpha=0.5)
            for k, (x, y, v, a) in enumerate(frame["last_traj"][:3]):
                ax.text(x+0.2, y+0.2+0.2*k, f"v={v:.2f}, a={a:.2f}",
                        color="blue", fontsize=8,
                        bbox=dict(facecolor="white", alpha=0.6, edgecolor="blue"))

        # --- 绘制 Current Trajectory ---
        if frame.get("current_traj"):
            xs, ys = zip(*[(p[0], p[1]) for p in frame["current_traj"]])
            ax.plot(xs, ys, "g-", linewidth=2, label="Current Traj")
            ax.scatter(xs, ys, c="green", s=15, alpha=0.7)
            for k, (x, y, v, a) in enumerate(frame["current_traj"][:3]):
                ax.text(x-0.5, y-0.3-0.2*k, f"v={v:.2f}, a={a:.2f}",
                        color="green", fontsize=8,
                        bbox=dict(facecolor="white", alpha=0.6, edgecolor="green"))

        # --- 绘制该范围内的 ego 历史位置 ---
        for j, f in enumerate(selected_frames):
            if f.get("ego"):
                xe, ye = f["ego"]
                if j == i:  # 当前帧
                    ax.scatter(xe, ye, c="red", marker="*", s=100,
                               label="Ego (current)" if j == 0 else "")
                else:       # 历史帧
                    ax.scatter(xe, ye, c="gray", marker="o", s=40, alpha=0.6,
                               label="Ego (history)" if j == 0 else "")

        # --- 图形属性 ---
        ax.set_title(f"Frame {len(frames)-start_offset+i}")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()

    plt.show()


if __name__ == "__main__":
    file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/txt/frame_data.txt"
    frames = load_frames(file)
    print(f"Loaded {len(frames)} frames.")
    # 画最后第25 ~ 第15帧
    plot_range_frames(frames, start_offset=23, end_offset=15)
