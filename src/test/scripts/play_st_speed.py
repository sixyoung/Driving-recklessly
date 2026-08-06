import matplotlib.pyplot as plt
import matplotlib as mpl
import time
import os

# ================== 数据加载 ==================
def load_st_boundaries(filename):
    """读取 ST 边界文件，按帧划分，并记录帧号"""
    frames = []
    cur = []
    current = None
    mode = None
    current_frame_id = None

    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # 识别帧号
            if line.startswith("# Frame ID:"):
                if cur:
                    frames.append({"frame_id": current_frame_id, "data": cur})
                    cur = []
                try:
                    current_frame_id = int(line.split(":")[1].strip())
                except:
                    current_frame_id = len(frames)
                continue

            if line.startswith("="):  # 兼容旧格式
                if cur:
                    frames.append({"frame_id": current_frame_id, "data": cur})
                cur = []
                continue

            if line.startswith("Obstacle"):
                if current:
                    cur.append(current)
                current = {"upper": [], "lower": [], "id": line}
            elif line.startswith("# Upper"):
                mode = "upper"
            elif line.startswith("# Lower"):
                mode = "lower"
            elif line[0].isdigit() or line[0] == "-":
                t, s = map(float, line.split())
                if current:
                    current[mode].append((t, s))

        # 收尾
        if current:
            cur.append(current)
        if cur:
            frames.append({"frame_id": current_frame_id, "data": cur})
    return frames


def load_st_constraints_all_frames(filename):
    """解析 # Frame ID: X 格式的 ST 约束文件"""
    st_constraint_frames = []
    current_frame = None
    current_frame_id = None

    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # 新帧开始
            if line.startswith("# Frame ID:"):
                if current_frame:
                    st_constraint_frames.append(
                        {"frame_id": current_frame_id, "data": current_frame})
                current_frame = {"time": [], "s_upper": [], "s_lower": []}
                try:
                    current_frame_id = int(line.split(":")[1].strip())
                except:
                    current_frame_id = len(st_constraint_frames)
                continue

            if line.startswith("# ST Constraints"):
                continue
            if line.startswith("---"):
                if current_frame:
                    st_constraint_frames.append(
                        {"frame_id": current_frame_id, "data": current_frame})
                    current_frame = None
                continue

            if current_frame and (line[0].isdigit() or line[0] == "-"):
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        t = float(parts[0])
                        s_upper = float(parts[1])
                        s_lower = float(parts[2])
                        current_frame["time"].append(t)
                        current_frame["s_upper"].append(s_upper)
                        current_frame["s_lower"].append(s_lower)
                    except ValueError:
                        continue

    if current_frame:
        st_constraint_frames.append(
            {"frame_id": current_frame_id, "data": current_frame})
    return st_constraint_frames


def load_speed_file(filename):
    """读取 speed profile 文件 (t, s, v, a)，按帧划分并记录帧号"""
    frames = []
    cur = []
    current_frame_id = None
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# Frame ID:"):
                if cur:
                    frames.append({"frame_id": current_frame_id, "data": cur})
                    cur = []
                try:
                    current_frame_id = int(line.split(":")[1].strip())
                except:
                    current_frame_id = len(frames)
                continue
            if line.startswith("="):
                if cur:
                    frames.append({"frame_id": current_frame_id, "data": cur})
                cur = []
                continue
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 4:
                t, s, v, a = map(float, parts)
                cur.append((t, s, v, a))
    if cur:
        frames.append({"frame_id": current_frame_id, "data": cur})
    return frames


# ================== 播放器 ==================
def play_frames(st_frames, dp_frames, qp_frames, st_constraint_frames=None, start=0, end=None, speed=1.0):
    if st_constraint_frames is None:
        st_constraint_frames = []

    # -------- 提取帧号索引 --------
    def get_ids(frames):
        return [f["frame_id"] for f in frames if f.get("frame_id") is not None]

    st_ids = get_ids(st_frames)
    dp_ids = get_ids(dp_frames)
    qp_ids = get_ids(qp_frames)
    stc_ids = get_ids(st_constraint_frames)

    # 全集并集，排序
    all_ids = sorted(set(st_ids + dp_ids + qp_ids + stc_ids))
    if end is None:
        end = len(all_ids)

    def find_by_id(frames, fid):
        for f in frames:
            if f["frame_id"] == fid:
                return f["data"]
        return None

    fig, ax_st = plt.subplots(figsize=(8, 6))
    paused = False
    frame_idx = start
    state = {"speed": speed}

    def on_key(event):
        nonlocal paused, frame_idx
        if event.key == " ":
            paused = not paused
            print("⏸ 暂停" if paused else "▶ 继续")
        elif event.key == "escape":
            plt.close()
        elif event.key == "up":
            state["speed"] *= 1.5
            print(f"⚡ 播放速度: {state['speed']:.2f}x")
        elif event.key == "down":
            state["speed"] /= 1.5
            print(f"🐢 播放速度: {state['speed']:.2f}x")
        elif event.key == "left" and paused:
            frame_idx = max(start, frame_idx - 1)
            update_plot(frame_idx)
        elif event.key == "right" and paused:
            frame_idx = min(end - 1, frame_idx + 1)
            update_plot(frame_idx)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update_plot(idx):
        ax_st.clear()
        fid = all_ids[idx]
        ax_st.set_title(f"ST Graph (Frame ID: {fid}, {idx+1}/{end})")

        st = find_by_id(st_frames, fid)
        dp = find_by_id(dp_frames, fid)
        qp = find_by_id(qp_frames, fid)
        stc = find_by_id(st_constraint_frames, fid)

        # 1️⃣ 绘制 ST 边界
        if st:
            cmap = mpl.colormaps["tab20b"].resampled(len(st))
            for i, obs in enumerate(st):
                if obs["upper"] and obs["lower"]:
                    t_up, s_up = zip(*obs["upper"])
                    t_lo, s_lo = zip(*obs["lower"])
                    ax_st.fill_between(t_up, s_lo, s_up, color=cmap(i), alpha=0.2, label=obs["id"])

        # 2️⃣ 约束区域
        if stc:
            c = stc
            valid_idx = [i for i, t in enumerate(c["time"]) if t <= 9.0]
            if valid_idx:
                t = [c["time"][i] for i in valid_idx]
                s_upper = [c["s_upper"][i] for i in valid_idx]
                s_lower = [c["s_lower"][i] for i in valid_idx]
                ax_st.fill_between(t, s_lower, s_upper, color="green", alpha=0.3, label="ST Constraint")

        # 3️⃣ DP/QP 曲线
        if dp:
            t, s = zip(*[(t, s) for (t, s, v, a) in dp])
            ax_st.plot(t, s, "g-", linewidth=2, label="DP speed")
        if qp:
            t, s = zip(*[(t, s) for (t, s, v, a) in qp])
            ax_st.plot(t, s, "m--", linewidth=2, label="QP speed")

        ax_st.set_xlim(0, 9)
        ax_st.set_xlabel("t [s]")
        ax_st.set_ylabel("s [m]")
        ax_st.grid()
        ax_st.legend(loc="best")
        fig.canvas.draw()
        fig.canvas.flush_events()

    while frame_idx < end:
        update_plot(frame_idx)
        t_start = time.time()
        while paused:
            plt.pause(0.1)
        frame_idx += 1
        elapsed = time.time() - t_start
        plt.pause(max(0.01, 0.3 / state["speed"] - elapsed))


# ================== 主入口 ==================
if __name__ == "__main__":
    base = "/home/bob/文档/备份/demo05/src/test/txt/"
    st_frames = load_st_boundaries(base + "st_boundaries/st_boundaries_0.txt")
    dp_frames = load_speed_file(base + "dp_speed/dp_speed_0.txt")
    qp_frames = load_speed_file(base + "qp_speed/qp_speed_0.txt")
    st_constraint_path = base + "st_constraints/st_constraints_0.txt"
    if os.path.exists(st_constraint_path):
        st_constraint_frames = load_st_constraints_all_frames(st_constraint_path)
    else:
        print(f"⚠️ 未找到 {st_constraint_path}，跳过 ST Constraint 绘制。")
        st_constraint_frames = []

    print(f"ST={len(st_frames)}, DP={len(dp_frames)}, QP={len(qp_frames)}, ST_Constraint={len(st_constraint_frames)}")

    play_frames(st_frames, dp_frames, qp_frames, st_constraint_frames,
                start=0, end=None, speed=1.0)
