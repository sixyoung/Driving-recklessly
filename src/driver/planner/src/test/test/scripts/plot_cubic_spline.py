import matplotlib.pyplot as plt
import numpy as np

def load_raw(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 3:
                s, x, y = map(float, parts[:3])
                data.append((s, x, y))
    return data

def load_smoothed(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 5:
                s, x, y, yaw, kappa = map(float, parts[:5])
                data.append((s, x, y, yaw, kappa))
    return data

def load_profile(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 5:
                s, x, y, heading, kappa = map(float, parts[:5])
                data.append((s, x, y, heading, kappa))
    return data

def compute_curvature_from_points(xs, ys, ss):
    xs = np.array(xs)
    ys = np.array(ys)
    ss = np.array(ss)

    dx = np.gradient(xs, ss)
    dy = np.gradient(ys, ss)
    ddx = np.gradient(dx, ss)
    ddy = np.gradient(dy, ss)

    kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
    return kappa

if __name__ == "__main__":
    raw_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/raw_path.txt"
    smooth_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/smoothed_path.txt"
    profile_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/profile_path.txt"

    raw_points = load_raw(raw_file)
    smoothed_points = load_smoothed(smooth_file)
    profile_points = load_profile(profile_file)

    s_raw = [p[0] for p in raw_points]
    x_raw = [p[1] for p in raw_points]
    y_raw = [p[2] for p in raw_points]

    s_smooth = [p[0] for p in smoothed_points]
    x_smooth = [p[1] for p in smoothed_points]
    y_smooth = [p[2] for p in smoothed_points]
    kappa_smooth = [p[4] for p in smoothed_points]

    s_profile = [p[0] for p in profile_points]
    kappa_profile = [p[4] for p in profile_points]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # === 子图1: XY轨迹对比 ===
    ax1 = axes[0]
    # 原始数据：蓝色x点 + 折线
    ax1.plot(x_raw, y_raw, "b-", linewidth=1, alpha=0.7)
    ax1.scatter(x_raw, y_raw, c="blue", marker="x", s=40, label="Raw Path")
    # 光滑数据：红色曲线（不用点）
    ax1.plot(x_smooth, y_smooth, "r-", linewidth=2, label="Smoothed Path")

    # 起点/终点标注（光滑轨迹首尾）
    ax1.scatter(x_smooth[0], y_smooth[0], c="green", s=80, marker="o", label="Start")
    ax1.text(x_smooth[0], y_smooth[0], "Start", fontsize=10, verticalalignment="bottom")
    ax1.scatter(x_smooth[-1], y_smooth[-1], c="purple", s=80, marker="x", label="End")
    ax1.text(x_smooth[-1], y_smooth[-1], "End", fontsize=10, verticalalignment="bottom")

    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_title("Raw vs Smoothed Path")
    ax1.grid(True)
    ax1.legend()

    # === 子图2: s-kappa 对比 ===
    ax2 = axes[1]
    ax2.plot(s_smooth, kappa_smooth, "g-", linewidth=2, label="κ(s) Smoothed")
    kappa_computed = compute_curvature_from_points(x_smooth, y_smooth, s_smooth)
    ax2.plot(s_smooth, kappa_computed, "r--", linewidth=2, label="κ(s) Recomputed")
    ax2.plot(s_profile, kappa_profile, "b-.", linewidth=2, label="κ(s) Profile")

    ax2.set_xlabel("s [m]")
    ax2.set_ylabel("Curvature κ [1/m]")
    ax2.set_title("Curvature Comparison")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()
