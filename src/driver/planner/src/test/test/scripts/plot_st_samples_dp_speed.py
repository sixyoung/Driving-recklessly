import matplotlib.pyplot as plt

def load_st_boundaries(filename):
    obstacles = []
    current = None
    mode = None
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("Obstacle"):
                if current:  # 保存上一个障碍物
                    obstacles.append(current)
                current = {"upper": [], "lower": [], "id": line}
            elif line.startswith("# Upper"):
                mode = "upper"
            elif line.startswith("# Lower"):
                mode = "lower"
            elif line[0].isdigit() or line[0] == "-":
                t, s = map(float, line.split())
                current[mode].append((t, s))
        if current:
            obstacles.append(current)
    return obstacles

def load_speed_profile(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if not line or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) == 4:
                t, s, v, a = map(float, parts)
                data.append((t, s, v, a))
    return data

def load_st_samples(filename):
    samples = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 2:
                t, s = map(float, parts)
                samples.append((t, s))
    return samples

if __name__ == "__main__":
    st_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_boundaries.txt"
    dp_speed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_speed.txt"
    sample_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_samples.txt"

    obstacles = load_st_boundaries(st_file)
    speed_data = load_speed_profile(dp_speed_file)
    st_samples = load_st_samples(sample_file)

    # === 图1: ST 图 + 速度轨迹 + 采样点 ===
    plt.figure(figsize=(10, 6))
    for obs in obstacles:
        upper = obs["upper"]
        lower = obs["lower"]
        if not upper or not lower:
            continue
        t_upper, s_upper = zip(*upper)
        t_lower, s_lower = zip(*lower)
        plt.plot(t_upper, s_upper, "r-")
        plt.plot(t_lower, s_lower, "b-")
        plt.fill_between(t_upper, s_lower, s_upper, alpha=0.3, label=obs["id"])

    # 绘制采样点
    if st_samples:
        t_vals = [p[0] for p in st_samples]
        s_vals = [p[1] for p in st_samples]
        plt.scatter(t_vals, s_vals, s=8, c="black", alpha=0.4, label="ST samples")

    # 绘制自车速度曲线
    if speed_data:
        t_vals = [p[0] for p in speed_data]
        s_vals = [p[1] for p in speed_data]
        plt.plot(t_vals, s_vals, "g-", linewidth=2, label="Ego speed profile")

    plt.xlabel("Time t (s)")
    plt.ylabel("Longitudinal distance s (m)")
    plt.title("ST Graph with Speed Profile & Samples")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # === 图2: 速度-时间曲线 ===
    plt.figure(figsize=(10, 4))
    if speed_data:
        t_vals = [p[0] for p in speed_data]
        v_vals = [p[2] for p in speed_data]
        plt.plot(t_vals, v_vals, "g-", linewidth=2)
    plt.xlabel("Time t (s)")
    plt.ylabel("Velocity v (m/s)")
    plt.title("Speed Profile")
    plt.grid(True)
    plt.tight_layout()

    plt.show()
