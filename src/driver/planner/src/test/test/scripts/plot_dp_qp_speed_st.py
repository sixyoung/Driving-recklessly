import matplotlib.pyplot as plt
import matplotlib as mpl
from numpy import interp

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

def load_collisions(filename):
    collisions = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                obs_id = parts[2].split("=")[1]   # "obstacle=0"
                t = float(parts[3].split("=")[1]) # "t=4.000000"
                s = float(parts[4].split("=")[1]) # "s=10.500000"
                collisions.append((t, s, obs_id))
            except Exception as e:
                print("Parse error:", line, e)
    return collisions

def load_traversed_points(filename):
    points = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                t = float(parts[0].split("=")[1])
                s = float(parts[1].split("=")[1])
                cost_str = parts[2].split("=")[1]
                cost = float(cost_str) if cost_str != "inf" else float("inf")
                points.append((t, s, cost))
            except Exception as e:
                print("Parse error:", line, e)
    return points

def load_st_constraints(filename):
    t, lower, upper = [], [], []
    with open(filename) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 3:
                continue
            ti, lo, up = map(float, parts)
            t.append(ti)
            lower.append(lo)
            upper.append(up)
    return t, lower, upper

def load_speed_limit(filename):
    s_vals, v_limits = [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 2:
                s, v = map(float, parts)
                s_vals.append(s)
                v_limits.append(v)
    return s_vals, v_limits

def load_dp_speed_limit(filename):
    """加载 DP 导出的限速文件: idx, s, limit_v"""
    s_vals, v_limits = [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 3:  # idx, s, v
                _, s, v = parts
                s_vals.append(float(s))
                v_limits.append(float(v))
    return s_vals, v_limits

if __name__ == "__main__":
    st_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_boundaries.txt"
    dp_speed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_speed.txt"
    qp_speed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/qp_speed.txt"
    sample_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_samples.txt"
    collision_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_collisions.txt"
    traversed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_traversed_points.txt"
    constraint_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_constraints.txt"
    speed_limit_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/speed_limit.txt"
    dp_speed_limit_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_speed_limit.txt"

    obstacles = load_st_boundaries(st_file)
    dp_speed_data = load_speed_profile(dp_speed_file)
    qp_speed_data = load_speed_profile(qp_speed_file)
    st_samples = load_st_samples(sample_file)
    collisions = load_collisions(collision_file)
    traversed = load_traversed_points(traversed_file)
    t_cons, s_lower_cons, s_upper_cons = load_st_constraints(constraint_file)
    s_limit, v_limit = load_speed_limit(speed_limit_file)
    s_limit_dp, v_limit_dp = load_dp_speed_limit(dp_speed_limit_file)

    # === 图1: ST 图 ===
    plt.figure(figsize=(10, 6))

    cmap = mpl.colormaps["tab20b"].resampled(len(obstacles))
    for i, obs in enumerate(obstacles):
        color = cmap(i)
        upper = obs["upper"]
        lower = obs["lower"]
        if not upper or not lower:
            continue
        t_upper, s_upper = zip(*upper)
        t_lower, s_lower = zip(*lower)
        plt.plot(t_upper, s_upper, "-", color=color, alpha=0.8, linewidth=1.2)
        plt.plot(t_lower, s_lower, "-", color=color, alpha=0.8, linewidth=1.2)
        plt.fill_between(t_upper, s_lower, s_upper, color=color, alpha=0.2, label=obs["id"])

    plt.plot(t_cons, s_upper_cons, "k--", linewidth=1.5, label="Constraint Upper")
    plt.plot(t_cons, s_lower_cons, "k--", linewidth=1.5, label="Constraint Lower")
    plt.fill_between(t_cons, s_lower_cons, s_upper_cons, color="green", alpha=0.1, label="ST Feasible Corridor")

    if st_samples:
        t_vals = [p[0] for p in st_samples]
        s_vals = [p[1] for p in st_samples]
        plt.scatter(t_vals, s_vals, s=8, c="black", alpha=0.4, label="ST samples")

    if dp_speed_data:
        t_vals = [p[0] for p in dp_speed_data]
        s_vals = [p[1] for p in dp_speed_data]
        plt.plot(t_vals, s_vals, "g-", linewidth=2, label="DP speed profile")

    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        s_vals = [p[1] for p in qp_speed_data]
        plt.plot(t_vals, s_vals, "m--", linewidth=2, label="QP speed profile")

    if collisions:
        t_vals = [p[0] for p in collisions]
        s_vals = [p[1] for p in collisions]
        plt.scatter(t_vals, s_vals, s=60, c="red", marker="x", label="Collision points")

    if traversed:
        finite_points = [(t, s, c) for (t, s, c) in traversed if c != float("inf")]
        inf_points = [(t, s, c) for (t, s, c) in traversed if c == float("inf")]
        if finite_points:
            t_vals = [p[0] for p in finite_points]
            s_vals = [p[1] for p in finite_points]
            c_vals = [p[2] for p in finite_points]
            sc = plt.scatter(t_vals, s_vals, c=c_vals, cmap="plasma", s=10, alpha=0.8, label="Traversed points")
            plt.colorbar(sc, label="Cost")
        if inf_points:
            t_inf = [p[0] for p in inf_points]
            s_inf = [p[1] for p in inf_points]
            plt.scatter(t_inf, s_inf, s=30, c="red", marker="o", label="Inf cost points")

    plt.xlabel("Time t (s)")
    plt.ylabel("Longitudinal distance s (m)")
    plt.title("ST Graph with DP/QP Profiles, Constraints, Samples & Collisions")
    plt.grid(True)
    plt.legend().set_draggable(True)   # ✅ 图例可拖动
    plt.tight_layout()

    # === 图2: 速度-时间曲线 ===
    plt.figure(figsize=(10, 4))
    if dp_speed_data:
        t_vals = [p[0] for p in dp_speed_data]
        s_vals = [p[1] for p in dp_speed_data]
        v_vals = [p[2] for p in dp_speed_data]
        plt.plot(t_vals, v_vals, "g-", linewidth=2, label="DP speed profile")

        # 插值 DP 限速曲线到时间域
        dp_limit_interp = interp(s_vals, s_limit_dp, v_limit_dp)
        plt.plot(t_vals, dp_limit_interp, "r:", linewidth=1.8, label="DP Speed Limit")

    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        v_vals = [p[2] for p in qp_speed_data]
        plt.plot(t_vals, v_vals, "m--", linewidth=2, label="QP speed profile")

        # === 插值限速线到时间域 ===
        s_vals = [p[1] for p in qp_speed_data]  # QP profile 的 s
        t_vals = [p[0] for p in qp_speed_data]  # QP profile 的 t
        limit_interp = interp(s_vals, s_limit, v_limit)
        plt.plot(t_vals, limit_interp, "r-.", linewidth=1.8, label="Speed Limit")

    plt.xlabel("Time t (s)")
    plt.ylabel("Velocity v (m/s)")
    plt.title("Speed Profile with Speed Limit")
    plt.grid(True)
    plt.legend().set_draggable(True)
    plt.tight_layout()

    # === 图3: QP 加速度-时间曲线 ===
    plt.figure(figsize=(10, 4))
    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        a_vals = [p[3] for p in qp_speed_data]
        plt.plot(t_vals, a_vals, "c-", linewidth=2, label="QP acceleration profile")
    plt.xlabel("Time t (s)")
    plt.ylabel("Acceleration a (m/s^2)")
    plt.title("QP Acceleration Profile")
    plt.grid(True)
    plt.legend().set_draggable(True)   # ✅ 图例可拖动
    plt.tight_layout()

    plt.show()
