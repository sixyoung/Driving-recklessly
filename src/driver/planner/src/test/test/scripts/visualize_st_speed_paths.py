import matplotlib.pyplot as plt
import matplotlib as mpl

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
                if current:
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
            if not line or line.startswith("#") or line.startswith("="):
                continue
            parts = line.strip().split()
            if len(parts) == 4:
                t, s, v, a = map(float, parts)
                data.append((t, s, v, a))
    return data

def load_traversed_points(filename):
    points = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("="):
                continue
            parts = line.split()
            try:
                t = float(parts[0].split("=")[1])
                s = float(parts[1].split("=")[1])
                cost_str = parts[2].split("=")[1]
                cost = float(cost_str) if cost_str != "inf" else float("inf")
                points.append((t, s, cost))
            except:
                pass
    return points

def load_collisions(filename):
    collisions = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                t = float(parts[3].split("=")[1])
                s = float(parts[4].split("=")[1])
                collisions.append((t, s))
            except:
                pass
    return collisions


if __name__ == "__main__":
    st_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_boundaries.txt"
    dp_speed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_speed.txt"
    qp_speed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/qp_speed.txt"
    traversed_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_traversed_points.txt"
    collision_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/st_collisions.txt"

    # 加载数据
    obstacles = load_st_boundaries(st_file)
    dp_speed_data = load_speed_profile(dp_speed_file)
    qp_speed_data = load_speed_profile(qp_speed_file)
    traversed = load_traversed_points(traversed_file)
    collisions = load_collisions(collision_file)

    # === 绘制 t–s 图 ===
    plt.figure(figsize=(10, 6))

    # 画 ST 边界
    cmap = mpl.colormaps["tab20b"].resampled(len(obstacles))
    for i, obs in enumerate(obstacles):
        color = cmap(i)
        upper = obs["upper"]
        lower = obs["lower"]
        if not upper or not lower:
            continue
        t_upper, s_upper = zip(*upper)
        t_lower, s_lower = zip(*lower)
        plt.fill_between(t_upper, s_lower, s_upper, color=color, alpha=0.2, label=obs["id"])

    # 画 DP 曲线
    if dp_speed_data:
        t_vals = [p[0] for p in dp_speed_data]
        s_vals = [p[1] for p in dp_speed_data]
        plt.plot(t_vals, s_vals, "g-", linewidth=2, label="DP speed profile")

    # 画 QP 曲线
    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        s_vals = [p[1] for p in qp_speed_data]
        plt.plot(t_vals, s_vals, "m--", linewidth=2, label="QP speed profile")

    # 画 Traversed points
    if traversed:
        finite_points = [(t, s, c) for (t, s, c) in traversed if c != float("inf")]
        inf_points = [(t, s) for (t, s, c) in traversed if c == float("inf")]

        if finite_points:
            t_vals = [p[0] for p in finite_points]
            s_vals = [p[1] for p in finite_points]
            c_vals = [p[2] for p in finite_points]
            sc = plt.scatter(t_vals, s_vals, c=c_vals, cmap="plasma", s=10, alpha=0.8, label="Traversed points")
            plt.colorbar(sc, label="Cost")

        if inf_points:
            t_inf, s_inf = zip(*inf_points)
            plt.scatter(t_inf, s_inf, s=30, c="red", marker="o", label="Inf cost points")

    # 画碰撞点
    if collisions:
        t_vals = [p[0] for p in collisions]
        s_vals = [p[1] for p in collisions]
        plt.scatter(t_vals, s_vals, s=60, c="red", marker="x", label="Collision points")

    # 美化
    plt.xlabel("Time t (s)")
    plt.ylabel("Longitudinal distance s (m)")
    plt.title("ST Graph with DP/QP Profiles, Traversed & Collisions")
    plt.grid(True)
    plt.legend().set_draggable(True)
    plt.tight_layout()
    plt.show()
