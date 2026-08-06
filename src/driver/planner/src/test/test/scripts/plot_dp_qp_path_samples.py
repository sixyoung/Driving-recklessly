import matplotlib.pyplot as plt

def load_path(filename):
    """通用路径文件读取 (s, x, y, theta, kappa)"""
    data = []
    with open(filename) as f:
        for line in f:
            if not line or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 5:
                s, x, y, theta, kappa = map(float, parts[:5])
                data.append((s, x, y, theta, kappa))
    return data

def load_frenet_path(filename):
    """读取 Frenet 路径 (s, l)"""
    data = []
    with open(filename) as f:
        for line in f:
            if not line or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                s, l = map(float, parts[:2])
                data.append((s, l))
    return data

def load_path_waypoints(filename):
    """
    支持三种格式:
    - layer_idx point_idx s l x y
    - layer_idx point_idx s l
    - s l
    返回统一格式: (s, l, x, y)，如果 x,y 不存在则为 None
    """
    layers = []
    current_layer = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("----"):
                if current_layer:
                    layers.append(current_layer)
                    current_layer = []
                continue
            parts = line.split()
            if len(parts) >= 6:
                _, _, s, l, x, y = parts
                current_layer.append((float(s), float(l), float(x), float(y)))
            elif len(parts) >= 4:
                _, _, s, l = parts
                current_layer.append((float(s), float(l), None, None))
            elif len(parts) == 2:
                s, l = parts
                current_layer.append((float(s), float(l), None, None))
        if current_layer:
            layers.append(current_layer)
    return layers

def load_vehicle_pos(filename):
    x = y = None
    with open(filename) as f:
        for line in f:
            if line.startswith("position"):
                _, xs, ys, zs = line.strip().split()
                x, y = float(xs), float(ys)
                break
    return (x, y)

def load_planning_init(filename):
    x = y = None
    with open(filename) as f:
        for line in f:
            if line.startswith("x "):
                _, xs = line.strip().split()
                x = float(xs)
            elif line.startswith("y "):
                _, ys = line.strip().split()
                y = float(ys)
    return (x, y)

def load_node_costs(filename):
    """读取节点的cost文件"""
    layers = []
    current_layer = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("----"):
                if current_layer:
                    layers.append(current_layer)
                    current_layer = []
                continue
            parts = line.split()
            kv = {kv.split("=")[0]: kv.split("=")[1] for kv in parts}
            s = float(kv["s"])
            l = float(kv["l"])
            safety = float(kv["safety"])
            smooth = float(kv["smooth"])
            current_layer.append((s, l, safety, smooth))
        if current_layer:
            layers.append(current_layer)
    return layers
def load_last_traj(filename):
    """读取 last_trajectory.txt (x y s v a t)"""
    data = []
    with open(filename) as f:
        for line in f:
            if not line.strip() or line.startswith("#") or line.startswith("Last"):
                continue
            parts = line.strip().split()
            if len(parts) >= 6:
                x, y, s, v, a, t = map(float, parts[:6])
                data.append((x, y, s, v, a, t))
    return data

if __name__ == "__main__":
    # 文件路径
    dp_path_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_path.txt"
    qp_path_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/qp_path.txt"
    ref_path_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/reference_line.txt"
    waypoint_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/path_waypoints.txt"
    vehicle_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/vehicle_odom.txt"
    planning_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/planning_init_point.txt"
    dp_frenet_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/dp_frenet.txt"
    qp_frenet_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/qp_frenet.txt"
    node_cost_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/node_costs.txt"
    history_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/last_trajectory.txt"
    # 读取数据
    dp_path = load_path(dp_path_file)
    qp_path = load_path(qp_path_file)
    history_traj = load_last_traj(history_file)
    ref_path = load_path(ref_path_file)
    path_waypoints = load_path_waypoints(waypoint_file)
    vehicle_pos = load_vehicle_pos(vehicle_file)
    planning_init = load_planning_init(planning_file)
    dp_frenet = load_frenet_path(dp_frenet_file)
    qp_frenet = load_frenet_path(qp_frenet_file)
    node_costs = load_node_costs(node_cost_file)

    # DP Cartesian
    s_dp, x_dp, y_dp, theta_dp = zip(*[(p[0], p[1], p[2], p[3]) for p in dp_path]) if dp_path else ([], [], [], [])

    # QP Cartesian
    s_qp, x_qp, y_qp, theta_qp = zip(*[(p[0], p[1], p[2], p[3]) for p in qp_path]) if qp_path else ([], [], [], [])

    # Reference Line
    s_ref, x_ref, y_ref, theta_ref = zip(*[(p[0], p[1], p[2], p[3]) for p in ref_path]) if ref_path else ([], [], [], [])

    # === 图1: XY 路径 ===
    plt.figure(figsize=(8, 6))
    if ref_path:
        plt.plot(x_ref, y_ref, "k-", linewidth=2, label="Reference Line")
        plt.scatter(x_ref, y_ref, c="black", s=8, marker="o", alpha=0.6)  # 🔹参考线点
    if dp_path:
        plt.plot(x_dp, y_dp, "g-", linewidth=2, label="DP Path")
        plt.scatter(x_dp, y_dp, c="green", s=10, marker="o", alpha=0.6)
    if qp_path:
        plt.plot(x_qp, y_qp, "m--", linewidth=2, label="QP Path")
        plt.scatter(x_qp, y_qp, c="magenta", s=10, marker="o", alpha=0.6)
    # if history_traj:
    #     x_hist = [p[0] for p in history_traj]
    #     y_hist = [p[1] for p in history_traj]
    #     plt.plot(x_hist, y_hist, "navy", linewidth=2, label="History Traj")
    #     plt.scatter(x_hist, y_hist, c="navy", s=8, marker="o", alpha=0.6)
    colors = ["orange", "purple", "cyan", "brown"]
    for idx, layer in enumerate(path_waypoints):
        if not layer:
            continue
        x_layer = [p[2] for p in layer if p[2] is not None]
        y_layer = [p[3] for p in layer if p[3] is not None]
        if x_layer and y_layer:
            plt.scatter(x_layer, y_layer,
                        marker="o", color=colors[idx % len(colors)],
                        s=10, alpha=0.6,
                        label=f"Waypoint Layer {idx}")
    if vehicle_pos[0] is not None:
        plt.scatter(vehicle_pos[0], vehicle_pos[1], c="red", s=60, marker="o", label="Vehicle Position")
    if planning_init[0] is not None:
        plt.scatter(planning_init[0], planning_init[1], c="blue", s=60, marker="x", label="Planning Init")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Reference Line, DP & QP Path with Vehicle & Init Point")
    plt.grid(True)
    plt.legend()

    # === 图2: SL 曲线 ===
    plt.figure(figsize=(10, 6))
    if ref_path:
        plt.plot(s_ref, [0.0]*len(s_ref), "k-", linewidth=2, label="Reference Line (l=0)")
        plt.scatter(s_ref, [0.0]*len(s_ref), c="black", s=8, marker="o", alpha=0.6)  # 🔹参考线点

    for idx, layer in enumerate(node_costs):
        s_layer = [p[0] for p in layer]
        l_layer = [p[1] for p in layer]
        plt.scatter(s_layer, l_layer, s=15, alpha=0.7, label=f"Node Layer {idx}")
        for (s, l, safety, smooth) in layer:
            total = safety + smooth
            plt.text(s, l, f"{total:.1f}", fontsize=6, color="red", ha="center", va="bottom")

    if dp_frenet:
        s_dp_f = [p[0] for p in dp_frenet]
        l_dp_f = [p[1] for p in dp_frenet]
        plt.plot(s_dp_f, l_dp_f, "g-", linewidth=2, label="DP SL")
        plt.scatter(s_dp_f, l_dp_f, c="green", s=10, marker="o", alpha=0.6)
    if qp_frenet:
        s_qp_f = [p[0] for p in qp_frenet]
        l_qp_f = [p[1] for p in qp_frenet]
        plt.plot(s_qp_f, l_qp_f, "m--", linewidth=2, label="QP SL")
        plt.scatter(s_qp_f, l_qp_f, c="magenta", s=10, marker="o", alpha=0.6)
    # if history_traj:
    #     s_hist = [p[2] for p in history_traj]   # s
    #     l_hist = [0.0 for _ in history_traj]    # 默认参考线 l=0
    #     plt.plot(s_hist, l_hist, "navy", linewidth=2, label="History SL")
    #     plt.scatter(s_hist, l_hist, c="navy", s=8, marker="o", alpha=0.6)
    plt.xlabel("s [m]")
    plt.ylabel("l [m]")
    plt.title("Frenet Path (s-l) with Costs")
    plt.grid(True)
    plt.legend()

    # === 图3: θ-s 曲线 ===
    plt.figure(figsize=(8, 4))
    if ref_path:
        plt.plot(s_ref, theta_ref, "k-", linewidth=2, label="Reference θ")
        plt.scatter(s_ref, theta_ref, c="black", s=8, marker="o", alpha=0.6)  # 🔹参考线点
    if dp_path:
        plt.plot(s_dp, theta_dp, "g-", linewidth=2, label="DP θ")
    if qp_path:
        plt.plot(s_qp, theta_qp, "m--", linewidth=2, label="QP θ")
    plt.xlabel("s [m]")
    plt.ylabel("Heading θ [rad]")
    plt.title("Heading along Path")
    plt.grid(True)
    plt.legend()

    plt.show()