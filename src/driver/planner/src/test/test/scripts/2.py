import matplotlib.pyplot as plt

def load_trajectory(filename):
    """读取轨迹文件 (x, y)"""
    xs, ys = [], []
    with open(filename) as f:
        for line in f:
            if (line.startswith("#") or 
                line.startswith("Optimized") or 
                line.startswith("Prev") or 
                line.startswith("Last")):
                continue
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    xs.append(float(parts[0]))
                    ys.append(float(parts[1]))
                except ValueError:
                    continue
    return xs, ys

def load_vehicle_odom(filename):
    """读取 vehicle_odom.txt (x, y)"""
    x = y = None
    with open(filename) as f:
        for line in f:
            if line.startswith("position") or line.startswith("position:"):
                _, xs, ys, zs = line.strip().split()
                x, y = float(xs), float(ys)
                break
    return (x, y)

def plot_trajectories(last_x, last_y, prev_x, prev_y,
                      ego_pos, last_odom=None, prev_odom=None):
    plt.figure(figsize=(8, 6))

    # --- 上上帧轨迹 ---
    if prev_x and prev_y:
        plt.plot(prev_x, prev_y, "g--", linewidth=2, label="Prev Prev Trajectory")
        plt.scatter(prev_x, prev_y, c="green", s=10, alpha=0.6)

    # --- 上一帧轨迹 ---
    if last_x and last_y:
        plt.plot(last_x, last_y, "b-", linewidth=2, label="Last Trajectory")
        plt.scatter(last_x, last_y, c="blue", s=10, alpha=0.6)

    # --- 当前自车位置 ---
    if ego_pos[0] is not None:
        plt.scatter(ego_pos[0], ego_pos[1], c="red", s=80, marker="*", label="Ego Vehicle")

    # --- 上一帧自车位置 ---
    if last_odom and last_odom[0] is not None:
        plt.scatter(last_odom[0], last_odom[1], c="blue", s=80, marker="x", label="Last Ego Odom")

    # --- 上上帧自车位置 ---
    if prev_odom and prev_odom[0] is not None:
        plt.scatter(prev_odom[0], prev_odom[1], c="green", s=80, marker="x", label="Prev Prev Ego Odom")

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Last & Prev Trajectories with Ego Vehicle Positions")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    last_traj_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/last_trajectory.txt"
    prev_traj_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/prev_prev_trajectory.txt"
    ego_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/vehicle_odom.txt"
    last_odom_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/last_vehicle_odom.txt"
    prev_odom_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/prev_prev_vehicle_odom.txt"

    last_x, last_y = load_trajectory(last_traj_file)
    prev_x, prev_y = load_trajectory(prev_traj_file)
    ego_pos = load_vehicle_odom(ego_file)
    last_odom = load_vehicle_odom(last_odom_file)
    prev_odom = load_vehicle_odom(prev_odom_file)

    plot_trajectories(last_x, last_y, prev_x, prev_y, ego_pos, last_odom, prev_odom)
