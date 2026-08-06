import matplotlib.pyplot as plt

def load_reference_points(filename):
    """加载参考线点 (x, y)"""
    ref_points = []
    with open(filename) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    ref_points.append((x, y))
                except ValueError:
                    continue
    return ref_points


def load_obstacles(filename):
    """加载障碍物多边形点"""
    obstacles = []
    current_obstacle = []
    current_id = None
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("ObstacleID"):
                if current_obstacle:
                    obstacles.append((current_id, current_obstacle))
                    current_obstacle = []
                current_id = line.split(":")[1].strip()
                continue
            if line.startswith("----"):
                if current_obstacle:
                    obstacles.append((current_id, current_obstacle))
                    current_obstacle = []
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    current_obstacle.append((x, y))
                except ValueError:
                    continue
    if current_obstacle:
        obstacles.append((current_id, current_obstacle))
    return obstacles


def plot_scene(ref_points, obstacles):
    plt.figure(figsize=(8, 6))

    # 绘制参考线
    if ref_points:
        xs, ys = zip(*ref_points)
        plt.plot(xs, ys, 'b-', label="Reference Line")

    # 绘制障碍物
    for idx, (obs_id, obs_points) in enumerate(obstacles):
        ox, oy = zip(*obs_points)
        # 闭合多边形
        ox = list(ox) + [ox[0]]
        oy = list(oy) + [oy[0]]
        plt.plot(ox, oy, '-', linewidth=2, label=f"Obstacle {obs_id}")
        plt.fill(ox, oy, alpha=0.3)

        # 在多边形中心标注 ID
        cx = sum(ox[:-1]) / len(obs_points)
        cy = sum(oy[:-1]) / len(obs_points)
        plt.text(cx, cy, f"ID:{obs_id}", fontsize=10, color="red", ha="center")

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Reference Line and Obstacles")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    ref_points = load_reference_points("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/discretized_ref_points.txt")
    obstacles = load_obstacles("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/obstacle_polygon.txt")
    plot_scene(ref_points, obstacles)
