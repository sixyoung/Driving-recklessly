import matplotlib.pyplot as plt
import numpy as np

def order_corners(corners):
    # 计算中心点
    cx = sum(x for x, _ in corners) / len(corners)
    cy = sum(y for _, y in corners) / len(corners)

    # 按角度排序
    corners_sorted = sorted(corners, key=lambda p: np.arctan2(p[1] - cy, p[0] - cx))
    return corners_sorted

def load_reference_line(filename):
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

def load_obstacles(filename):
    obstacles = {}
    current_id = None
    corners = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# Obstacle ID:"):
                if current_id and corners:
                    obstacles[current_id] = corners
                    corners = []
                current_id = line.split(":")[1].strip()
            elif line.startswith("----"):
                if current_id and corners:
                    obstacles[current_id] = corners
                    corners = []
            else:
                parts = line.split()
                if len(parts) >= 2:
                    x, y = map(float, parts[:2])
                    corners.append((x, y))
        if current_id and corners:
            obstacles[current_id] = corners
    return obstacles

# 文件路径
ref_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/reference_line.txt"
obs_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/static_obstacle_polygon.txt"

# 加载数据
ref_path = load_reference_line(ref_file)
obstacles = load_obstacles(obs_file)

# 绘图
plt.figure(figsize=(10, 6))

# 绘制参考线
if ref_path:
    xs = [p[1] for p in ref_path]
    ys = [p[2] for p in ref_path]
    plt.plot(xs, ys, "k-", linewidth=2, label="Reference Line")

# 绘制障碍物
for obs_id, corners in obstacles.items():
    corners = order_corners(corners)
    xs = [p[0] for p in corners] + [corners[0][0]]
    ys = [p[1] for p in corners] + [corners[0][1]]
    plt.plot(xs, ys, "r-")
    plt.fill(xs, ys, alpha=0.3)
    # 标注障碍物ID
    cx = sum(x for x, _ in corners) / len(corners)
    cy = sum(y for _, y in corners) / len(corners)
    plt.text(cx, cy, obs_id, fontsize=8, ha="center", va="center", color="blue")

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Reference Line with Static Obstacles")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
