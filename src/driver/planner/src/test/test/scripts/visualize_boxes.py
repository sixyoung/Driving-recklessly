import matplotlib.pyplot as plt
import itertools
import re
import numpy as np

# ========== 读取参考线 ==========
ref_points = []
with open("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/discretized_ref_points.txt", "r") as f:
    lines = f.readlines()
    for line in lines[1:]:  # 跳过第一行 "Reference line"
        parts = line.strip().split()
        if len(parts) >= 2:
            try:
                x, y = float(parts[0]), float(parts[1])
                ref_points.append((x, y))
            except ValueError:
                continue

# ========== 读取障碍物盒子 ==========
obstacles = {}  # key: (obs_id, t), value: list of corners
current_id, current_t = None, None

with open("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/obstacle_box.txt", "r") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("----"):
            continue

        if line.startswith("Obstacle"):
            # 匹配 "Obstacle 18 t=0.1"
            m = re.match(r"Obstacle\s+(\d+)\s+t=([0-9.]+)", line)
            if m:
                current_id = m.group(1)
                current_t = float(m.group(2))
                obstacles[(current_id, current_t)] = []
            continue

        # 否则就是角点
        if current_id is not None and current_t is not None:
            try:
                x, y = map(float, line.split())
                obstacles[(current_id, current_t)].append((x, y))
            except ValueError:
                continue

print("读取到的障碍物数量：", len(obstacles))

# ========= 绘制 ==========
plt.figure(figsize=(12, 10))

# 绘制参考线（黑色虚线）
if ref_points:
    xs, ys = zip(*ref_points)
    plt.plot(xs, ys, "k--", label="Reference Line")

# 在参考线起点画一个自车 box
if ref_points:
    ego_x, ego_y = ref_points[0]
    ego_length, ego_width = 4.5, 2.0
    ego_corners = [
        (ego_x + ego_length/2, ego_y - ego_width/2),
        (ego_x + ego_length/2, ego_y + ego_width/2),
        (ego_x - ego_length/2, ego_y + ego_width/2),
        (ego_x - ego_length/2, ego_y - ego_width/2),
    ]
    ex, ey = zip(*ego_corners)
    ex = list(ex) + [ex[0]]
    ey = list(ey) + [ey[0]]
    plt.plot(ex, ey, "k-", linewidth=2)  # ego 用黑色实线
    plt.text(ego_x, ego_y, "ego_v", fontsize=10, ha="center", va="center", color="black")

# 为每个障碍物分配颜色映射 (colormap)
obs_ids = sorted(set([obs_id for (obs_id, _) in obstacles.keys()]))
cmaps = itertools.cycle([plt.cm.Blues, plt.cm.Greens, plt.cm.Reds, plt.cm.Oranges, plt.cm.Purples])
obs_cmaps = {obs_id: next(cmaps) for obs_id in obs_ids}

# 绘制障碍物盒子
for obs_id in obs_ids:
    times = sorted([t for (oid, t) in obstacles.keys() if oid == obs_id])
    t_min, t_max = min(times), max(times)

    for t in times:
        corners = obstacles[(obs_id, t)]
        if len(corners) < 4:
            continue

        xs, ys = zip(*corners)
        xs = list(xs) + [xs[0]]
        ys = list(ys) + [ys[0]]

        norm_t = (t - t_min) / (t_max - t_min + 1e-6)
        color = obs_cmaps[obs_id](norm_t)
        plt.plot(xs, ys, "-", color=color, alpha=0.8)

    # 在第一个时间点加 ID 标注
    first_corners = obstacles[(obs_id, times[0])]
    fx, fy = zip(*first_corners)
    cx, cy = sum(fx) / len(fx), sum(fy) / len(fy)
    plt.text(cx, cy, f"ID={obs_id}", fontsize=8, ha="center", va="center", color="black")

# ========= 图属性 ==========
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Reference Line, Ego Vehicle and Obstacles")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
