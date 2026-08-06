import matplotlib.pyplot as plt
import numpy as np

def load_planning_inits(filename):
    """读取 Planning Init Point 文件 (x, y, theta)"""
    points = []
    with open(filename) as f:
        frame = {}
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# Planning Init Point"):
                if frame:
                    points.append(frame)
                frame = {}
            else:
                key, val = line.split(maxsplit=1)
                frame[key] = float(val)
        if frame:
            points.append(frame)
    return points

def plot_planning_inits(points):
    plt.figure(figsize=(10, 6))
    cmap = plt.get_cmap("tab10")

    for i, p in enumerate(points):
        x, y, theta = p["x"], p["y"], p["theta"]
        color = cmap(i % 10)

        # 绘制方向箭头
        plt.arrow(x, y,
                  0.5 * np.cos(theta), 0.5 * np.sin(theta),  # 箭头方向
                  head_width=0.1, head_length=0.15,
                  fc=color, ec=color, alpha=0.8)

        # 绘制点
        plt.scatter(x, y, c=color, marker="o", s=40)

        # 标注编号
        plt.text(x, y, f"{i}", fontsize=8, color=color)

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Planning Init Points with Heading")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/txt/planning_init_point.txt"
    points = load_planning_inits(file)
    print(f"Loaded {len(points)} planning init points.")
    plot_planning_inits(points)
