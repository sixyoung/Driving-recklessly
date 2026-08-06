import pandas as pd
import matplotlib.pyplot as plt

# ---------- 1. 配置文件路径 ----------
right_vector_path = "/home/hzq/demo_03/src/tool/right_vectors.csv"
ego_polygon_path = "/home/hzq/demo_03/src/tool/ego_polygon.csv"
path_polygon_path = "/home/hzq/demo_03/src/tool/path_polygon.csv"  # ← 添加路径多边形文件

# ---------- 2. 加载数据 ----------
right_df = pd.read_csv(right_vector_path)   # 路径点及方向向量
ego_df = pd.read_csv(ego_polygon_path)      # 自车轮廓（9点）
path_df = pd.read_csv(path_polygon_path)    # 路径区域多边形

# ---------- 3. 创建图像 ----------
plt.figure(figsize=(12, 9))

# 右方向向量（绿色箭头）
plt.quiver(right_df["x"], right_df["y"], right_df["right_x"], right_df["right_y"],
           angles='xy', scale_units='xy', scale=1, color='green', width=0.003, label='right vector')

# 路径点（红点）
plt.plot(right_df["x"], right_df["y"], 'ro', markersize=3, label='path points')

# 自车轮廓 ego polygon（蓝色 + 点编号）
plt.plot(ego_df["x"], ego_df["y"], 'b-o', label='ego polygon (pt[0]-pt[8])')
for i, (x, y) in enumerate(zip(ego_df["x"], ego_df["y"])):
    plt.text(x, y + 0.3, f"pt[{i}]", fontsize=8, color='blue')

# 路径区域 path polygon（紫色线）
# 路径区域 path polygon（紫色点）
plt.plot(path_df["x"], path_df["y"], 'mo', markersize=4, label='path polygon points')

# ---------- 4. 图像美化 ----------
plt.title("Ego Polygon + Right Vectors + Path Polygon")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

# ---------- 5. 显示 ----------
plt.show()
