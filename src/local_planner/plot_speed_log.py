import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取数据
df = pd.read_csv("/home/hzq/demo_03/src/local_planner/result/speed_log_hero0.csv")
print("列名：", df.columns.tolist())
print(df.head(3))

# 断点处理：去除含 NaN 的行
df = df.dropna(subset=["time", "target_speed", "current_speed"])

# 准备图像
fig, ax = plt.subplots(figsize=(12, 6))

# 主图
line1, = ax.plot(df['time'], df['target_speed'], label='Target Speed (m/s)', linestyle='--')
line2, = ax.plot(df['time'], df['current_speed'], label='Current Speed (m/s)', linewidth=2)

# 轴设置
ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (m/s)")
ax.set_title("Target vs Current Speed")
ax.grid(True)
ax.legend()
plt.tight_layout()

# 注释框
annot = ax.annotate("", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="w"),
                    arrowprops=dict(arrowstyle="->"))
annot.set_visible(False)

# 数据准备
x_data = df['time'].values
target_speed = df['target_speed'].values
current_speed = df['current_speed'].values
throttle = df['throttle'].values if 'throttle' in df.columns else None
brake = df['brake'].values if 'brake' in df.columns else None

def update_annot(ind, line, label):
    idx = ind["ind"][0]
    x = x_data[idx]
    y = target_speed[idx] if label == "Target" else current_speed[idx]

    annot.xy = (x, y)
    text = f"{label} Speed\nTime: {x:.2f}s\nSpeed: {y:.2f} m/s"
    if label == "Current" and throttle is not None and brake is not None:
        text += f"\nThrottle: {throttle[idx]:.2f}\nBrake: {brake[idx]:.2f}"
    annot.set_text(text)
    annot.get_bbox_patch().set_alpha(0.9)

def hover(event):
    vis = annot.get_visible()
    if event.inaxes == ax:
        for line, label in [(line1, "Target"), (line2, "Current")]:
            cont, ind = line.contains(event)
            if cont and len(ind["ind"]) > 0:  # ✅ 修正关键
                update_annot(ind, line, label)
                annot.set_visible(True)
                fig.canvas.draw_idle()
                return
    if vis:
        annot.set_visible(False)
        fig.canvas.draw_idle()

# 鼠标移动事件绑定
fig.canvas.mpl_connect("motion_notify_event", hover)

plt.show()
