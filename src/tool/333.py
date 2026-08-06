import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from scipy.signal import savgol_filter

# === 0. 全局字体设置 ===
plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 18,
    'axes.labelsize': 16,
    'xtick.labelsize': 13,
    'ytick.labelsize': 13,
    'legend.fontsize': 14
})

# === 1. 加载数据 ===
df = pd.read_csv("/home/hzq/demo_03/src/tool/manual_driver_log_hero0.csv")
df['time'] = df['time'] - df['time'].iloc[0]

# === 2. 修正加速度 ===
def fix_acc(row):
    if row['ego_speed'] <= 1e-3:
        return 0.0
    return max(min(row['ego_acc'], 5.0), -5.0)

df['ego_acc_fixed'] = df.apply(fix_acc, axis=1)

# === 3. 滤波处理 ===
df['ego_speed_smooth'] = savgol_filter(df['ego_speed'], 11, 2)
df['ego_acc_smooth']   = savgol_filter(df['ego_acc_fixed'], 11, 2)
df['obs_speed_smooth'] = savgol_filter(df['obs_speed'].where(df['obs_speed'] > 0).interpolate().bfill(), 11, 2)
df['obs_s_smooth']     = savgol_filter(df['obs_s'].interpolate().bfill(), 11, 2)

# === 4. 创建图像和子图 ===
fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# === 第一子图：速度 / 加速度 ===
axs[0].plot(df['time'], df['ego_speed_smooth'], label='Ego Speed', color='blue', linewidth=2.5)
axs[0].plot(df['time'], df['ego_acc_smooth'], label='Ego Acceleration', color='purple', linestyle='-.', linewidth=2.5)
axs[0].plot(df['time'], df['obs_speed_smooth'], label='Obstacle Speed', color='darkorange', linestyle='--', linewidth=2.5)
axs[0].set_ylabel('Speed / Acc (m/s or m/s²)')
axs[0].set_title('Ego Vehicle Speed, Acceleration and Obstacle Speed')
axs[0].grid(True)

# === 第二子图：障碍物位置（仅绘制有值部分） ===
mask_valid = df['obs_s'].notna()
axs[1].plot(df['time'][mask_valid], df['obs_s_smooth'][mask_valid],
            label='Obstacle Relative Position (s)', color='darkred', linewidth=2.5)
axs[1].set_ylabel('Obstacle s (m)')
axs[1].set_xlabel('Time (seconds)')
axs[1].set_title('Obstacle Relative Position over Time')
axs[1].grid(True)

# === 仅绘制让行阶段的阴影 ===
def plot_yield_phase(ax, df, already_plotted):
    is_yield = df['yield_flag'] == 1
    in_block = False
    start_time = None
    for i in range(len(df)):
        if is_yield.iloc[i] and not in_block:
            start_time = df['time'].iloc[i]
            in_block = True
        elif not is_yield.iloc[i] and in_block:
            end_time = df['time'].iloc[i]
            ax.axvspan(start_time, end_time, color='gold', alpha=0.2,
                       label='Yielding Phase' if not already_plotted[0] else None)
            already_plotted[0] = True
            in_block = False
    if in_block:
        end_time = df['time'].iloc[-1]
        ax.axvspan(start_time, end_time, color='gold', alpha=0.2,
                   label='Yielding Phase' if not already_plotted[0] else None)

already_yield_plotted = [False]
for ax in axs:
    plot_yield_phase(ax, df, already_yield_plotted)

# === 添加图例（仅第一子图）===
axs[0].legend(loc='upper left')

# === 布局优化 & 展示 ===
plt.tight_layout()
plt.show()
