import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from scipy.signal import savgol_filter

plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 18,
    'axes.labelsize': 16,
    'xtick.labelsize': 13,
    'ytick.labelsize': 13,
    'legend.fontsize': 14
})

df = pd.read_csv("/home/hzq/demo_03/src/tool/manual_driver_log12121.csv")
df['time'] = df['time'] - df['time'].iloc[0]

def fix_acc(row):
    if row['ego_speed'] <= 1e-3:
        return 0.0
    return max(min(row['ego_acc'], 5.0), -5.0)

df['ego_acc_fixed'] = df.apply(fix_acc, axis=1)

df['ego_speed_smooth'] = savgol_filter(df['ego_speed'], 11, 2)
df['ego_acc_smooth']   = savgol_filter(df['ego_acc_fixed'], 11, 2)
df['obs_speed_smooth'] = savgol_filter(df['obs_speed'].where(df['obs_speed'] > 0).interpolate().bfill(), 11, 2)
df['obs_s_smooth']     = savgol_filter(df['obs_s'].interpolate().bfill(), 11, 2)

fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

axs[0].plot(df['time'], df['ego_speed_smooth'], label='Ego Speed', color='blue', linewidth=2.5)
axs[0].plot(df['time'], df['ego_acc_smooth'], label='Ego Acceleration', color='purple', linestyle='-.', linewidth=2.5)
axs[0].plot(df['time'], df['obs_speed_smooth'], label='Obstacle Speed', color='darkorange', linestyle='--', linewidth=2.5)
axs[0].set_ylabel('Speed / Acc (m/s or m/s²)')
axs[0].set_title('Ego Vehicle Speed, Acceleration and Obstacle Speed')
axs[0].grid(True)

axs[1].plot(df['time'], df['obs_s_smooth'], label='Obstacle Relative Position (s)', color='darkred', linewidth=2.5)
axs[1].set_ylabel('Obstacle s (m)')
axs[1].set_xlabel('Time (seconds)')
axs[1].set_title('Obstacle Relative Position over Time')
axs[1].grid(True)

def plot_phase_interval(ax, df, phase_value, color, label, legend_shown_set):
    is_phase = df['light_state'] == phase_value
    in_block = False
    start_time = None
    for i in range(len(df)):
        if is_phase.iloc[i] and not in_block:
            start_time = df['time'].iloc[i]
            in_block = True
        elif not is_phase.iloc[i] and in_block:
            end_time = df['time'].iloc[i]
            ax.axvspan(start_time, end_time, color=color, alpha=0.2,
                       label=label if label not in legend_shown_set else None)
            legend_shown_set.add(label)
            in_block = False
    if in_block:
        end_time = df['time'].iloc[-1]
        ax.axvspan(start_time, end_time, color=color, alpha=0.2,
                   label=label if label not in legend_shown_set else None)
        legend_shown_set.add(label)

legend_set_ax0 = set()
legend_set_ax1 = set()

plot_phase_interval(axs[0], df, 1, 'red', 'Red Light Phase', legend_set_ax0)
plot_phase_interval(axs[0], df, 3, 'green', 'Green Light Phase', legend_set_ax0)
axs[0].legend(loc='upper left')

plot_phase_interval(axs[1], df, 1, 'red', 'Red Light Phase', legend_set_ax1)
plot_phase_interval(axs[1], df, 3, 'green', 'Green Light Phase', legend_set_ax1)
axs[1].legend(loc='upper left')

plt.tight_layout()
plt.show()
