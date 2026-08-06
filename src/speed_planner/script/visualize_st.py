import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os
import matplotlib
import mplcursors  # 用于交互查看点坐标
import glob

if __name__ == '__main__':
    # === 路径设置 ===
    result_dir = "/home/hzq/demo_03/src/speed_planner/result"

    # === 样式设定 ===
    linewidth_ = 2.5
    plt.rcParams['font.family'] = 'DejaVu Serif'
    plt.rcParams['mathtext.fontset'] = 'stix'  # 数学字体
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    plt.rcParams['axes.linewidth'] = 1.0
    plt.rcParams['axes.grid'] = True
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    # === 创建图形 ===
    fig = plt.figure(figsize=(9, 6))
    ax1 = fig.add_subplot(111)

    # === 读取优化结果并绘制 ===
    opt_data = pd.read_csv(os.path.join(result_dir, "optimization_result.csv"))
    line2, = ax1.plot(opt_data['max_time'], opt_data['max_position'],
                      label="Original", color="black", linewidth=linewidth_)
    line3, = ax1.plot(opt_data['obs_filtered_time'], opt_data['obs_position'],
                      label="Avoidance", color="purple", linewidth=linewidth_)
    line4, = ax1.plot(opt_data['jerk_filtered_time'], opt_data['jerk_position'],
                      label="Jerk Filter", color="orange", linewidth=linewidth_)
    line5, = ax1.plot(opt_data['lp_time'], opt_data['lp_position'],
                      label="LP", color="blue", linewidth=linewidth_)
    # line6, = ax1.plot(opt_data['qp_time'], opt_data['qp_position'],
    #                   label="QP", color="red", linewidth=linewidth_)

    # === 批量读取并绘制多个障碍物轨迹 ===
    obstacle_lines = []
    obs_files = sorted(glob.glob(os.path.join(result_dir, "obs_*.csv")))
    for f in obs_files:
        df = pd.read_csv(f)
        # 自动匹配列名
        tcol = 'obs_time' if 'obs_time' in df.columns else df.columns[0]
        scol = 'obs_s'    if 'obs_s' in df.columns    else df.columns[1]
        label = os.path.splitext(os.path.basename(f))[0]  # obs_0 / obs_1 / ...
        line, = ax1.plot(df[tcol], df[scol],
                         label=label, color="black", linestyle="dashed", linewidth=linewidth_)
        obstacle_lines.append(line)

    # === 坐标轴与图例 ===
    ax1.set_xlabel("t [s]", fontsize=16)
    ax1.set_ylabel("s [m]", fontsize=16)
    ax1.tick_params(labelsize=14)
    ax1.legend(fontsize=14)

    # === 启用悬停交互（障碍线 + 优化线）===
    mplcursors.cursor(obstacle_lines + [line2, line3, line4, line5], hover=True)

    plt.show()
