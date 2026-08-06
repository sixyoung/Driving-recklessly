import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os
import matplotlib
import mplcursors  # 添加用于交互查看点

if __name__ == '__main__':
    os.chdir('../')
    path = os.getcwd()
    data = pd.read_csv("/home/hzq/demo_03/src/speed_planner/result/optimization_result.csv")

    # 设置绘图字体与样式
    plt.rcParams['font.family'] = 'DejaVu Serif'
    plt.rcParams['mathtext.fontset'] = 'stix'  # math fontの设置
    plt.rcParams['xtick.direction'] = 'in'     # x axis in
    plt.rcParams['ytick.direction'] = 'in'     # y axis in
    plt.rcParams['axes.linewidth'] = 1.0       # axis line width
    plt.rcParams['axes.grid'] = True           # make grid
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    # 缩小线宽和图像尺寸以适配屏幕显示
    linewidth_ = 2.5
    fig = plt.figure(figsize=(9, 6))  # 适合普通屏幕显示
    ax = fig.add_subplot(111)

    # 画图
    line1, = ax.plot(data['position'], data['max_velocity'], label="Maximum Velocity", color="black", linewidth=linewidth_)
    line2, = ax.plot(data["position"], data["obs_filtered_velocity"], label="Obstacle Avoidance Velocity", color="purple", linewidth=linewidth_)
    line3, = ax.plot(data["position"], data["jerk_filtered_velocity"], label="Jerk Filtered Velocity", color="orange", linewidth=linewidth_)
    line4, = ax.plot(data["position"], data["lp_velocity"], label="Lp Velocity", color="blue", linewidth=linewidth_)

    # 设置坐标轴与图例
    ax.set_xlabel("s [m]", fontsize=16)
    ax.set_ylabel("velocity [m/s]", fontsize=16)
    ax.tick_params(labelsize=14)
    ax.legend(fontsize=14)

    # 交互式光标显示数值（可选）
    mplcursors.cursor([line1, line2, line3, line4], hover=True)

    # 显示交互式窗口
    plt.show()
