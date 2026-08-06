# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib as mpl
plt.rcParams['font.size'] = 18            # 全局字体大小
plt.rcParams['axes.titlesize'] = 25       # 标题字体
plt.rcParams['axes.labelsize'] = 18       # 坐标轴标签
plt.rcParams['xtick.labelsize'] = 20      # x轴刻度
plt.rcParams['ytick.labelsize'] = 20      # y轴刻度
plt.rcParams['legend.fontsize'] = 16      # 图例字体
plt.rcParams['figure.titlesize'] = 25     # 整体标题字体
def load_st_boundaries(filename):
    obstacles = []
    current = None
    mode = None
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("Obstacle"):
                if current:  # 保存上一个障碍物
                    obstacles.append(current)
                current = {"upper": [], "lower": [], "id": line}
            elif line.startswith("# Upper"):
                mode = "upper"
            elif line.startswith("# Lower"):
                mode = "lower"
            elif line[0].isdigit() or line[0] == "-":
                t, s = map(float, line.split())
                current[mode].append((t, s))
        if current:
            obstacles.append(current)
    return obstacles

def load_speed_profile(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if not line or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) == 4:
                t, s, v, a = map(float, parts)
                data.append((t, s, v, a))
    return data

def load_st_samples(filename):
    samples = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 2:
                t, s = map(float, parts)
                samples.append((t, s))
    return samples

def load_collisions(filename):
    collisions = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                obs_id = parts[2].split("=")[1]   # "obstacle=0"
                t = float(parts[3].split("=")[1]) # "t=4.000000"
                s = float(parts[4].split("=")[1]) # "s=10.500000"
                collisions.append((t, s, obs_id))
            except Exception as e:
                print("Parse error:", line, e)
    return collisions

def load_traversed_points(filename):
    points = []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            try:
                t = float(parts[0].split("=")[1])
                s = float(parts[1].split("=")[1])
                cost_str = parts[2].split("=")[1]
                cost = float(cost_str) if cost_str != "inf" else float("inf")
                points.append((t, s, cost))
            except Exception as e:
                print("Parse error:", line, e)
    return points

def load_st_constraints(filename):
    t, lower, upper = [], [], []
    with open(filename) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 3:
                continue
            ti, lo, up = map(float, parts)
            t.append(ti)
            lower.append(lo)
            upper.append(up)
    return t, lower, upper

if __name__ == "__main__":
    st_file = "/home/bob/文档/备份/demo05/src/test/txt/st_boundaries.txt"
    dp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/dp_speed.txt"
    qp_speed_file_low = "/home/bob/文档/备份/demo05/src/test/scripts/low/qp_speed.txt"
    qp_speed_file_up = "/home/bob/文档/备份/demo05/src/test/scripts/up/qp_speed.txt"
    # qp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed.txt"
    qp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_smoothed.txt"
    sample_file = "/home/bob/文档/备份/demo05/src/test/txt/st_samples.txt"
    constraint_file = "/home/bob/文档/备份/demo05/src/test/txt/st_constraints/st_constraints_0.txt"

    obstacles = load_st_boundaries(st_file)
    dp_speed_data = load_speed_profile(dp_speed_file)
    qp_speed_data_low = load_speed_profile(qp_speed_file_low)
    qp_speed_data_up = load_speed_profile(qp_speed_file_up)
    qp_speed_data = load_speed_profile(qp_speed_file)
    st_samples = load_st_samples(sample_file)
    t_cons, s_lower_cons, s_upper_cons = load_st_constraints(constraint_file)

    # === 图1: ST 图 ===
    plt.figure(figsize=(10, 6))

    cmap = mpl.colormaps["tab20b"].resampled(len(obstacles))
    for i, obs in enumerate(obstacles):
        color = cmap(i)
        upper = obs["upper"]
        lower = obs["lower"]
        if not upper or not lower:
            continue
        t_upper, s_upper = zip(*upper)
        t_lower, s_lower = zip(*lower)
        plt.plot(t_upper, s_upper, "-", color=color, alpha=0.8, linewidth=1.2)
        plt.plot(t_lower, s_lower, "-", color=color, alpha=0.8, linewidth=1.2)
        plt.fill_between(t_upper, s_lower, s_upper, color=color, alpha=0.2, label=obs["id"])

    plt.plot(t_cons, s_upper_cons, "k--", linewidth=1.5)
    plt.plot(t_cons, s_lower_cons, "k--", linewidth=1.5)
    plt.fill_between(t_cons, s_lower_cons, s_upper_cons, color="green", alpha=0.1, label="ST Feasible Corridor")

    if st_samples:
        t_vals = [p[0] for p in st_samples]
        s_vals = [p[1] for p in st_samples]
        plt.scatter(t_vals, s_vals, s=8, c="black", alpha=0.4)

    if dp_speed_data:
        t_vals = [p[0] for p in dp_speed_data]
        s_vals = [p[1] for p in dp_speed_data]
        plt.plot(t_vals, s_vals, "g-", linewidth=2, label="DP line")

    if qp_speed_data_low:
        t_vals = [p[0] for p in qp_speed_data_low]
        s_vals = [p[1] for p in qp_speed_data_low]
        plt.plot(t_vals, s_vals, "m--", linewidth=4, label="ST(up)")

    if qp_speed_data_up:
        t_vals = [p[0] for p in qp_speed_data_up]
        v_vals = [p[1] for p in qp_speed_data_up]
        plt.plot(t_vals, v_vals, "b--", linewidth=4, label="ST(low)")

    # if qp_speed_data:
    #     t_vals = [p[0] for p in qp_speed_data]
    #     v_vals = [p[1] for p in qp_speed_data]
    #     plt.plot(t_vals, v_vals, "r--", linewidth=8, label="QP line")

    plt.xlabel("Time t (s)")
    plt.ylabel("Longitudinal distance s (m)")
    plt.title("ST")
    plt.grid(True)
    plt.legend().set_draggable(True)   # ✅ 图例可拖动
    plt.tight_layout()

    # === 图2: 速度-时间曲线 ===
    plt.figure(figsize=(10, 6))

    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        v_vals = [p[2] for p in qp_speed_data]
        plt.plot(t_vals, v_vals, "m--", linewidth=2, label="QP line")

    # 🔹 添加速度上下界虚线
    plt.axhline(y=10, color="blue", linestyle="--", linewidth=4, label="Upper limit")
    plt.axhline(y=0, color="purple", linestyle="--", linewidth=4, label="Lower limit")

    plt.xlabel("Time t (s)")
    plt.ylabel("Velocity v (m/s)")
    plt.title("Speed Profile")
    plt.grid(True)
    plt.legend().set_draggable(True)
    plt.tight_layout()


    # === 图3: QP 加速度-时间曲线 ===
    plt.figure(figsize=(10, 6))
    if qp_speed_data:
        t_vals = [p[0] for p in qp_speed_data]
        a_vals = [p[3] for p in qp_speed_data]
        plt.plot(t_vals, a_vals, "c-", linewidth=2, label="QP acceleration")

    # 🔹 添加加速度上下界虚线
    plt.axhline(y=5, color="blue", linestyle="--", linewidth=4, label="Upper limit")
    plt.axhline(y=-8, color="purple", linestyle="--", linewidth=4, label="Lower limit")

    plt.xlabel("Time t (s)")
    plt.ylabel("Acceleration a (m/s²)")
    plt.title("AT")
    plt.grid(True)
    plt.legend().set_draggable(True)
    plt.tight_layout()

    # === 图4: SL 图 ===
    plt.figure(figsize=(10, 6))
    s_vals = list(range(0, 41))  # s ∈ [0, 40]

    # 🔹 车道线绘制
    # l = -6 贯穿全程
    s_l6 = list(range(0, 41))
    plt.plot(s_l6, [-6] * len(s_l6), linewidth=3, color="tab:blue", label="lane(l=-6 m)")

    # l = -2 分段绘制：0–20 虚线，20–40 实线
    s1 = list(range(0, 21))
    s2 = list(range(20, 41))
    plt.plot(s1, [-2] * len(s1), "k--", linewidth=3, label="lane(l=-2 m, dashed)")
    plt.plot(s2, [-2] * len(s2), "k-", linewidth=3, label="lane(l=-2 m, solid)")

    # l = 2 全程绘制
    plt.plot(s_vals, [2] * len(s_vals), linewidth=3, color="tab:orange", label="lane(l=2 m)")

    plt.plot(list(range(0, 41)), [0] * len(list(range(0, 41))), "b--", linewidth=4, label="Frenet Path (l→-l)")

    # === ✅ 区域填充 ===
    # 1️⃣ s ∈ [0, 20], l ∈ [-6, 2] → 绿色透明带（可换道）
    plt.fill_between(range(0, 21), -6, 2, color="green", alpha=0.15, label="lane change allowed (0–20 m)")

    # 2️⃣ s ∈ [20, 40], l ∈ [-6, 2] → 红色透明带（不可换道）
    plt.fill_between(range(20, 41), -6, 2, color="red", alpha=0.15, label="lane change forbidden (20–40 m)")

    # === 图形属性 ===
    plt.xlabel("Longitudinal distance s (m)")
    plt.ylabel("Lateral offset l (m)")
    plt.title("SL")
    plt.grid(True)
    plt.legend().set_draggable(True)
    plt.tight_layout()
    # === 额外: 从文件读取 Frenet Path 并绘制取反后的轨迹 ===
    # sl_file = "/home/bob/文档/备份/demo05/src/test/txt/sl.txt"

    # s_data, l_data = [], []
    # with open(sl_file, "r") as f:
    #     for line in f:
    #         line = line.strip()
    #         if not line or line.startswith("#"):
    #             continue
    #         parts = line.split()
    #         if len(parts) == 2:
    #             s, l = map(float, parts)
    #             s_data.append(s)
    #             # l_data.append((l-3.82)+2*(-4-(l-3.82))) # ✅ 将 l 取反
    #             l_data.append(l) # ✅ 将 l 取反
    # output_file = "/home/bob/文档/备份/demo05/src/test/scripts/5/sl_mirrored.txt"
    # with open(output_file, "w") as f:
    #     f.write("# Frenet Path (s, l_mirrored)\n")
    #     for s, l in zip(s_data, l_data):
            # f.write(f"{s:.6f} {l:.6f}\n")

    # print(f"✅ 已保存取反后轨迹到 {output_file}")
    # ✅ 在现有 SL 图上绘制取反后的轨迹
    # plt.plot(s_data, l_data, "b--", linewidth=4, label="Frenet Path (l→-l)")
    plt.show()
