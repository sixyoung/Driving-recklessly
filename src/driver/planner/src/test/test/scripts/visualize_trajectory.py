import matplotlib.pyplot as plt

traj_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/optim_trajectory.txt"

# ========== 读取轨迹 ==========
trajectory = []
with open(traj_file, "r") as f:
    lines = f.readlines()
    for line in lines[1:]:  # 跳过第一行标题
        parts = line.strip().split()
        if len(parts) >= 6:  # 确保有 x y s v a t
            try:
                x = float(parts[0])
                y = float(parts[1])
                s = float(parts[2])
                v = float(parts[3])
                a = float(parts[4])
                t = float(parts[5])
                trajectory.append((x, y, s, v, a, t))
            except ValueError:
                continue

if not trajectory:
    print("No trajectory points found.")
    exit()

xs, ys, ss, vs, accs, ts = zip(*trajectory)

# ========== 1. XY 轨迹 ==========
fig1 = plt.figure(figsize=(7, 6))
plt.plot(xs, ys, "b-", label="Trajectory Path")
plt.scatter(xs, ys, c="r", s=20, label="Trajectory Points")
for i, (x, y, t) in enumerate(zip(xs, ys, ts)):
    plt.text(x, y, f"{i}\n{t:.1f}s", fontsize=6, ha="center", va="bottom")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Optimized Trajectory (XY)")
plt.axis("equal")
plt.legend()
plt.grid(True)

# ========== 2. 弧长 s 曲线 ==========
fig2 = plt.figure(figsize=(7, 6))
plt.plot(ts, ss, "c-", label="Arc Length s")
plt.scatter(ts, ss, c="k", s=10)
for i, (t, s) in enumerate(zip(ts, ss)):
    if i % 5 == 0:
        plt.text(t, s, f"{i}", fontsize=6, ha="center", va="bottom")
plt.xlabel("Time [s]")
plt.ylabel("Arc Length s [m]")
plt.title("Longitudinal Progress (s vs t)")
plt.legend()
plt.grid(True)

# ========== 3. 速度曲线 ==========
fig3 = plt.figure(figsize=(7, 6))
plt.plot(ts, vs, "r-", label="Velocity")
plt.scatter(ts, vs, c="k", s=10)
for i, (t, v) in enumerate(zip(ts, vs)):
    if i % 5 == 0:
        plt.text(t, v, f"{i}", fontsize=6, ha="center", va="bottom")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Velocity Profile")
plt.legend()
plt.grid(True)

# ========== 4. 加速度曲线 ==========
fig4 = plt.figure(figsize=(7, 6))
plt.plot(ts, accs, "g-", label="Acceleration")
plt.scatter(ts, accs, c="k", s=10)
for i, (t, a) in enumerate(zip(ts, accs)):
    if i % 5 == 0:
        plt.text(t, a, f"{i}", fontsize=6, ha="center", va="bottom")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.title("Acceleration Profile")
plt.legend()
plt.grid(True)

# 一次性展示所有窗口
plt.show(block=True)
