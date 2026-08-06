import matplotlib.pyplot as plt

def load_speed_limits(filename):
    """读取 speed_limit.txt 文件，返回 (s, v_limit)"""
    s_vals, v_limits = [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    s_vals.append(float(parts[0]))
                    v_limits.append(float(parts[1]))
                except ValueError:
                    continue
    return s_vals, v_limits


def load_speed_profile(filename):
    """读取速度QP结果文件，返回 (s, v, a)"""
    s_vals, v_vals, a_vals = [], [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    s_vals.append(float(parts[1]))  # s
                    v_vals.append(float(parts[2]))  # v
                    a_vals.append(float(parts[3]))  # a
                except ValueError:
                    continue
    return s_vals, v_vals, a_vals


def plot_speed_info(speed_limit_file, speed_profile_file):
    # 加载限速和QP结果
    s_limit, v_limit = load_speed_limits(speed_limit_file)
    s_qp, v_qp, a_qp = load_speed_profile(speed_profile_file)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- 上图：速度曲线 ---
    ax1.plot(s_limit, v_limit, "r--", linewidth=2, label="Speed Limit")
    ax1.plot(s_qp, v_qp, "b-", linewidth=2, label="QP Speed Profile")
    ax1.set_ylabel("Speed [m/s]")
    ax1.set_title("Speed Limit vs QP Result")
    ax1.grid(True)
    ax1.legend()

    # --- 下图：加速度曲线 ---
    ax2.plot(s_qp, a_qp, "g-", linewidth=2, label="QP Acceleration")
    ax2.set_xlabel("s [m]")
    ax2.set_ylabel("Acceleration [m/s^2]")
    ax2.grid(True)
    ax2.legend()

    plt.show()


if __name__ == "__main__":
    speed_limit_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/speed_limit.txt"
    speed_profile_file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/qp_speed.txt"

    plot_speed_info(speed_limit_file, speed_profile_file)
