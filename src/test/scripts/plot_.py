import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
# === 原始数据加载函数 ===
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
                if current:
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
def save_to_file():
        """保存当前 s(t), v(t), a(t) 数据为文件"""
        global t_vals, s_vals
        output_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_modified.txt"

        # --- 平滑导数计算 ---
        s_smooth = savgol_filter(s_vals, window_length=51, polyorder=3)
        v_vals = np.gradient(s_smooth, t_vals)
        v_smooth = savgol_filter(v_vals, window_length=51, polyorder=3)
        a_vals = np.gradient(v_smooth, t_vals)
        a_smooth = savgol_filter(a_vals, window_length=51, polyorder=3)

        # --- 保存 ---
        with open(output_file, "w") as f:
            f.write("# Modified Speed Profile (t, s, v, a)\n")
            for ti, si, vi, ai in zip(t_vals, s_smooth, v_smooth, a_smooth):
                f.write(f"{ti:.6f} {si:.6f} {vi:.6f} {ai:.6f}\n")

        print(f"💾 已保存到 {output_file}")
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


# === 主程序 ===
if __name__ == "__main__":
    st_file = "/home/bob/文档/备份/demo05/src/test/txt/st_boundaries.txt"
    dp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/dp_speed.txt"
    qp_speed_file_low = "/home/bob/文档/备份/demo05/src/test/scripts/low/qp_speed.txt"
    qp_speed_file_up = "/home/bob/文档/备份/demo05/src/test/scripts/up/qp_speed.txt"
    qp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_smoothed.txt"
    # qp_speed_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_modified.txt"
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
    fig_st, ax_st = plt.subplots(figsize=(10, 6))
    cmap = mpl.colormaps["tab20b"].resampled(len(obstacles))
    for i, obs in enumerate(obstacles):
        color = cmap(i)
        upper = obs["upper"]
        lower = obs["lower"]
        if not upper or not lower:
            continue
        t_upper, s_upper = zip(*upper)
        t_lower, s_lower = zip(*lower)
        ax_st.plot(t_upper, s_upper, "-", color=color, alpha=0.8, linewidth=1.2)
        ax_st.plot(t_lower, s_lower, "-", color=color, alpha=0.8, linewidth=1.2)
        ax_st.fill_between(t_upper, s_lower, s_upper, color=color, alpha=0.2, label=obs["id"])

    ax_st.plot(t_cons, s_upper_cons, "k--", linewidth=1.5, label="Constraint Upper")
    ax_st.plot(t_cons, s_lower_cons, "k--", linewidth=1.5, label="Constraint Lower")
    ax_st.fill_between(t_cons, s_lower_cons, s_upper_cons, color="green", alpha=0.1, label="ST Feasible Corridor")

    if st_samples:
        t_vals = [p[0] for p in st_samples]
        s_vals = [p[1] for p in st_samples]
        ax_st.scatter(t_vals, s_vals, s=8, c="black", alpha=0.4, label="ST samples")

    if dp_speed_data:
        t_vals = [p[0] for p in dp_speed_data]
        s_vals = [p[1] for p in dp_speed_data]
        ax_st.plot(t_vals, s_vals, "g-", linewidth=2, label="DP speed profile")

    if qp_speed_data_low:
        t_vals = [p[0] for p in qp_speed_data_low]
        s_vals = [p[1] for p in qp_speed_data_low]
        ax_st.plot(t_vals, s_vals, "m--", linewidth=4, label="QP speed profile (up)")

    if qp_speed_data_up:
        t_vals = [p[0] for p in qp_speed_data_up]
        v_vals = [p[1] for p in qp_speed_data_up]
        ax_st.plot(t_vals, v_vals, "b--", linewidth=4, label="QP speed profile (low)")

    if qp_speed_data:
        t_vals = np.array([p[0] for p in qp_speed_data])
        s_vals = np.array([p[1] for p in qp_speed_data])
        line_st, = ax_st.plot(t_vals, s_vals, "r--", linewidth=8, label="QP ST curve")

    ax_st.set_xlabel("Time t (s)")
    ax_st.set_ylabel("Longitudinal distance s (m)")
    ax_st.set_title("ST Graph with DP/QP Profiles, Constraints, Samples")
    ax_st.grid(True)
    ax_st.legend().set_draggable(True)
    plt.tight_layout()

    # === 图2、图3 初始化 ===
    fig_v, ax_v = plt.subplots(figsize=(10, 4))
    fig_a, ax_a = plt.subplots(figsize=(10, 4))

    # === 初次计算平滑导数 ===
    # 先对 s 进行平滑，再计算 v, a
    s_smooth = savgol_filter(s_vals, window_length=21, polyorder=3)
    v_vals = np.gradient(s_smooth, t_vals)
    v_smooth = savgol_filter(v_vals, window_length=21, polyorder=3)
    a_vals = np.gradient(v_smooth, t_vals)
    a_smooth = savgol_filter(a_vals, window_length=21, polyorder=3)

    line_v, = ax_v.plot(t_vals, v_smooth, "m--", linewidth=2, label="Velocity (smoothed)")
    line_a, = ax_a.plot(t_vals, a_smooth, "c-", linewidth=2, label="Acceleration (smoothed)")

    for ax, title, ylab in [(ax_v, "Velocity Profile (QP)", "v (m/s)"),
                            (ax_a, "Acceleration Profile (QP)", "a (m/s²)")]:
        ax.set_xlabel("Time t (s)")
        ax.set_ylabel(ylab)
        ax.set_title(title)
        ax.grid(True)
        ax.legend().set_draggable(True)
        plt.tight_layout()

    # === 💡 添加交互功能 ===
    # 每隔约0.5m生成一个可拖拽点
    avg_ds = np.mean(np.diff(s_vals))
    step = max(1, int(1.0 / avg_ds))
    sample_idx = np.arange(0, len(s_vals), step)
    drag_t = t_vals[sample_idx].copy()
    drag_s = s_vals[sample_idx].copy()
    drag_points, = ax_st.plot(drag_t, drag_s, "bo", picker=5)

    # 标签用于显示坐标
    label_text = ax_st.text(0.02, 0.95, "", transform=ax_st.transAxes,
                            va="top", fontsize=10, color="blue",
                            bbox=dict(boxstyle="round", fc="w", alpha=0.7))

    selected_index = None  # 当前选中点索引

    def recompute_curves():
        """重新计算速度和加速度（以 Δt=0.5s 均匀采样计算）"""
        global t_vals, s_vals

        # === 1️⃣ 对 s(t) 建立样条模型 ===
        cs = CubicSpline(t_vals, s_vals)

        # === 2️⃣ 生成等间隔采样点 ===
        t_resampled = np.arange(t_vals[0], t_vals[-1], 0.1)

        # === 3️⃣ 计算位置、速度、加速度 ===
        s_resampled = cs(t_resampled)
        v_resampled = cs.derivative(1)(t_resampled)
        a_resampled = cs.derivative(2)(t_resampled)

        # === 4️⃣ 可选平滑 ===
        from scipy.signal import savgol_filter
        v_smooth = savgol_filter(v_resampled, window_length=21, polyorder=3)
        a_smooth = savgol_filter(a_resampled, window_length=21, polyorder=3)

        # === 5️⃣ 更新可视化曲线 ===
        line_v.set_data(t_resampled, v_smooth)
        line_a.set_data(t_resampled, a_smooth)

        for ax in [ax_v, ax_a]:
            ax.relim()
            ax.autoscale_view()

        fig_v.canvas.draw_idle()
        fig_a.canvas.draw_idle()

        # === 打印信息以验证 ===
        print(f"✅ Δt = 0.5s, 样本点数 = {len(t_resampled)}")
        print(f"   平均速度范围: [{v_smooth.min():.2f}, {v_smooth.max():.2f}] m/s")
        print(f"   平均加速度范围: [{a_smooth.min():.2f}, {a_smooth.max():.2f}] m/s²")


    def on_pick(event):
        global selected_index
        if event.artist != drag_points:
            return
        mouse_x, mouse_y = event.mouseevent.xdata, event.mouseevent.ydata
        if mouse_x is None or mouse_y is None:
            return
        d = np.sqrt((drag_t - mouse_x) ** 2 + (drag_s - mouse_y) ** 2)
        selected_index = np.argmin(d)
        label_text.set_text(f"Selected: t={drag_t[selected_index]:.2f}, s={drag_s[selected_index]:.2f}")
        fig_st.canvas.draw_idle()

    def on_motion(event):
        global selected_index, drag_t, drag_s, s_vals
        if selected_index is None or event.inaxes != ax_st:
            return
        if event.xdata is None or event.ydata is None:
            return
        drag_s[selected_index] = event.ydata  # 纵向拖动
        cs = CubicSpline(drag_t, drag_s)
        s_vals = cs(t_vals)
        line_st.set_data(t_vals, s_vals)
        drag_points.set_data(drag_t, drag_s)
        label_text.set_text(f"(t={drag_t[selected_index]:.2f}, s={drag_s[selected_index]:.2f})")
        ax_st.figure.canvas.draw_idle()
        recompute_curves()

    def on_release(event):
        global selected_index
        if selected_index is not None:
            print(f"✅ Released point {selected_index}")
        selected_index = None
        label_text.set_text("")
        fig_st.canvas.draw_idle()

    # === 🔤 新功能：键盘输入修改坐标 ===
# === 🔤 更新 on_key 支持保存 ===
    def on_key(event):
        global selected_index, drag_t, drag_s, s_vals
        # === 编辑模式 ===
        if selected_index is not None and event.key == "e":
            print("\n🟢 编辑模式：输入新坐标")
            print("👉 仅输入单个数值修改 s，例如：12.5")
            print("👉 或输入两个值修改 t,s，例如：3.2, 15.7")
            try:
                new_str = input("输入新坐标 (s 或 t,s)：").strip()
                if "," in new_str:
                    t_new, s_new = map(float, new_str.split(","))
                    drag_t[selected_index] = t_new
                    drag_s[selected_index] = s_new
                else:
                    s_new = float(new_str)
                    drag_s[selected_index] = s_new

                # 更新插值曲线
                cs = CubicSpline(drag_t, drag_s)
                s_vals = cs(t_vals)
                line_st.set_data(t_vals, s_vals)
                drag_points.set_data(drag_t, drag_s)
                label_text.set_text(f"(t={drag_t[selected_index]:.2f}, s={drag_s[selected_index]:.2f})")

                recompute_curves()
                fig_st.canvas.draw_idle()
                print(f"✅ 点 {selected_index} 修改为 (t={drag_t[selected_index]:.2f}, s={drag_s[selected_index]:.2f})")

            except Exception as e:
                print(f"❌ 输入错误: {e}")

        # === 保存模式 ===
        elif event.key.lower() == "s":
            save_to_file()

    # === 事件绑定 ===
    fig_st.canvas.mpl_connect("pick_event", on_pick)
    fig_st.canvas.mpl_connect("motion_notify_event", on_motion)
    fig_st.canvas.mpl_connect("button_release_event", on_release)
    fig_st.canvas.mpl_connect("key_press_event", on_key)

    plt.show()
