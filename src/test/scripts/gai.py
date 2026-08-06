import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

# === 文件路径 ===
input_file = "/home/bob/文档/备份/demo05/src/test/scripts/5/sl_mirrored.txt"
output_file = "/home/bob/文档/备份/demo05/src/test/scripts/5/sl_mirrored_modified.txt"

# === 读取 Frenet Path 文件 ===
def load_sl_file(filename):
    s_vals, l_vals = [], []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) == 2:
                s, l = map(float, parts)
                s_vals.append(s)
                l_vals.append(l)
    print(f"✅ 载入 Frenet Path，共 {len(s_vals)} 个点")
    return np.array(s_vals), np.array(l_vals)


def save_sl_file(s_ctrl, l_ctrl, offset=0.0, sample_step=0.1, add_zero_start=True):
    """
    保存为均匀采样轨迹。
    若 add_zero_start=True，则拼接 (0,0)，并在 (0,0) 与首点间插值采样。
    """
    # 拼接 (0,0)
    if add_zero_start:
        s_all = np.concatenate(([0.0], s_ctrl))
        l_all = np.concatenate(([0.0], l_ctrl))
        print("🔗 已拼接起点 (0,0)，将对起点至首点间插值采样")
    else:
        s_all, l_all = s_ctrl.copy(), l_ctrl.copy()
        print("🚫 未拼接 (0,0)，使用原首点")

    # 构建样条
    cs = CubicSpline(s_all, l_all)

    # === 均匀采样 s ===
    s_start = 0.0 + offset                     # 起点偏移后的位置
    s_end = s_all[-1] + offset                 # 终点偏移后的位置
    s_sample = np.arange(s_start, s_end + sample_step, sample_step)

    # 注意样条定义域不平移
    l_sample = cs(s_sample - offset)

    # === 写入文件 ===
    with open(output_file, "w") as f:
        f.write(f"# Frenet Path (s, l) - sampled every {sample_step:.2f} m, offset={offset:+.2f}\n")
        f.write(f"# add_zero_start={add_zero_start}\n")
        for s, l in zip(s_sample, l_sample):
            f.write(f"{s:.6f} {l:.6f}\n")

    print(f"💾 已保存采样轨迹到 {output_file}")
    print(f"🧭 起点 s={s_sample[0]:.6f}, l={l_sample[0]:.6f}, 共 {len(s_sample)} 点 (offset={offset:+.2f})")



# === 主程序 ===
if __name__ == "__main__":
    s_vals, l_vals = load_sl_file(input_file)
    add_zero_flag = False
    # 控制点间距，可调整
    CONTROL_POINT_SPACING = 5.0
    sample_idx = [0]
    last_s = s_vals[0]
    for i in range(1, len(s_vals)):
        if s_vals[i] - last_s >= CONTROL_POINT_SPACING:
            sample_idx.append(i)
            last_s = s_vals[i]
    sample_idx = np.array(sample_idx)

    s_ctrl = s_vals[sample_idx]
    l_ctrl = l_vals[sample_idx]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_title(f"可交互编辑 Frenet Path (控制点间距={CONTROL_POINT_SPACING:.1f} m)")
    ax.set_xlabel("s (m)")
    ax.set_ylabel("l (m)")
    ax.grid(True)

    cs = CubicSpline(s_ctrl, l_ctrl)
    s_dense = np.linspace(s_vals.min(), s_vals.max(), 500)
    l_dense = cs(s_dense)
    line, = ax.plot(s_dense, l_dense, "r-", linewidth=2, label="平滑曲线")
    drag_points, = ax.plot(s_ctrl, l_ctrl, "bo", picker=5, label="控制点")
    ax.legend().set_draggable(True)

    selected_index = None

    def recompute_curve():
        """更新平滑曲线"""
        cs = CubicSpline(s_ctrl, l_ctrl)
        s_dense = np.linspace(s_vals.min(), s_vals.max(), 500)
        l_dense = cs(s_dense)
        line.set_data(s_dense, l_dense)
        drag_points.set_data(s_ctrl, l_ctrl)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw_idle()

    def on_pick(event):
        """选中点"""
        global selected_index
        if event.artist != drag_points:
            return
        mx, my = event.mouseevent.xdata, event.mouseevent.ydata
        if mx is None or my is None:
            return
        d = np.sqrt((s_ctrl - mx) ** 2 + (l_ctrl - my) ** 2)
        selected_index = np.argmin(d)
        print(f"🎯 选中点 {selected_index}, s={s_ctrl[selected_index]:.2f}, l={l_ctrl[selected_index]:.2f}")

    def on_motion(event):
        """拖动修改点"""
        global selected_index
        if selected_index is None or event.inaxes != ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        l_ctrl[selected_index] = event.ydata
        recompute_curve()

    def on_release(event):
        """释放"""
        global selected_index
        if selected_index is not None:
            print(f"✅ 释放点 {selected_index}")
        selected_index = None

    def on_key(event):
        """键盘事件"""
        global selected_index, add_zero_flag

        # 编辑控制点
        if selected_index is not None and event.key == "e":
            try:
                new_str = input("输入新坐标 (s,l 或仅 l)：").strip()
                if "," in new_str:
                    s_new, l_new = map(float, new_str.split(","))
                    s_ctrl[selected_index] = s_new
                    l_ctrl[selected_index] = l_new
                else:
                    l_new = float(new_str)
                    l_ctrl[selected_index] = l_new
                recompute_curve()
                print(f"✅ 修改点 {selected_index}: s={s_ctrl[selected_index]:.2f}, l={l_ctrl[selected_index]:.2f}")
            except Exception as e:
                print(f"❌ 输入错误: {e}")

        # 切换拼接模式
        elif event.key.lower() == "p":
            add_zero_flag = not add_zero_flag
            mode = "开启" if add_zero_flag else "关闭"
            print(f"🔁 拼接模式已切换为：{mode} (add_zero_start={add_zero_flag})")

        # 保存采样
        elif event.key.lower() == "s":
            try:
                offset_str = input("输入 s 偏移量 m（默认 0）：").strip()
                offset = float(offset_str) if offset_str else 0.0
            except ValueError:
                offset = 0.0

            save_sl_file(s_ctrl, l_ctrl, offset=offset, sample_step=0.1, add_zero_start=add_zero_flag)


    # === 事件绑定 ===
    fig.canvas.mpl_connect("pick_event", on_pick)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)

    plt.show()
