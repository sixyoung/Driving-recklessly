import subprocess
import os

# 设置脚本目录
SCRIPT_DIR = "/home/hzq/demo_03/src/speed_planner/script"

# 要运行的脚本列表
scripts = [
    "visualize_experiments.py",
    "visualize_st.py",
    "visualize_filtered_velocity.py"
]

# 构造完整路径
script_paths = [os.path.join(SCRIPT_DIR, script) for script in scripts]

# 使用 subprocess 并行运行所有脚本
processes = []
for path in script_paths:
    if os.path.exists(path):
        print(f"[RUNNING] {path}")
        p = subprocess.Popen(["python", path])
        processes.append(p)
    else:
        print(f"[ERROR] Script not found: {path}")

# 等待所有子进程结束
for p in processes:
    p.wait()

print("[DONE] All visualizations completed.")
