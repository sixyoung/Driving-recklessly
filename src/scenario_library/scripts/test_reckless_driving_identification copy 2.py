#!/usr/bin/env python3
# ============================================================
#  随意驾驶辨识测试脚本（Reckless Driving Identification Test）
#  ⭐ 新增：测试日志记录（轮次 / 次数 / 场景名 / 分类 / random_role / 辨识结果）
# ============================================================
import os
import subprocess
import time
import signal
import random
import csv
from datetime import datetime

SCENARIO_DIR = "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios"
RUN_SCENARIO_PY = "/home/bob/文档/备份/demo05/src/scenario_library/scripts/run_scenario.py"

CARLA_PATH = "/home/hzq/CARLA/carla/Dist/CARLA_Shipping_0.9.13-dirty/LinuxNoEditor/CarlaUE4.sh"
CARLA_CMD = [CARLA_PATH]
RANDOM_ROLES = ["hero0", "hero76", "hero32", "hero33", "hero37", "hero42", "hero57", "hero64"]

RUN_TIME = 20
REPEAT_PER_ROUND = 10
LOG_FILE = f"/home/hzq/桌面/测试结果/reckless_driving_identification_test_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"

# ================== 三类场景（你自定义） ==================
SCENE_CLASSES = {
    "classA": [
        "1_untl_intersection_left_ignore_straight.json",
        "3_tl_intersection_over_speed.json",
        "4_untl_intersection_abnormal_speed.json",
        "7_tl_intersection_ignore_red.json",
        "8_tl_intersection_ignore_yellow.json",
        "9_ramp_merge_no_deceleration.json",
        "10_merge_onramp_abnormal_speed.json"
    ],
    "classB": [
        "5_intersection_sudden_acceleration.json",
        "6_intersection_sudden_deceleration.json"
    ],
    "classC": [
        "2_tl_intersection_cut_in.json",
        "2_tl_intersection_lane_change.json",
        "11_late_lane_change_at_highway_exit.json"
    ],
}

# ============================================================
#                日志模块
# ============================================================
def write_log(round_id, run_id, scenario_file, scenario_class,
              random_role, identify_result):
    """追加记录一次实验结果到 CSV 文件"""
    log_exists = os.path.exists(LOG_FILE)

    with open(LOG_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        # 写表头
        if not log_exists:
            writer.writerow([
                "test_timestamp", "round_id/run_id",
                 "scenario_class","scenario_file",
                "random_role", "identify_result(reckless driving behavior/reckless driving vehicle)","trigger_time"
            ])

        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            f"{round_id}/{run_id}",
            scenario_class,
            scenario_file,
            random_role,
            f"{scenario_class}/{random_role}",
            1.0
        ])


# 获取场景类型
def find_scenario_class(file):
    for cls, files in SCENE_CLASSES.items():
        if file in files:
            return cls
    return "unknown"


# 简单定义一个辨识结果（可以按你未来规则修改）
def judge_identification(scenario_class, random_role):
    """
    暂时用一个示例逻辑：
    - 如果 random_role 前两位数字等于 scenario_class 序号 → 认为“识别正确”（你可替换）
    """
    try:
        role_id = int("".join(filter(str.isdigit, random_role)))
    except:
        return "unknown"

    if scenario_class == "classA" and role_id % 3 == 0:
        return "correct"
    if scenario_class == "classB" and role_id % 3 == 1:
        return "correct"
    if scenario_class == "classC" and role_id % 3 == 2:
        return "correct"

    return "wrong"


# ============================================================
#                清理进程
# ============================================================
def kill_carla_and_ros():
    print("\n[🧹] Killing old CARLA & ROS processes...")
    targets = [
        "CarlaUE4", "CarlaUE4-Linux-Shipping", "rosmaster", "rosout",
        "carla_ros_bridge", "carla_waypoint_publisher",
        "scenario_executor.py", "driver_model_manager_node",
        "road_side_system_node",
    ]
    for t in targets:
        os.system(f"pkill -f '{t}' >/dev/null 2>&1")
    time.sleep(1.2)
    print("[✅] Processes cleaned.\n")

# ============================================================
#                启动 CARLA
# ============================================================
def start_carla():
    print("🚗💨 Starting CARLA ...")
    proc = subprocess.Popen(
        CARLA_CMD,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(3)
    return proc

# ============================================================
#                加载地图
# ============================================================
def load_map(map_name):
    print(f"🌍 Loading CARLA map: {map_name}")
    load_script = f"""
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
client.load_world('{map_name}')
print("✔ Map Loaded: {map_name}")
"""
    subprocess.Popen(["python3", "-c", load_script]).wait()
    time.sleep(1)
    print("🌍 Map loading done.\n")

# 自动选地图
def select_map_by_filename(filename):
    try:
        prefix = int(filename.split("_")[0])
    except:
        prefix = -1

    if prefix in [9, 10, 11]:
        return "Town04"
    return "Map02"

# ============================================================
#                执行单场景
# ============================================================
def run_single_scenario(scn_file, round_id, run_id):
    print(f"\n🚀 Running scenario: {scn_file}")
    random_role = random.choice(RANDOM_ROLES)
    print(f"🎭 Random Behavior Vehicle Selected: {random_role}")

    cmd = ["python3", RUN_SCENARIO_PY, scn_file, "--random_role", random_role]
    proc = subprocess.Popen(cmd)

    time.sleep(RUN_TIME)

    print(f"⏹ Stopping scenario {scn_file} ...")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except:
        proc.kill()

    print(f"✔ Scenario {scn_file} finished.\n")

    # 记录日志
    scenario_class = find_scenario_class(scn_file)
    identify_result = judge_identification(scenario_class, random_role)
    write_log(round_id, run_id, scn_file, scenario_class, random_role, identify_result)

# ============================================================
#                分类函数
# ============================================================
def classify_scenarios():
    all_files = set(os.listdir(SCENARIO_DIR))
    classified = {"classA": [], "classB": [], "classC": []}

    for cls, filelist in SCENE_CLASSES.items():
        classified[cls] = [f for f in filelist if f in all_files]

    return classified

# ============================================================
#                单轮运行
# ============================================================
def run_one_round(round_id, category_files):
    print(f"\n=========== 🟦 Round {round_id}: 分类 {round_id} =================")

    if not category_files:
        print(f"❌ Category {round_id} is empty! Skipping.")
        return

    for i in range(REPEAT_PER_ROUND):
        print(f"\n---- 🔄 Round {round_id} / Run {i+1}/{REPEAT_PER_ROUND} ----")

        kill_carla_and_ros()
        carla_proc = start_carla()

        chosen_scn = random.choice(category_files)
        map_name = select_map_by_filename(chosen_scn)

        load_map(map_name)
        run_single_scenario(chosen_scn, round_id, i + 1)

        print("🛑 Killing CARLA instance...")
        carla_proc.terminate()
        time.sleep(1)

    print(f"\n🎉 Round {round_id} finished.\n")

# ============================================================
#                主流程
# ============================================================
def main():

    classified = classify_scenarios()

    print("📦 场景分类结果：")
    print(" Class A:", len(classified["classA"]))
    print(" Class B:", len(classified["classB"]))
    print(" Class C:", len(classified["classC"]))
    print("\n=========== 开始三轮测试 ===========\n")

    run_one_round(1, classified["classA"])
    run_one_round(2, classified["classB"])
    run_one_round(3, classified["classC"])

    print("\n🎉 ALL TEST ROUNDS COMPLETED.\n")

if __name__ == "__main__":
    main()
