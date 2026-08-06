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
import threading
import rospy
from std_msgs.msg import String
import csv
from datetime import datetime


SCENARIO_DIRS = [
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/速度异常随意驾驶辨识测试场景库",
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/加速度异常随意驾驶辨识测试场景库",
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/换道异常随意驾驶辨识测试场景库",
]
RUN_SCENARIO_PY = "/home/bob/文档/备份/demo05/src/scenario_library/scripts/run_scenario.py"

CARLA_PATH = "/home/hzq/CARLA/carla/Dist/CARLA_Shipping_0.9.13-dirty/LinuxNoEditor/CarlaUE4.sh"
CONTEXT_FILE = "/home/hzq/桌面/文字稿/latest_context.csv"
CARLA_CMD = [CARLA_PATH]

RUN_TIME = 20
REPEAT_PER_ROUND = 10
DETECTION_FILE = "/home/hzq/桌面/测试结果/latest_detection.csv"
LOG_FILE = f"/home/hzq/桌面/测试结果/reckless_driving_identification_test_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"

# ================== 三类场景（你自定义） ==================
SCENE_CLASSES = {
    "Speed Anomaly": [
        "非信控路口_左转不让直行.json",
        "信控路口_超速行驶.json",
        "信控路口_闯红灯.json",
        "信控路口_黄灯抢行.json",
        "信控路口_异常缓慢行驶.json",
        "信控路口_正常行驶.json",
        "匝道合流_冲突不减速.json",
        "匝道合流_正常行驶.json",
        "匝道行驶_速度过慢.json",
        "匝道行驶_速度正常.json"
    ],
    "Acceleration Anomaly": [
        "信控路口_急加速.json",
        "信控路口_急减速.json",
        "信控路口_加速度正常.json"
    ],
    "Lane Change Anomaly": [
        "高速公路_下高速时未及时变道.json",
        "高速公路_正常行驶.json",
        "多车道_加塞.json",
        "路口_实线换道.json",
        "路口_正常行驶.json"
    ]
}

TOWN04_SCENES = {
    "高速公路_下高速时未及时变道.json",
    "高速公路_正常行驶.json",
    "匝道合流_冲突不减速.json",
    "匝道合流_正常行驶.json",
    "匝道行驶_速度过慢.json",
    "匝道行驶_速度正常.json"
}

DEFAULT_MAP = "Map02"

# ============================================================
#                日志模块
# ============================================================
def judge_identification_success(
    scenario_class,
    random_role,
    detected_scene,
    detected_role
):
    """
    判断是否辨识成功
    """
    # === 情况 1：正常场景 ===
    if scenario_class == "正常场景":
        if detected_scene == "未识别随意驾驶场景":
            return "是"
        else:
            return "否"

    # === 情况 2：非正常（随意驾驶）场景 ===
    else:
        if (
            detected_scene == scenario_class and
            detected_role == random_role
        ):
            return "是"
        else:
            return "否"


def write_log(
    round_id,
    run_id,
    scenario_file,
    scenario_class,
    random_role,
    spawn_position,   # ⭐ 新增：初始位置
    speed,
    distance,
    scene,   # 辨识场景类型
    role,    # 辨识随意驾驶车辆
    trigger
):
    new_file = not os.path.exists(LOG_FILE)

    # === 计算是否辨识成功 ===
    identify_result = judge_identification_success(
        scenario_class=scenario_class,
        random_role=random_role,
        detected_scene=scene,
        detected_role=role
    )

    with open(LOG_FILE, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        if new_file:
            writer.writerow([
                "时间",
                "轮次",
                "场景文件",
                "场景类型",
                "随意驾驶车辆",
                "初始位置",          # ⭐ 新增
                "目标速度",
                "变速/换道位置",
                "辨识场景类型",
                "辨识随意驾驶车辆",
                "辨识触发时间",
                "是否辨识成功",
            ])

        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            f"{round_id}/{run_id}",
            scenario_file,
            scenario_class,
            random_role,
            spawn_position,        # ⭐ 新增
            speed,
            distance,
            scene,
            role,
            trigger,
            identify_result,
        ])


# 获取场景类型
def parse_location(loc_str):
    """
    解析 location 字段
    输入: "x=-116.0,y=-63.2,z=8.0" 或 "无"
    输出: (x, y, z) 或 None
    """
    if not loc_str or loc_str.strip() == "无":
        return None

    try:
        parts = loc_str.split(",")
        kv = {}
        for p in parts:
            k, v = p.split("=")
            kv[k.strip()] = float(v)

        return kv.get("x"), kv.get("y"), kv.get("z")
    except Exception:
        return None


def find_scenario_class(_=None):
    if not os.path.exists(CONTEXT_FILE):
        return "unknown", None, None, None, None

    try:
        with open(CONTEXT_FILE, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)

            # 只有表头或空文件
            if len(rows) <= 1:
                return "unknown", None, None, None, None

            last = rows[-1]

            # 列顺序:
            # 0 timestamp
            # 1 scene_class
            # 2 random_role
            # 3 speed
            # 4 distance
            # 5 location

            scene_class = last[1].strip() if len(last) > 1 else "unknown"
            random_role = last[2].strip() if len(last) > 2 else None
            speed = last[3].strip() if len(last) > 3 else None
            distance = last[4].strip() if len(last) > 4 else None
            location_str = last[5].strip() if len(last) > 5 else None

            location = parse_location(location_str)

            return scene_class, random_role, speed, distance, location

    except Exception as e:
        print(f"[find_scenario_class] error: {e}")
        return "unknown", None, None, None, None



# ============================================================
#                清理进程
# ============================================================
def kill_carla_and_ros():
    print("\n[🧹] Killing old CARLA & ROS processes...")
    targets = [
        "CarlaUE4", "CarlaUE4-Linux-Shipping",
        "carla_ros_bridge", "carla_waypoint_publisher",
        "scenario_executor.py", "driver_model_manager_node",
        "road_side_system_node",
    ]
    for t in targets:
        os.system(f"pkill -f '{t}' >/dev/null 2>&1")
    try:
        time.sleep(1.2)
    except KeyboardInterrupt:
        # ✅ 关键：清理阶段不再向外抛
        pass
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
def select_map_by_filename(filename: str) -> str:
    if filename in TOWN04_SCENES:
        return "Town04"
    return DEFAULT_MAP

def get_latest_detection():
    if not os.path.exists(DETECTION_FILE):
        return None, None, None

    try:
        with open(DETECTION_FILE, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)

            # 只有表头或空文件
            if len(rows) <= 1:
                return None, None, None

            last = rows[-1]

            # timestamp, detection_1, detection_2, detection_3
            detected_scene = last[1].strip() if len(last) > 1 else None
            detected_role = last[2].strip() if len(last) > 2 else None
            detected_value = last[3].strip() if len(last) > 3 else None

            # 统一 none → None
            if detected_scene == "none":
                detected_scene = None
            if detected_role == "none":
                detected_role = None
            if detected_value == "none":
                detected_value = None

            return detected_scene, detected_role, detected_value

    except Exception as e:
        print(f"[get_latest_detection] error: {e}")
        return None, None, None



# ============================================================
#                执行单场景
# ============================================================
def run_single_scenario(scn_file, round_id, run_id, random_role=None):

    print(f"\n🚀 Running scenario: {scn_file}")
    print(f"🎭 Random Behavior Vehicle Selected: {random_role}")

    scn_path = find_scenario_file(scn_file)
    if scn_path is None:
        print(f"[❌] Scenario file not found: {scn_file}")
        return

    cmd = ["python3", RUN_SCENARIO_PY, scn_path, "--random_role", random_role]
    proc = subprocess.Popen(cmd)

    time.sleep(RUN_TIME)

    print(f"⏹ Stopping scenario {scn_file} ...")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except:
        proc.kill()

    # 获取 ROS 识别结果
    scene, role, trigger = get_latest_detection()
    # 写日志
    scenario_class, random_role1, speed, distance, location = find_scenario_class(scn_file)

    write_log(round_id, run_id, scn_file, scenario_class, random_role1, location, speed, distance, scene, role, trigger)

# ============================================================
#                分类函数
# ============================================================
def find_scenario_file(scn_file: str):
    """
    在多个场景目录中查找场景文件
    返回完整路径，找不到返回 None
    """
    for d in SCENARIO_DIRS:
        candidate = os.path.join(d, scn_file)
        if os.path.exists(candidate):
            return candidate
    return None

def classify_scenarios():
    all_files = set()

    for d in SCENARIO_DIRS:
        if not os.path.exists(d):
            continue
        for f in os.listdir(d):
            if f.endswith(".json"):
                all_files.add(f)

    classified = {
        "Speed Anomaly": [],
        "Acceleration Anomaly": [],
        "Lane Change Anomaly": []
    }

    for cls, filelist in SCENE_CLASSES.items():
        classified[cls] = [f for f in filelist if f in all_files]

    return classified


# ============================================================
#                单轮运行
# ============================================================
def run_one_round(round_id,
                  category_A_files,
                  category_B_files,
                  prob_A=0.5):

    print(f"\n=========== 🟦 Round {round_id} =================")

    if not category_A_files and not category_B_files:
        print("❌ Both categories are empty! Skipping.")
        return

    for i in range(REPEAT_PER_ROUND):
        print(f"\n---- 🔄 Round {round_id} / Run {i+1}/{REPEAT_PER_ROUND} ----")

        kill_carla_and_ros()
        carla_proc = start_carla()

        # ================= 核心：按概率选类别 =================
        r = random.random()  # [0,1)
        random_role = f"hero{random.randint(100, 500)}"
        if r < prob_A and category_A_files:
            chosen_scn = random.choice(category_A_files)
        else:
            # 如果 B 为空，则回退到 A
            if category_B_files:
                chosen_scn = random.choice(category_B_files)
            else:
                chosen_scn = random.choice(category_A_files)

        # ================= 原有逻辑保持不变 =================
        map_name = select_map_by_filename(chosen_scn)
        load_map(map_name)

        run_single_scenario(chosen_scn, round_id, i + 1, random_role)

        print("🛑 Killing CARLA instance...")
        carla_proc.terminate()
        time.sleep(1)

    print(f"\n🎉 Round {round_id} finished.\n")

# ============================================================
#                主流程
# ============================================================
def main():
    try:
        classified = classify_scenarios()

        print("📦 场景分类结果：")
        print(" Speed Anomaly:", len(classified["Speed Anomaly"]))
        print(" Acceleration Anomaly:", len(classified["Acceleration Anomaly"]))
        print(" Lane Change Anomaly:", len(classified["Lane Change Anomaly"]))
        print("\n=========== 开始三轮测试 ===========\n")

        run_one_round(1, classified["Speed Anomaly"], classified["Speed Anomaly"], 0.5)
        run_one_round(2, classified["Acceleration Anomaly"], classified["Acceleration Anomaly"], 0.5)
        run_one_round(3, classified["Lane Change Anomaly"], classified["Lane Change Anomaly"], 0.5)

        print("\n🎉 ALL TEST ROUNDS COMPLETED.\n")

    except KeyboardInterrupt:
        print("\n🛑 [INTERRUPTED] User interrupted the test (Ctrl+C).")

    except Exception as e:
        print(f"\n❌ [ERROR] Unexpected exception: {e}")

    finally:
        print("\n🧹 [FINAL CLEANUP] Cleaning CARLA & ROS processes...")
        kill_carla_and_ros()
        print("✅ Cleanup done. Exiting safely.\n")

if __name__ == "__main__":
    main()
