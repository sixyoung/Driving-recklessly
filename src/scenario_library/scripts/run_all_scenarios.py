#!/usr/bin/env python3
import os
import subprocess
import time
import signal

SCENARIO_DIRS = [
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/速度异常随意驾驶辨识测试场景库",
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/加速度异常随意驾驶辨识测试场景库",
    "/home/bob/文档/备份/demo05/src/scenario_library/config/scenarios_more/换道异常随意驾驶辨识测试场景库",
]
RUN_SCENARIO_PY = "/home/bob/文档/备份/demo05/src/scenario_library/scripts/run_scenario.py"
RUN_TIME = 20  # 每个场景运行时间（秒）

CARLA_PATH = "/home/bob/CARLA/LinuxNoEditor/CarlaUE4.sh"
CARLA_CMD = [CARLA_PATH]
TOWN04_SCENES = {
    "高速公路_下高速时未及时变道.json",
    "高速公路_正常行驶.json",
    "匝道合流_冲突不减速.json",
    "匝道合流_正常行驶.json",
    "匝道行驶_速度过慢.json",
    "匝道行驶_速度正常.json"
}
SCENE_CLASSES = {
    "All Scenarios": [
        "非信控路口_左转不让直行.json",
        "路口_实线换道.json",
        "信控路口_超速行驶.json",
        "信控路口_异常缓慢行驶.json",
        "信控路口_急加速.json",
        "信控路口_急减速.json",
        "信控路口_闯红灯.json",
        "信控路口_黄灯抢行.json",
        "匝道合流_冲突不减速.json",
        "匝道行驶_速度过慢.json",
        "高速公路_下高速时未及时变道.json",
        "非信控路口_路权冲突.json"
    ]
}
DEFAULT_MAP = "Map02"

def kill_carla_and_ros():
    print("\n[🧹] Killing old CARLA & ROS processes...")
    targets = [
        "CarlaUE4",
        "CarlaUE4-Linux-Shipping",
        "rosmaster",
        "rosout",
        "carla_ros_bridge",
        "carla_waypoint_publisher",
        "scenario_executor.py",
        "driver_model_manager_node",
        "road_side_system_node"
    ]
    for t in targets:
        os.system(f"pkill -f '{t}' >/dev/null 2>&1")
    time.sleep(1.2)
    print("[✅] Processes cleaned.\n")


def start_carla():
    print("🚗💨 Starting CARLA ...")
    proc = subprocess.Popen(
        CARLA_CMD,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    print("⏳ Waiting for CARLA to start...")
    time.sleep(3)
    return proc


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
    time.sleep(1)   # 等地图真的加载完
    print("🌍 Map loading done.\n")


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


def select_map_by_filename(filename: str) -> str:
    if filename in TOWN04_SCENES:
        return "Town04"
    return DEFAULT_MAP


def run_single_scenario(scn_file):
    print(f"\n🚀 Running scenario: {scn_file}")

    cmd = ["python3", RUN_SCENARIO_PY, scn_file]
    proc = subprocess.Popen(cmd)

    time.sleep(RUN_TIME)

    print(f"⏹ Stopping scenario {scn_file} ...")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except:
        proc.kill()

    print(f"✔ Scenario {scn_file} finished.\n")


def main():
    scene_list = SCENE_CLASSES["All Scenarios"]

    if not scene_list:
        print("❌ No scenarios defined!")
        return

    print("📦 本轮将运行以下场景：")
    for s in scene_list:
        print(" -", s)

    print("\n============ 开始单轮逐场景测试 ============\n")

    for scn_file in scene_list:
        print(f"\n==============================")
        print(f"🧪 Testing scenario: {scn_file}")
        print(f"==============================\n")

        # 1. 清理旧进程
        kill_carla_and_ros()

        # 2. 启动 CARLA
        carla_proc = start_carla()

        # 3. 选择并加载地图
        map_name = select_map_by_filename(scn_file)
        load_map(map_name)

        # 4. 查找场景文件
        scn_path = find_scenario_file(scn_file)
        if scn_path is None:
            print(f"❌ Scenario not found: {scn_file}")
            carla_proc.terminate()
            continue

        # 5. 运行场景
        run_single_scenario(scn_path)

        # 6. 关闭 CARLA
        print("🛑 Terminating CARLA...")
        carla_proc.terminate()
        time.sleep(1)

    print("\n🎉 单轮所有场景测试完成。\n")


if __name__ == "__main__":
    main()
