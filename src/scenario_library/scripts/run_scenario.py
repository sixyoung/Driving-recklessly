#!/usr/bin/env python3
import os
import subprocess
import argparse
import signal
import time

# ================== 配置 ==================
SCENARIO_DIRS = [
                    "/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/速度异常随意驾驶辨识测试场景库",
                    "/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/加速度异常随意驾驶辨识测试场景库",
                    "/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/换道异常随意驾驶辨识测试场景库",
                ]
VIDEO_SAVE_DIR = "/home/bob/文档/备份/demo06/scenario_videos"
VIDEO_FPS = 30
VIDEO_SIZE = "1920x1080"   # 按你的屏幕分辨率改
DISPLAY_ID = ":1.0"
FFMPEG_BIN = "/usr/bin/ffmpeg"


# ================== 工具函数 ==================
def clean_old_processes():
    """清理残留的 ROS / CARLA 相关进程"""
    print("[🧹] Cleaning up old ROS/CARLA processes...")
    targets = [
        "rosmaster",
        "rosout",
        "carla_ros_bridge",
        "carla_waypoint_publisher",
        "scenario_executor.py",
        "passive_sync_stepper.py",
        "driver_model_manager_node",
        "road_side_system_node",
        "rostopic pub /carla/hero0/target_speed",
        "run_scenario.py",
        "roslaunch"
    ]
    for t in targets:
        os.system(f"pkill -f '{t}' >/dev/null 2>&1")
    print("[✅] Cleanup done.\n")

def find_scenario_file(scenario_file):
    """
    在多个场景目录中查找场景文件
    返回：完整路径
    """
    for d in SCENARIO_DIRS:
        candidate = os.path.join(d, scenario_file)
        if os.path.exists(candidate):
            return candidate
    return None

def list_scenarios():
    """列出所有可用的场景文件（来自所有目录）"""
    print("Available scenarios:\n")
    for d in SCENARIO_DIRS:
        if not os.path.exists(d):
            continue
        print(f"[DIR] {d}")
        for file in os.listdir(d):
            if file.endswith(".json"):
                print(f"  - {file}")
        print()


def show_remaining_processes():
    """显示剩余的 ROS/CARLA 进程"""
    print("\n[🔍] Checking remaining ROS/CARLA processes...\n")
    os.system(
        "ps aux | egrep 'rosmaster|rosout|carla|scenario|driver_model_manager|road_side_system' | grep -v grep"
    )
    print("\n[📊] Done.\n")

def start_recording(scenario_file):
    """启动 ffmpeg 录屏，保存为 mp4"""
    os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    name = os.path.splitext(os.path.basename(scenario_file))[0]
    video_path = os.path.join(VIDEO_SAVE_DIR, f"{name}_{timestamp}.mp4")

    print(f"[🎥] Start recording: {video_path}")

    cmd = [
        FFMPEG_BIN,
        "-y",
        "-video_size", VIDEO_SIZE,
        "-framerate", str(VIDEO_FPS),
        "-f", "x11grab",
        "-i", DISPLAY_ID,
        "-c:v", "libx264",
        "-preset", "veryfast",
        "-pix_fmt", "yuv420p",
        video_path
    ]

    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=None,
        start_new_session=True
    )

    return proc, video_path


def stop_recording(record_proc, video_path):
    """停止 ffmpeg 录屏"""
    if record_proc is None:
        print("[⚠️] Recording process was not started.")
        return

    if record_proc.poll() is None:
        print("[🎬] Stopping recording...")

        try:
            record_proc.stdin.write(b"q")
            record_proc.stdin.flush()
            record_proc.wait(timeout=8)
        except Exception as e:
            print(f"[⚠️] Failed to stop ffmpeg by q: {e}")
            try:
                os.killpg(record_proc.pid, signal.SIGINT)
                record_proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                print("[⚠️] ffmpeg SIGINT timeout, sending SIGTERM...")
                os.killpg(record_proc.pid, signal.SIGTERM)

    if video_path and os.path.exists(video_path) and os.path.getsize(video_path) > 0:
        print(f"[✅] Video saved: {video_path}")
    else:
        print(f"[❌] Video was not created or is empty: {video_path}")
        print("[提示] 请检查 ffmpeg 是否启动成功、DISPLAY 是否正确、保存目录是否存在。")

# ================== 主逻辑 ==================
def run_scenario(scenario_file, dynamic_spawn_flag=False, max_vehicles=30, random_role="", record=False):
    """运行指定场景"""
    full_path = find_scenario_file(scenario_file)
    if full_path is None:
        print(f"[❌] Scenario file not found in any scenario directory: {scenario_file}")
        return
    # 每次运行前清理
    # clean_old_processes()

    print(f"[🚀] Running scenario: {scenario_file}")
    print(f"[📂] Full path: {full_path}\n")

    cmd = [
        "roslaunch", "scenario_library", "scenario_library.launch",
        f"scenario_path:={full_path}",
        f"dynamic_spawn_flag:={str(dynamic_spawn_flag).lower()}",
        f"max_vehicles:={max_vehicles}",
        f"random_behavior_role_name:={random_role}"
    ]

    proc = None
    record_proc = None
    video_path = None

    try:
        if record:
            record_proc, video_path = start_recording(scenario_file)
            time.sleep(1)

        # ✅ 用 Popen 启动，以便手动控制终止
        proc = subprocess.Popen(cmd, start_new_session=True)
        proc.wait()

    except KeyboardInterrupt:
        print("\n[🛑] Ctrl+C detected! Stopping scenario...")

        if proc and proc.poll() is None:
            print("[⚡] Sending SIGINT to process group...")
            os.killpg(proc.pid, signal.SIGINT)

            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[⚠️] SIGINT timeout, sending SIGTERM...")
                os.killpg(proc.pid, signal.SIGTERM)

    finally:
        if record:
            stop_recording(record_proc, video_path)

        print("\n[🧩] Final cleanup...")
        clean_old_processes()
        time.sleep(1)
        show_remaining_processes()


# ================== 主入口 ==================``
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run and manage CARLA scenario simulations.")
    parser.add_argument("scenario", nargs='?', help="Scenario JSON file name (from config/scenarios)")
    parser.add_argument("--list", action="store_true", help="List available scenarios")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic vehicle spawning")
    parser.add_argument("--max_vehicles", type=int, default=30, help="Maximum number of vehicles to spawn")
    parser.add_argument("--random_role", type=str, default="",
                        help="role_name of the vehicle that should perform random_behavior")
    parser.add_argument("--record", action="store_true", help="Record the scenario screen to mp4")
    args = parser.parse_args()

    if args.list:
        list_scenarios()
    elif args.scenario:
        run_scenario(
        args.scenario,
        dynamic_spawn_flag=args.dynamic,
        max_vehicles=args.max_vehicles,
        random_role=args.random_role,
        record=args.record
    )
    else:
        parser.print_help()
