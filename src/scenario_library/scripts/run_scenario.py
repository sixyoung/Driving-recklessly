#!/usr/bin/env python3
"""Launch a CARLA scenario with optional channel visualization and GIF output."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
WORKSPACE_DIR = PACKAGE_DIR.parent.parent
LEGACY_SCENARIO_DIRS = [
    Path("/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/速度异常随意驾驶辨识测试场景库"),
    Path("/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/加速度异常随意驾驶辨识测试场景库"),
    Path("/home/bob/文档/备份/demo06/src/scenario_library/config/scenarios_more/换道异常随意驾驶辨识测试场景库"),
]
SCENARIO_ROOTS = [
    PACKAGE_DIR / "config" / "scenarios",
    PACKAGE_DIR / "config" / "scenarios_more",
    *LEGACY_SCENARIO_DIRS,
]
VIDEO_SAVE_DIR = WORKSPACE_DIR / "scenario_videos"
FFMPEG_BIN = "/usr/bin/ffmpeg"
VIDEO_FPS = 30
VIDEO_SIZE = "1920x1080"
DISPLAY_ID = ":1.0"
ROS_SHUTDOWN_TIMEOUT = 180
MONITOR_START_TIMEOUT = 30


def bool_arg(value: bool) -> str:
    return "true" if value else "false"


def clean_old_processes() -> None:
    print("[🧹] Cleaning residual ROS/CARLA processes...")
    for target in (
        "rosmaster",
        "rosout",
        "carla_ros_bridge",
        "carla_waypoint_publisher",
        "scenario_executor.py",
        "passive_sync_stepper.py",
        "driver_model_manager_node",
        "road_side_system_node",
        "realtime_channel_visualizer",
        "rostopic pub /carla/hero0/target_speed",
        "roslaunch scenario_library scenario_library.launch",
    ):
        subprocess.run(
            ["pkill", "-f", target],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def find_scenario_file(name: str):
    supplied = Path(name).expanduser()
    if supplied.is_file():
        return supplied.resolve()
    for root in SCENARIO_ROOTS:
        direct = root / name
        if direct.is_file():
            return direct.resolve()
    matches = []
    for root in SCENARIO_ROOTS:
        if root.exists():
            matches.extend(path.resolve() for path in root.rglob(supplied.name))
    unique = sorted(set(matches))
    if len(unique) == 1:
        return unique[0]
    if len(unique) > 1:
        print(f"[⚠️] Multiple scenarios named {supplied.name}:")
        for path in unique:
            print(f"  - {path}")
    return None


def list_scenarios() -> None:
    seen = set()
    for root in SCENARIO_ROOTS:
        if not root.exists():
            continue
        print(f"[DIR] {root}")
        for path in sorted(root.rglob("*.json")):
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            print(f"  - {path.relative_to(root)}")


def start_recording(scenario_path: Path):
    VIDEO_SAVE_DIR.mkdir(parents=True, exist_ok=True)
    output = VIDEO_SAVE_DIR / (
        f"{scenario_path.stem}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
    )
    process = subprocess.Popen(
        [
            FFMPEG_BIN,
            "-y",
            "-video_size",
            VIDEO_SIZE,
            "-framerate",
            str(VIDEO_FPS),
            "-f",
            "x11grab",
            "-i",
            DISPLAY_ID,
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-pix_fmt",
            "yuv420p",
            str(output),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        start_new_session=True,
    )
    print(f"[🎥] Recording: {output}")
    return process, output


def stop_recording(process, output) -> None:
    if process is None:
        return
    if process.poll() is None:
        try:
            if process.stdin is not None:
                process.stdin.write(b"q")
                process.stdin.flush()
            process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
    if output and output.exists() and output.stat().st_size > 0:
        print(f"[✅] Video saved: {output}")


def rosnode_names():
    result = subprocess.run(
        ["rosnode", "list"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    return result.stdout.splitlines() if result.returncode == 0 else []


def wait_for_monitor(process: subprocess.Popen) -> bool:
    deadline = time.monotonic() + MONITOR_START_TIMEOUT
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False
        if "/realtime_channel_visualizer" in rosnode_names():
            print("[✅] Channel monitor ROS node is running.")
            return True
        time.sleep(0.5)
    return False


def stop_roslaunch(process: subprocess.Popen) -> int:
    if process.poll() is not None:
        return process.returncode
    print("[GIF] Finalizing lossless recording and palette-optimized GIF...")
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=ROS_SHUTDOWN_TIMEOUT)
    except subprocess.TimeoutExpired:
        print(
            f"[⚠️] GIF finalization exceeded {ROS_SHUTDOWN_TIMEOUT}s; sending SIGTERM."
        )
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return process.wait(timeout=5)


def gif_snapshot(output_dir: Path):
    if not output_dir.exists():
        return set()
    return {path.resolve() for path in output_dir.glob("*_channels.gif")}


def report_gif(output_dir: Path, before) -> None:
    current = gif_snapshot(output_dir)
    new_files = sorted(current - before, key=lambda path: path.stat().st_mtime)
    valid = [path for path in new_files if path.stat().st_size > 128]
    if valid:
        for path in valid:
            print(f"[✅] GIF saved: {path} ({path.stat().st_size / 1024:.1f} KiB)")
        return
    print("[❌] No new valid GIF was produced.")
    for pattern in ("*.part.gif", "*.part.mkv"):
        for path in sorted(output_dir.glob(pattern)) if output_dir.exists() else []:
            print(f"[诊断] Partial file: {path} ({path.stat().st_size} bytes)")


def run_scenario(args) -> int:
    scenario = find_scenario_file(args.scenario)
    if scenario is None:
        print(f"[❌] Scenario not found: {args.scenario}")
        return 2

    visualize = args.visualize_channels or args.save_gif
    role = args.visualized_role or args.random_role
    output_dir = Path(args.gif_output_dir).expanduser().resolve()
    before = gif_snapshot(output_dir) if args.save_gif else set()

    print(f"[🚀] Scenario: {scenario}")
    if visualize:
        print(
            f"[📈] Channel monitor enabled; DISPLAY={os.environ.get('DISPLAY', '<unset>')}"
        )
    if args.save_gif:
        print(f"[GIF] Output directory: {output_dir}")

    command = [
        "roslaunch",
        "scenario_library",
        "scenario_library.launch",
        f"scenario_path:={scenario}",
        f"dynamic_spawn_flag:={bool_arg(args.dynamic)}",
        f"max_vehicles:={args.max_vehicles}",
        f"random_behavior_role_name:={args.random_role}",
        f"enable_channel_visualizer:={bool_arg(visualize)}",
        f"save_channel_gif:={bool_arg(args.save_gif)}",
        f"show_channel_window:={bool_arg(not args.headless_channels)}",
        f"visualized_vehicle_id:={args.vehicle_id}",
        f"visualized_role_name:={role}",
        f"channel_gif_output_dir:={output_dir}",
    ]

    process = None
    record_process = None
    video_path = None
    return_code = 0
    try:
        if args.record:
            record_process, video_path = start_recording(scenario)
            time.sleep(1.0)
        process = subprocess.Popen(command, start_new_session=True)
        if visualize and not wait_for_monitor(process):
            print("[❌] Channel monitor did not start within 30 seconds.")
            print("[修复] catkin_make && source devel/setup.bash")
            return_code = stop_roslaunch(process)
        else:
            return_code = process.wait()
    except KeyboardInterrupt:
        print("\n[🛑] Ctrl+C detected. Please wait for GIF finalization...")
        if process is not None:
            return_code = stop_roslaunch(process)
    finally:
        stop_recording(record_process, video_path)
        if args.save_gif:
            report_gif(output_dir, before)
        clean_old_processes()
    return return_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", nargs="?", help="Scenario JSON name or path")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--max_vehicles", type=int, default=30)
    parser.add_argument("--random_role", default="")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--visualize_channels", action="store_true")
    parser.add_argument("--save_gif", action="store_true")
    parser.add_argument("--vehicle_id", type=int, default=-1)
    parser.add_argument("--visualized_role", default="")
    parser.add_argument("--gif_output_dir", default="~/scenario_gifs")
    parser.add_argument("--headless_channels", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.list:
        list_scenarios()
        return 0
    if not args.scenario:
        parser.print_help()
        return 1
    return run_scenario(args)


if __name__ == "__main__":
    raise SystemExit(main())
