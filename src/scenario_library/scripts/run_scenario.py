#!/usr/bin/env python3
"""Launch CARLA scenarios and optional real-time ST/SL/AT visualization."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
WORKSPACE_DIR = PACKAGE_DIR.parent.parent

# Keep the original deployment directories, while also supporting a cloned
# workspace at any path.
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
VIDEO_FPS = 30
VIDEO_SIZE = "1920x1080"
DISPLAY_ID = ":1.0"
FFMPEG_BIN = "/usr/bin/ffmpeg"


def bool_arg(value: bool) -> str:
    return "true" if value else "false"


def clean_old_processes() -> None:
    """Clean residual ROS/CARLA child processes without killing this launcher."""
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
        "realtime_channel_visualizer.py",
        "rostopic pub /carla/hero0/target_speed",
        "roslaunch scenario_library scenario_library.launch",
    ]
    for target in targets:
        subprocess.run(
            ["pkill", "-f", target],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    print("[✅] Cleanup done.\n")


def find_scenario_file(scenario_file: str):
    """Resolve an absolute/relative scenario path or search configured roots."""
    supplied = Path(scenario_file).expanduser()
    if supplied.is_file():
        return supplied.resolve()

    for root in SCENARIO_ROOTS:
        if not root.exists():
            continue
        direct = root / scenario_file
        if direct.is_file():
            return direct.resolve()

    basename = supplied.name
    matches = []
    for root in SCENARIO_ROOTS:
        if root.exists():
            matches.extend(path.resolve() for path in root.rglob(basename) if path.is_file())

    unique_matches = sorted(set(matches))
    if len(unique_matches) == 1:
        return unique_matches[0]
    if len(unique_matches) > 1:
        print(f"[⚠️] Multiple scenarios named {basename} were found:")
        for item in unique_matches:
            print(f"  - {item}")
        print("[提示] 请传入场景文件的完整路径。")
    return None


def list_scenarios() -> None:
    """List all JSON scenario files under the configured roots."""
    print("Available scenarios:\n")
    seen = set()
    for root in SCENARIO_ROOTS:
        if not root.exists():
            continue
        files = sorted(path for path in root.rglob("*.json") if path.is_file())
        if not files:
            continue
        print(f"[DIR] {root}")
        for path in files:
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            try:
                print(f"  - {path.relative_to(root)}")
            except ValueError:
                print(f"  - {path}")
        print()


def show_remaining_processes() -> None:
    print("\n[🔍] Checking remaining ROS/CARLA processes...\n")
    subprocess.run(
        [
            "bash",
            "-lc",
            "ps aux | egrep 'rosmaster|rosout|carla|scenario|driver_model_manager|road_side_system|realtime_channel_visualizer' | grep -v grep || true",
        ],
        check=False,
    )
    print("\n[📊] Done.\n")


def start_recording(scenario_file: str):
    """Record the configured X11 display to MP4."""
    VIDEO_SAVE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    name = Path(scenario_file).stem
    video_path = VIDEO_SAVE_DIR / f"{name}_{timestamp}.mp4"

    print(f"[🎥] Start recording: {video_path}")
    command = [
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
        str(video_path),
    ]

    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=None,
        start_new_session=True,
    )
    return process, video_path


def stop_recording(record_process, video_path) -> None:
    if record_process is None:
        print("[⚠️] Recording process was not started.")
        return

    if record_process.poll() is None:
        print("[🎬] Stopping recording...")
        try:
            if record_process.stdin is not None:
                record_process.stdin.write(b"q")
                record_process.stdin.flush()
            record_process.wait(timeout=8)
        except Exception as exc:  # ffmpeg can close stdin before q is written
            print(f"[⚠️] Failed to stop ffmpeg by q: {exc}")
            try:
                os.killpg(record_process.pid, signal.SIGINT)
                record_process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                print("[⚠️] ffmpeg SIGINT timeout, sending SIGTERM...")
                os.killpg(record_process.pid, signal.SIGTERM)

    if video_path and video_path.exists() and video_path.stat().st_size > 0:
        print(f"[✅] Video saved: {video_path}")
    else:
        print(f"[❌] Video was not created or is empty: {video_path}")


def run_scenario(
    scenario_file: str,
    dynamic_spawn_flag: bool = False,
    max_vehicles: int = 30,
    random_role: str = "",
    record: bool = False,
    visualize_channels: bool = False,
    save_gif: bool = False,
    vehicle_id: int = -1,
    visualized_role: str = "",
    gif_output_dir: str = "~/scenario_gifs",
    show_channel_window: bool = True,
) -> int:
    """Run one scenario and optionally start the four-panel channel monitor."""
    full_path = find_scenario_file(scenario_file)
    if full_path is None:
        print(f"[❌] Scenario file not found: {scenario_file}")
        return 2

    if save_gif:
        visualize_channels = True

    selected_role = visualized_role or random_role
    expanded_gif_dir = str(Path(gif_output_dir).expanduser())

    print(f"[🚀] Running scenario: {full_path.name}")
    print(f"[📂] Full path: {full_path}")
    if visualize_channels:
        target_text = f"ID={vehicle_id}" if vehicle_id >= 0 else (selected_role or "auto-select")
        print(f"[📈] Channel monitor enabled; target={target_text}")
        if save_gif:
            print(f"[GIF] Output directory: {expanded_gif_dir}")
    print()

    command = [
        "roslaunch",
        "scenario_library",
        "scenario_library.launch",
        f"scenario_path:={full_path}",
        f"dynamic_spawn_flag:={bool_arg(dynamic_spawn_flag)}",
        f"max_vehicles:={max_vehicles}",
        f"random_behavior_role_name:={random_role}",
        f"enable_channel_visualizer:={bool_arg(visualize_channels)}",
        f"save_channel_gif:={bool_arg(save_gif)}",
        f"show_channel_window:={bool_arg(show_channel_window)}",
        f"visualized_vehicle_id:={vehicle_id}",
        f"visualized_role_name:={selected_role}",
        f"channel_gif_output_dir:={expanded_gif_dir}",
    ]

    process = None
    record_process = None
    video_path = None
    return_code = 0

    try:
        if record:
            record_process, video_path = start_recording(str(full_path))
            time.sleep(1.0)

        process = subprocess.Popen(command, start_new_session=True)
        return_code = process.wait()
    except KeyboardInterrupt:
        print("\n[🛑] Ctrl+C detected. Stopping scenario...")
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                return_code = process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                print("[⚠️] ROS launch did not stop after SIGINT; sending SIGTERM...")
                os.killpg(process.pid, signal.SIGTERM)
                return_code = process.wait(timeout=5)
    finally:
        if record:
            stop_recording(record_process, video_path)

        print("\n[🧩] Final cleanup...")
        clean_old_processes()
        time.sleep(0.5)
        show_remaining_processes()

    return return_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run CARLA scenarios with optional real-time ST/SL/AT channel visualization."
    )
    parser.add_argument("scenario", nargs="?", help="Scenario JSON name or full path")
    parser.add_argument("--list", action="store_true", help="List available scenarios")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic vehicle spawning")
    parser.add_argument("--max_vehicles", type=int, default=30, help="Maximum spawned vehicles")
    parser.add_argument(
        "--random_role",
        type=str,
        default="",
        help="role_name of the vehicle that performs random behavior",
    )
    parser.add_argument("--record", action="store_true", help="Record the X11 display to MP4")

    parser.add_argument(
        "--visualize_channels",
        action="store_true",
        help="Open the real-time Scenario/ST/SL/AT Matplotlib monitor",
    )
    parser.add_argument(
        "--save_gif",
        action="store_true",
        help="Save the channel monitor as a GIF; also enables channel visualization",
    )
    parser.add_argument(
        "--vehicle_id",
        type=int,
        default=-1,
        help="CARLA actor ID to visualize; default auto-selects the random-behavior vehicle",
    )
    parser.add_argument(
        "--visualized_role",
        type=str,
        default="",
        help="role_name to visualize; defaults to --random_role",
    )
    parser.add_argument(
        "--gif_output_dir",
        type=str,
        default="~/scenario_gifs",
        help="Directory for generated channel GIFs",
    )
    parser.add_argument(
        "--headless_channels",
        action="store_true",
        help="Do not open a Matplotlib window; useful when only saving a GIF",
    )
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

    return run_scenario(
        args.scenario,
        dynamic_spawn_flag=args.dynamic,
        max_vehicles=args.max_vehicles,
        random_role=args.random_role,
        record=args.record,
        visualize_channels=args.visualize_channels,
        save_gif=args.save_gif,
        vehicle_id=args.vehicle_id,
        visualized_role=args.visualized_role,
        gif_output_dir=args.gif_output_dir,
        show_channel_window=not args.headless_channels,
    )


if __name__ == "__main__":
    raise SystemExit(main())
