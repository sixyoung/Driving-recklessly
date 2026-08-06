#!/usr/bin/env python3
"""Run selected CARLA scenarios and collect real behavior-channel runtime data."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import carla


ROOT = Path("/home/bob/\u6587\u6863/\u5907\u4efd/demo06")
SCENARIO_ROOT = ROOT / "src/scenario_library/config/scenarios_more"
RECORDER = ROOT / "src/behavior_identification/scripts/record_runtime_channel_data.py"
OUTPUT = ROOT / "src/behavior_identification/runtime_results"

SCENARIOS = [
    ("\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u6b63\u5e38\u884c\u9a76.json", "normal"),
    ("\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u95ef\u7ea2\u706f.json", "red_light"),
    ("\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u8d85\u901f\u884c\u9a76.json", "overspeed"),
    ("\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u5f02\u5e38\u7f13\u6162\u884c\u9a76.json", "slow"),
    ("\u52a0\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u6025\u52a0\u901f.json", "aggressive_accel"),
    ("\u52a0\u901f\u5ea6\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u4fe1\u63a7\u8def\u53e3_\u6025\u51cf\u901f.json", "aggressive_decel"),
    ("\u6362\u9053\u5f02\u5e38\u968f\u610f\u9a7e\u9a76\u8fa8\u8bc6\u6d4b\u8bd5\u573a\u666f\u5e93", "\u8def\u53e3_\u5b9e\u7ebf\u6362\u9053.json", "lane_deviation"),
]


def stop_process_group(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def load_clean_world(client):
    client.load_world("Map02")
    time.sleep(3.0)


def run_one(client, directory, filename, expected, duration):
    scenario_path = SCENARIO_ROOT / directory / filename
    if not scenario_path.exists():
        raise FileNotFoundError(scenario_path)

    safe_name = Path(filename).stem
    csv_path = OUTPUT / f"{expected}.csv"
    log_path = OUTPUT / f"{expected}.log"
    recorder_log_path = OUTPUT / f"{expected}_recorder.log"

    load_clean_world(client)
    launch_cmd = [
        "/opt/ros/noetic/bin/roslaunch", "scenario_library", "scenario_library.launch",
        f"scenario_path:={scenario_path}",
        "host:=127.0.0.1", "port:=2000", "town:=Map02",
        "passive:=true", "synchronous_mode:=true",
        "fixed_delta_seconds:=0.02",
    ]
    recorder_cmd = [
        sys.executable, str(RECORDER),
        "--output", str(csv_path),
        "--scenario", safe_name,
        "--expected-behavior", expected,
        "--duration", str(duration),
        "--sample-period", "0.10",
    ]

    print("RUN", expected, scenario_path, flush=True)
    launch = None
    recorder = None
    with open(log_path, "w", encoding="utf-8") as launch_log, open(
        recorder_log_path, "w", encoding="utf-8"
    ) as recorder_log:
        try:
            launch = subprocess.Popen(
                launch_cmd, stdout=launch_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(5.0)
            if launch.poll() is not None:
                raise RuntimeError(f"roslaunch exited early: {launch.returncode}")
            recorder = subprocess.Popen(
                recorder_cmd, stdout=recorder_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            code = recorder.wait(timeout=duration + 15.0)
            if code != 0:
                raise RuntimeError(f"recorder failed: {code}")
        finally:
            stop_process_group(recorder)
            stop_process_group(launch)
            time.sleep(2.0)

    rows = max(sum(1 for _ in open(csv_path, encoding="utf-8")) - 1, 0)
    print("DONE", expected, "rows", rows, flush=True)
    return rows


def main():
    conda_bin = "/home/miniconda3/envs/carla0_9_13/bin"
    carla_api = "/home/bob/CARLA/LinuxNoEditor/PythonAPI/carla"
    carla_egg = carla_api + "/dist/carla-0.9.13-py3.8-linux-x86_64.egg"
    os.environ["PATH"] = conda_bin + ":" + os.environ.get("PATH", "")
    os.environ["PYTHONPATH"] = carla_egg + ":" + carla_api + ":" + os.environ.get("PYTHONPATH", "")

    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=16.0)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()

    OUTPUT.mkdir(parents=True, exist_ok=True)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20.0)
    print("CARLA", client.get_server_version(), client.get_world().get_map().name)

    selected = SCENARIOS[:1] if args.smoke else SCENARIOS
    summary = []
    for entry in selected:
        try:
            rows = run_one(client, *entry, duration=args.duration)
            summary.append((entry[2], rows, "ok"))
        except Exception as exc:
            print("FAILED", entry[2], repr(exc), flush=True)
            summary.append((entry[2], 0, repr(exc)))
    print("SUMMARY", summary)
    if not any(rows > 0 for _, rows, _ in summary):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
