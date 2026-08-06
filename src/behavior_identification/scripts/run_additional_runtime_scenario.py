#!/usr/bin/env python3
"""Collect one additional real traffic-interaction scenario for plotting."""

import os
import sys

import carla

sys.path.insert(0, "/home/bob/文档/备份/demo06/src/behavior_identification/scripts")
from run_runtime_channel_experiments import run_one


def main():
    conda_bin = "/home/miniconda3/envs/carla0_9_13/bin"
    carla_api = "/home/bob/CARLA/LinuxNoEditor/PythonAPI/carla"
    carla_egg = carla_api + "/dist/carla-0.9.13-py3.8-linux-x86_64.egg"
    os.environ["PATH"] = conda_bin + ":" + os.environ.get("PATH", "")
    os.environ["PYTHONPATH"] = (
        carla_egg + ":" + carla_api + ":" + os.environ.get("PYTHONPATH", "")
    )
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(20.0)
    rows = run_one(
        client,
        "速度异常随意驾驶辨识测试场景库",
        "非信控路口_路权冲突.json",
        "traffic_conflict",
        16.0,
    )
    print("traffic_conflict rows", rows)


if __name__ == "__main__":
    main()
