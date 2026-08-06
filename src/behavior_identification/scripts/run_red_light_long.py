#!/usr/bin/env python3
import os
import sys

import carla

sys.path.insert(0, "/home/bob/文档/备份/demo06/src/behavior_identification/scripts")
from run_runtime_channel_experiments import run_one

conda_bin = "/home/miniconda3/envs/carla0_9_13/bin"
carla_api = "/home/bob/CARLA/LinuxNoEditor/PythonAPI/carla"
carla_egg = carla_api + "/dist/carla-0.9.13-py3.8-linux-x86_64.egg"
os.environ["PATH"] = conda_bin + ":" + os.environ.get("PATH", "")
os.environ["PYTHONPATH"] = carla_egg + ":" + carla_api + ":" + os.environ.get("PYTHONPATH", "")

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(20.0)
run_one(
    client,
    "速度异常随意驾驶辨识测试场景库",
    "信控路口_闯红灯.json",
    "red_light_long",
    30.0,
)
