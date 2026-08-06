#!/usr/bin/env python3
"""Record runtime channel features from the behavior-identification ROS topics."""

import argparse
import csv
import math
import os
import threading
import time

import rospy
from behavior_identification.msg import VehStateSequenceArray
from road_side_system.msg import TrafficLightPhaseArray


HEADER = [
    "wall_time", "ros_time", "scenario", "expected_behavior",
    "vehicle_id", "history_size", "x", "y", "z", "speed_mps",
    "accel_mps2", "travel_distance_m", "lateral_deviation_m",
    "nearest_stop_distance_m", "nearest_stop_state",
    "st_constraint", "sl_ratio", "accel_ratio", "overspeed_ratio",
]


def finite(value, fallback=0.0):
    value = float(value)
    return value if math.isfinite(value) else fallback


def travel_distance(poses):
    total = 0.0
    for previous, current in zip(poses[:-1], poses[1:]):
        dx = current.position.x - previous.position.x
        dy = current.position.y - previous.position.y
        total += math.hypot(dx, dy)
    return total


def lateral_deviation(poses):
    if len(poses) < 4:
        return 0.0
    start = poses[0].position
    end = poses[-1].position
    dx = end.x - start.x
    dy = end.y - start.y
    length = math.hypot(dx, dy)
    if length < 0.1:
        return 0.0
    return max(
        abs(((pose.position.x - start.x) * dy
             - (pose.position.y - start.y) * dx) / length)
        for pose in poses
    )


class RuntimeRecorder:
    def __init__(self, output_path, scenario, expected_behavior, sample_period):
        self.output_path = output_path
        self.scenario = scenario
        self.expected_behavior = expected_behavior
        self.sample_period = sample_period
        self.last_write = {}
        self.lights = []
        self.lock = threading.Lock()

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        self.handle = open(output_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.handle)
        self.writer.writerow(HEADER)
        self.handle.flush()

        rospy.Subscriber(
            "/traffic_light/phases", TrafficLightPhaseArray,
            self.light_callback, queue_size=10,
        )
        rospy.Subscriber(
            "/veh_state_sequences", VehStateSequenceArray,
            self.vehicle_callback, queue_size=5,
        )

    def light_callback(self, message):
        with self.lock:
            self.lights = list(message.phases)

    def nearest_light(self, point):
        with self.lock:
            lights = list(self.lights)
        if not lights:
            return float("nan"), "none"
        best = min(
            lights,
            key=lambda phase: math.hypot(
                point.x - phase.stop_location.x,
                point.y - phase.stop_location.y,
            ),
        )
        distance = math.hypot(
            point.x - best.stop_location.x,
            point.y - best.stop_location.y,
        )
        return distance, str(best.state).lower()

    def vehicle_callback(self, message):
        now_ros = message.header.stamp.to_sec()
        now_wall = time.time()
        wrote = False
        for vehicle in message.vehicles:
            if not vehicle.pose:
                continue
            previous_write = self.last_write.get(vehicle.id, -1e9)
            if now_ros - previous_write < self.sample_period:
                continue
            self.last_write[vehicle.id] = now_ros

            pose = vehicle.pose[-1]
            speed = finite(vehicle.speed[-1]) if vehicle.speed else 0.0
            accel = finite(vehicle.accel[-1].linear.x) if vehicle.accel else 0.0
            path_length = travel_distance(vehicle.pose)
            lateral = lateral_deviation(vehicle.pose)
            stop_distance, light_state = self.nearest_light(pose.position)

            signal_weight = {
                "red": 1.0, "yellow": 0.65, "green": 0.10,
            }.get(light_state, 0.0)
            st_constraint = (
                signal_weight * math.exp(-stop_distance / 18.0)
                if math.isfinite(stop_distance) else 0.0
            )
            sl_ratio = lateral / 1.5
            accel_ratio = abs(accel) / 3.0
            overspeed_ratio = speed / 15.0

            self.writer.writerow([
                f"{now_wall:.6f}", f"{now_ros:.6f}",
                self.scenario, self.expected_behavior,
                int(vehicle.id), len(vehicle.pose),
                f"{pose.position.x:.6f}", f"{pose.position.y:.6f}",
                f"{pose.position.z:.6f}", f"{speed:.6f}",
                f"{accel:.6f}", f"{path_length:.6f}", f"{lateral:.6f}",
                f"{stop_distance:.6f}" if math.isfinite(stop_distance) else "",
                light_state, f"{st_constraint:.6f}", f"{sl_ratio:.6f}",
                f"{accel_ratio:.6f}", f"{overspeed_ratio:.6f}",
            ])
            wrote = True
        if wrote:
            self.handle.flush()

    def close(self):
        if not self.handle.closed:
            self.handle.flush()
            self.handle.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--expected-behavior", required=True)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--sample-period", type=float, default=0.10)
    args = parser.parse_args()

    rospy.init_node("behavior_runtime_data_recorder", anonymous=True)
    recorder = RuntimeRecorder(
        args.output, args.scenario,
        args.expected_behavior, args.sample_period,
    )
    rospy.loginfo("Runtime recorder writing to %s", args.output)
    try:
        rospy.sleep(args.duration)
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
