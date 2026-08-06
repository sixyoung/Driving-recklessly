#!/usr/bin/env python3
"""Lane-aware expected-channel monitor with multimodal SL permissions."""

from __future__ import annotations

import atexit
import signal
from typing import Dict, Set

import rospy

from driver_models_types.msg import VehicleConfig
from realtime_channel_visualizer_expected import ExpectedChannelMonitor


class LaneAwareExpectedChannelMonitor(ExpectedChannelMonitor):
    """Infer leftmost/middle/rightmost lane before defining legal maneuvers."""

    def __init__(self) -> None:
        # Protect against a VehicleConfig callback arriving during the parent
        # constructor before its ROS parameters have been loaded.
        self.include_unknown_lanes = True
        self.path_retry_interval = 0.5
        super().__init__()

    @staticmethod
    def scalar_parameters(config: VehicleConfig) -> Dict[str, float]:
        keys = list(getattr(config.random_behavior, "keys", []))
        values = list(getattr(config.random_behavior, "values", []))
        return {
            str(key).strip().lower(): float(value)
            for key, value in zip(keys, values)
        }

    @staticmethod
    def adjacency_hints(config: VehicleConfig):
        has_left_neighbor = False
        has_right_neighbor = False
        for lane in getattr(config.random_behavior, "other_lanes", []):
            name = str(lane.name).strip().lower().replace("-", "_")
            if any(token in name for token in (
                "left_lane", "lane_left", "left_adjacent", "adjacent_left",
                "左侧车道", "左车道", "左邻车道",
            )):
                has_left_neighbor = True
            if any(token in name for token in (
                "right_lane", "lane_right", "right_adjacent", "adjacent_right",
                "右侧车道", "右车道", "右邻车道",
            )):
                has_right_neighbor = True
        return has_left_neighbor, has_right_neighbor

    @classmethod
    def infer_allowed_maneuvers(cls, config: VehicleConfig) -> Set[str]:
        params = cls.scalar_parameters(config)

        if params.get("is_leftmost_lane", 0.0) > 0.5:
            return {"straight", "left"}
        if params.get("is_rightmost_lane", 0.0) > 0.5:
            return {"straight", "right"}

        # Optional numeric convention in scenario JSON:
        # lane_position = -1 leftmost, 0 middle, +1 rightmost.
        if "lane_position" in params:
            position = params["lane_position"]
            if position <= -0.5:
                return {"straight", "left"}
            if position >= 0.5:
                return {"straight", "right"}
            return {"straight"}

        has_left_neighbor, has_right_neighbor = cls.adjacency_hints(config)
        if has_right_neighbor and not has_left_neighbor:
            # Only a lane on the right means the current lane is leftmost.
            return {"straight", "left"}
        if has_left_neighbor and not has_right_neighbor:
            # Only a lane on the left means the current lane is rightmost.
            return {"straight", "right"}
        if has_left_neighbor and has_right_neighbor:
            return {"straight"}

        # Fall back to spawn-key/road-option inference implemented by the
        # parent monitor when lane-position evidence is unavailable.
        return super().infer_allowed_maneuvers(config)


def main() -> None:
    rospy.init_node("realtime_channel_visualizer", anonymous=False)
    monitor = LaneAwareExpectedChannelMonitor()
    atexit.register(monitor.close)

    def shutdown_handler(signum, _frame) -> None:
        rospy.loginfo(
            "Signal %d received; finalizing GIF before scenario shutdown.", signum
        )
        monitor.close()
        if not rospy.is_shutdown():
            rospy.signal_shutdown(f"signal {signum}")

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        monitor.spin()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
