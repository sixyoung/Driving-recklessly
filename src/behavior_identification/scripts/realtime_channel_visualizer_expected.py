#!/usr/bin/env python3
"""Expected-channel view: CARLA scene + ST speed + SL route set + AT acceleration."""

from __future__ import annotations

import atexit
import math
import signal
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
import rospy

from behavior_identification.msg import VehStateSequenceArray
from driver_models_types.msg import VehicleConfig
from driver_models_types.srv import PathWithOptionsServiceRequest
from realtime_channel_visualizer_row import Monitor as BaseMonitor, ReferencePath


class ExpectedChannelMonitor(BaseMonitor):
    """Visualize speed, multimodal lateral, and acceleration expected channels."""

    MANEUVER_NAMES = {0: "straight", 1: "left", 2: "right"}

    def __init__(self) -> None:
        # These members are initialized before BaseMonitor subscribes, because
        # ROS callbacks may arrive immediately during the base constructor.
        self.candidate_specs: List[Tuple[str, object, object]] = []
        self.candidate_paths: Dict[str, ReferencePath] = {}
        self.candidate_last_s: Dict[str, Optional[float]] = {}
        self.allowed_maneuvers: Set[str] = {"straight"}
        self.current_sl_route = "unresolved"
        self.candidate_config_id: Optional[int] = None
        self.last_candidate_attempt = 0.0

        super().__init__()

        self.min_speed = float(rospy.get_param("~slow_speed_threshold", 6.0))
        self.include_unknown_lanes = bool(
            rospy.get_param("~sl_include_unknown_other_lanes", True)
        )
        self.path_retry_interval = max(
            float(rospy.get_param("~sl_path_retry_interval", 0.5)), 0.1
        )
        if self.min_speed > self.max_speed:
            rospy.logwarn(
                "slow_speed_threshold %.3f exceeds overspeed_threshold %.3f; swapping them.",
                self.min_speed,
                self.max_speed,
            )
            self.min_speed, self.max_speed = self.max_speed, self.min_speed

        self.draw_waiting()
        self.fig.canvas.draw()

    def draw_waiting(self) -> None:
        titles = (
            "CARLA real scene",
            "ST — speed expected channel",
            "SL — legal-route expected channel",
            "AT — acceleration expected channel",
        )
        for axis, title in zip(
            (self.ax_scene, self.ax_st, self.ax_sl, self.ax_at), titles
        ):
            axis.clear()
            axis.set_title(title)
            axis.set_xticks([])
            axis.set_yticks([])
            axis.text(
                0.5,
                0.5,
                "Waiting for fixed\nreckless-driving vehicle...",
                ha="center",
                va="center",
                transform=axis.transAxes,
            )
        self.ax_speed.clear()
        self.ax_speed.set_visible(False)
        self.fig.suptitle("Waiting — no channel sampling yet")

    # ------------------------------------------------------------------
    # Target and legal-route construction
    # ------------------------------------------------------------------
    def config_cb(self, msg: VehicleConfig) -> None:
        old_config = self.target_config
        super().config_cb(msg)
        if self.target_config is old_config or self.target_config is None:
            return

        with self.lock:
            self.candidate_config_id = int(self.target_config.carla_id)
            self.allowed_maneuvers = self.infer_allowed_maneuvers(self.target_config)
            self.candidate_specs = self.build_candidate_specs(self.target_config)
            self.candidate_paths.clear()
            self.candidate_last_s.clear()
            self.current_sl_route = "waiting for legal routes"
            self.path = None
            self.last_s = None

        rospy.loginfo(
            "SL legal maneuvers for %s: %s; candidate specs=%s",
            self.target_config.role_name,
            "+".join(sorted(self.allowed_maneuvers)),
            [name for name, _, _ in self.candidate_specs],
        )

    @staticmethod
    def quaternion_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @classmethod
    def classify_route(cls, name: str, start_pose, end_pose) -> str:
        text = name.lower().replace("-", "_")
        if any(token in text for token in ("straight", "through", "直行", "直道")):
            return "straight"
        if any(token in text for token in ("left_turn", "turn_left", "左转")):
            return "left"
        if any(token in text for token in ("right_turn", "turn_right", "右转")):
            return "right"

        dx = float(end_pose.position.x - start_pose.position.x)
        dy = float(end_pose.position.y - start_pose.position.y)
        if math.hypot(dx, dy) < 1.0:
            return "unknown"
        yaw = cls.quaternion_yaw(start_pose.orientation)
        signed_angle = math.atan2(
            math.cos(yaw) * dy - math.sin(yaw) * dx,
            math.cos(yaw) * dx + math.sin(yaw) * dy,
        )
        if abs(signed_angle) < math.radians(25.0):
            return "straight"
        return "left" if signed_angle > 0.0 else "right"

    @classmethod
    def infer_allowed_maneuvers(cls, config: VehicleConfig) -> Set[str]:
        # In the current scenario schema road_option is 0=straight, 1=left,
        # 2=right. A left-edge lane admits straight+left and a right-edge lane
        # admits straight+right, as requested. spawn_key text can override the
        # default when a scenario explicitly labels the lane position.
        hint = f"{getattr(config, 'spawn_key', '')} {getattr(config, 'scene_class', '')}".lower()
        if any(token in hint for token in ("leftmost", "left_lane", "最左", "左侧车道")):
            return {"straight", "left"}
        if any(token in hint for token in ("rightmost", "right_lane", "最右", "右侧车道")):
            return {"straight", "right"}

        option = int(getattr(config, "road_option", -1))
        if option == 1:
            return {"straight", "left"}
        if option == 2:
            return {"straight", "right"}
        return {"straight"}

    def build_candidate_specs(self, config: VehicleConfig) -> List[Tuple[str, object, object]]:
        specs: List[Tuple[str, object, object]] = []
        primary = self.MANEUVER_NAMES.get(int(getattr(config, "road_option", -1)), "planned")
        specs.append(
            (f"primary:{primary}", config.spawn_point.pose, config.goal_point.pose)
        )

        seen_names = {specs[0][0]}
        for lane in getattr(config.random_behavior, "other_lanes", []):
            maneuver = self.classify_route(
                str(lane.name), lane.start_point, lane.end_point
            )
            if (
                maneuver not in self.allowed_maneuvers
                and not (maneuver == "unknown" and self.include_unknown_lanes)
            ):
                continue
            label = f"{maneuver}:{lane.name or 'other_lane'}"
            if label in seen_names:
                continue
            seen_names.add(label)
            specs.append((label, lane.start_point, lane.end_point))
        return specs

    def ensure_path(self) -> None:
        with self.lock:
            config = self.target_config
            specs = list(self.candidate_specs)
            existing = set(self.candidate_paths.keys())
        if config is None or not specs:
            return
        if time.monotonic() - self.last_candidate_attempt < self.path_retry_interval:
            return

        missing = next((spec for spec in specs if spec[0] not in existing), None)
        if missing is None:
            return
        self.last_candidate_attempt = time.monotonic()
        name, start_pose, goal_pose = missing

        try:
            rospy.wait_for_service(self.path_service, timeout=0.25)
            request = PathWithOptionsServiceRequest()
            request.role_name = f"{config.role_name}__sl__{name}"
            request.start = start_pose
            request.goal = goal_pose
            response = self.path_client(request)
            if not response.success:
                rospy.logwarn_throttle(
                    2.0, "SL route %s request failed: %s", name, response.message
                )
                return

            points = [
                (wp.pose.position.x, wp.pose.position.y)
                for wp in response.path.waypoints
            ]
            route = ReferencePath(points)
            with self.lock:
                if self.target_config is config:
                    self.candidate_paths[name] = route
                    self.candidate_last_s[name] = None
                    if name.startswith("primary:"):
                        self.path = route
            rospy.loginfo("Loaded SL legal route %s with %d points.", name, len(points))
        except (rospy.ROSException, rospy.ServiceException, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for SL route %s: %s", name, exc)

    # ------------------------------------------------------------------
    # Sampling: project onto every legal route and retain minimum evidence
    # ------------------------------------------------------------------
    def state_cb(self, msg: VehStateSequenceArray) -> None:
        with self.lock:
            target_id = self.target_id
        if target_id is None:
            rospy.loginfo_throttle(
                3.0,
                "Receiving /veh_state_sequences with %d vehicles; waiting for random VehicleConfig.",
                len(msg.vehicles),
            )
            return

        target = next((v for v in msg.vehicles if int(v.id) == target_id), None)
        if target is None or not target.pose:
            rospy.logwarn_throttle(
                3.0,
                "Target %d not in state IDs: %s",
                target_id,
                [int(v.id) for v in msg.vehicles],
            )
            return

        with self.lock:
            stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
            if self.time and stamp <= self.time[-1]:
                stamp = self.time[-1] + 1.0 / self.hz

            speed = self.get_speed(target)
            accel = self.get_accel_no_lock(target, stamp, speed)
            position = target.pose[-1].position

            best_lateral = float("nan")
            best_route = "no legal route loaded"
            for name, route in self.candidate_paths.items():
                try:
                    s_value, lateral = route.project(
                        position.x,
                        position.y,
                        self.candidate_last_s.get(name),
                    )
                    self.candidate_last_s[name] = s_value
                    if not np.isfinite(best_lateral) or abs(lateral) < abs(best_lateral):
                        best_lateral = float(lateral)
                        best_route = name
                except (ValueError, FloatingPointError):
                    continue

            self.current_sl_route = best_route
            first_sample = not self.started
            if first_sample:
                self.started = True

            self.time.append(stamp)
            self.speed.append(speed)
            self.accel.append(accel)
            self.lateral.append(best_lateral)
            # ST no longer means stop-line distance; retain aligned placeholders
            # because BaseMonitor's snapshot expects all buffers to match.
            self.stop_distance.append(float("nan"))
            self.stop_state.append(best_route)
            self.prune_no_lock()
            target_role = self.target_role
            target_id_now = self.target_id

        if first_sample:
            rospy.loginfo(
                "Target appeared in /veh_state_sequences; expected-channel drawing and GIF start now."
            )
            self.start_gif()
        rospy.loginfo_throttle(
            2.0,
            "Channel input %s(%d): speed=%.3f accel=%.3f SL=%.3f route=%s candidates=%d",
            target_role,
            target_id_now,
            speed,
            accel,
            best_lateral,
            best_route,
            len(self.candidate_paths),
        )

    def snapshot(self):
        data = super().snapshot()
        if data is None:
            return None
        with self.lock:
            data["sl_route"] = self.current_sl_route
            data["sl_allowed"] = "+".join(sorted(self.allowed_maneuvers))
            data["sl_candidate_count"] = len(self.candidate_paths)
        return data

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return

        try:
            self.draw_scene()
            self.draw_st_speed(data["t"], data["speed"])
            self.draw_sl_multiroute(
                data["t"],
                data["lateral"],
                data["sl_route"],
                data["sl_allowed"],
                data["sl_candidate_count"],
            )
            self.draw_at_acceleration(data["t"], data["accel"])
            self.fig.suptitle(
                f"Reckless vehicle: {data['target_role']} / ID {data['target_id']}"
            )
            self.fig.canvas.draw()
            frame = np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy()

            now = time.monotonic()
            with self.lock:
                writer = self.gif
            if writer is not None and now - self.last_gif_time >= 1.0 / self.gif_fps:
                writer.write(frame)
                self.last_gif_time = now
        except Exception as exc:
            rospy.logerr_throttle(
                2.0, "Expected-channel draw error (node kept alive): %s", exc
            )
        finally:
            self.pump_gui()

    def draw_st_speed(self, t: np.ndarray, speed: np.ndarray) -> None:
        self.ax_st.clear()
        self.ax_st.set_xlabel("Time (s)")
        self.ax_st.set_ylabel("Speed (m/s)")
        self.ax_st.grid(True, linestyle="--", alpha=0.25)
        self.ax_st.axhspan(
            self.min_speed,
            self.max_speed,
            alpha=0.16,
            label="Expected speed channel",
        )
        self.ax_st.axhline(
            self.min_speed, linestyle="--", linewidth=1.1,
            label=f"Lower {self.min_speed:.1f}",
        )
        self.ax_st.axhline(
            self.max_speed, linestyle="--", linewidth=1.1,
            label=f"Upper {self.max_speed:.1f}",
        )

        valid = np.isfinite(speed)
        if not np.any(valid):
            self.ax_st.set_title("ST — waiting for speed")
            return
        self.ax_st.plot(t[valid], speed[valid], linewidth=2.0, label="Actual speed")
        low = valid & (speed < self.min_speed)
        high = valid & (speed > self.max_speed)
        if np.any(low):
            self.ax_st.scatter(t[low], speed[low], marker="v", s=24, label="Too slow")
        if np.any(high):
            self.ax_st.scatter(t[high], speed[high], marker="^", s=24, label="Too fast")

        current = float(speed[valid][-1])
        state = "TOO SLOW" if current < self.min_speed else (
            "TOO FAST" if current > self.max_speed else "NORMAL"
        )
        self.ax_st.set_title(f"ST — {state} ({current:.2f} m/s)")
        self.ax_st.legend(loc="best", fontsize=7)

    def draw_sl_multiroute(
        self,
        t: np.ndarray,
        lateral: np.ndarray,
        route_name: str,
        allowed: str,
        candidate_count: int,
    ) -> None:
        self.ax_sl.clear()
        self.ax_sl.set_xlabel("Time (s)")
        self.ax_sl.set_ylabel(r"Minimum legal-route offset $\Delta l$ (m)")
        self.ax_sl.grid(True, linestyle="--", alpha=0.25)
        self.ax_sl.axhspan(
            -self.sl_limit,
            self.sl_limit,
            alpha=0.16,
            label="Union of legal SL channels",
        )
        self.ax_sl.axhline(self.sl_limit, linestyle="--", linewidth=1.1)
        self.ax_sl.axhline(-self.sl_limit, linestyle="--", linewidth=1.1)

        valid = np.isfinite(lateral)
        if not np.any(valid):
            self.ax_sl.set_title(
                f"SL — loading legal routes ({candidate_count}) | allowed: {allowed}"
            )
            self.ax_sl.text(
                0.5,
                0.5,
                "No legal reference route loaded yet",
                ha="center",
                va="center",
                transform=self.ax_sl.transAxes,
            )
            return

        self.ax_sl.plot(
            t[valid], lateral[valid], linewidth=2.0,
            label="Nearest legal-route offset",
        )
        abnormal = valid & (np.abs(lateral) > self.sl_limit)
        if np.any(abnormal):
            self.ax_sl.scatter(
                t[abnormal], lateral[abnormal], marker="x", s=28,
                label="Outside all legal routes",
            )

        current = float(lateral[valid][-1])
        state = "VIOLATION" if abs(current) > self.sl_limit else "NORMAL"
        self.ax_sl.set_title(
            f"SL — {state} | nearest: {route_name} | allowed: {allowed}"
        )
        self.ax_sl.legend(loc="best", fontsize=7)

    def draw_at_acceleration(self, t: np.ndarray, accel: np.ndarray) -> None:
        self.ax_at.clear()
        self.ax_speed.clear()
        self.ax_speed.set_visible(False)
        self.ax_at.set_xlabel("Time (s)")
        self.ax_at.set_ylabel("Acceleration (m/s²)")
        self.ax_at.grid(True, linestyle="--", alpha=0.25)
        self.ax_at.axhspan(
            -self.max_accel,
            self.max_accel,
            alpha=0.16,
            label="Expected acceleration channel",
        )
        self.ax_at.axhline(self.max_accel, linestyle="--", linewidth=1.1)
        self.ax_at.axhline(-self.max_accel, linestyle="--", linewidth=1.1)

        valid = np.isfinite(accel)
        if not np.any(valid):
            self.ax_at.set_title("AT — waiting for acceleration")
            return
        self.ax_at.plot(t[valid], accel[valid], linewidth=2.0, label="Actual acceleration")
        abnormal = valid & (np.abs(accel) > self.max_accel)
        if np.any(abnormal):
            self.ax_at.scatter(
                t[abnormal], accel[abnormal], marker="x", s=28,
                label="Outside channel",
            )

        current = float(accel[valid][-1])
        state = "VIOLATION" if abs(current) > self.max_accel else "NORMAL"
        self.ax_at.set_title(f"AT — {state} ({current:.2f} m/s²)")
        self.ax_at.legend(loc="best", fontsize=7)


def main() -> None:
    rospy.init_node("realtime_channel_visualizer", anonymous=False)
    monitor = ExpectedChannelMonitor()
    atexit.register(monitor.close)

    def shutdown_handler(signum, _frame) -> None:
        rospy.loginfo(
            "Signal %d received; finalizing the current GIF before shutdown.", signum
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
