#!/usr/bin/env python3
"""Real-time four-panel visualizer for scenario, ST, SL and AT channels.

The SL channel uses Frenet lateral displacement relative to the frozen initial
route returned by /carla_waypoint_publisher/get_path. Normal intersection turns
therefore remain near zero, while a true lateral lane shift is visible.
"""

from __future__ import annotations

import math
import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
import rospy
from std_msgs.msg import String

from behavior_identification.msg import VehStateSequenceArray
from driver_models_types.msg import VehicleConfig
from driver_models_types.srv import PathWithOptionsService
from road_side_system.msg import TrafficLightPhaseArray

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt


@dataclass
class Projection:
    s: float
    lateral: float
    x: float
    y: float
    tx: float
    ty: float


class ReferencePath:
    def __init__(self, points: Sequence[Tuple[float, float]]) -> None:
        path = np.asarray(points, dtype=np.float64)
        if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
            raise ValueError("reference path must contain at least two 2-D points")

        delta = path[1:] - path[:-1]
        length = np.linalg.norm(delta, axis=1)
        valid = length > 1e-4
        if not np.any(valid):
            raise ValueError("reference path contains no valid segment")

        self.points = path
        self.starts = path[:-1][valid]
        self.delta = delta[valid]
        self.length = length[valid]
        self.length_sq = self.length * self.length
        self.tangent = self.delta / self.length[:, None]
        self.cumulative = np.concatenate(([0.0], np.cumsum(self.length)))

    def project(self, x: float, y: float) -> Projection:
        point = np.array([x, y], dtype=np.float64)
        rel = point - self.starts
        ratio = np.sum(rel * self.delta, axis=1) / self.length_sq
        ratio = np.clip(ratio, 0.0, 1.0)
        candidates = self.starts + ratio[:, None] * self.delta
        distance_sq = np.sum((point - candidates) ** 2, axis=1)
        index = int(np.argmin(distance_sq))

        tx, ty = self.tangent[index]
        normal = np.array([-ty, tx])
        projection = candidates[index]
        lateral = float(np.dot(point - projection, normal))
        s_value = float(self.cumulative[index] + ratio[index] * self.length[index])

        return Projection(
            s=s_value,
            lateral=lateral,
            x=float(projection[0]),
            y=float(projection[1]),
            tx=float(tx),
            ty=float(ty),
        )


class GifPipe:
    def __init__(self, output_path: Path, width: int, height: int, fps: float, ffmpeg_bin: str) -> None:
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.process: Optional[subprocess.Popen] = None

        output_path.parent.mkdir(parents=True, exist_ok=True)
        filter_graph = (
            f"fps={fps},scale={width}:{height}:flags=lanczos,"
            "split[s0][s1];[s0]palettegen=max_colors=256[p];"
            "[s1][p]paletteuse=dither=sierra2_4a"
        )
        command = [
            ffmpeg_bin,
            "-loglevel", "warning",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-i", "-",
            "-vf", filter_graph,
            "-loop", "0",
            str(output_path),
        ]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def write(self, frame: np.ndarray) -> None:
        if self.process is None or self.process.stdin is None:
            return
        if frame.shape != (self.height, self.width, 3):
            raise ValueError(f"unexpected frame shape: {frame.shape}")
        try:
            self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        except BrokenPipeError:
            rospy.logerr("FFmpeg GIF process terminated unexpectedly.")
            self.close()

    def close(self) -> None:
        if self.process is None:
            return
        try:
            if self.process.stdin is not None:
                self.process.stdin.close()
            self.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.process.terminate()
        finally:
            self.process = None


class RealtimeChannelVisualizer:
    def __init__(self) -> None:
        self.lock = threading.RLock()

        self.update_hz = float(rospy.get_param("~update_hz", 10.0))
        self.gif_fps = float(rospy.get_param("~gif_fps", 5.0))
        self.max_history_seconds = float(rospy.get_param("~max_history_seconds", 45.0))
        self.gif_width = int(rospy.get_param("~gif_width", 1280))
        self.gif_height = int(round(self.gif_width * 9.0 / 16.0))
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.save_gif = bool(rospy.get_param("~save_gif", False))
        self.gif_output_dir = os.path.expanduser(
            str(rospy.get_param("~gif_output_dir", "~/scenario_gifs"))
        )
        self.ffmpeg_bin = str(rospy.get_param("~ffmpeg_bin", "/usr/bin/ffmpeg"))

        self.requested_vehicle_id = int(rospy.get_param("~vehicle_id", -1))
        self.requested_role_name = str(rospy.get_param("~role_name", ""))
        self.auto_select_random = bool(rospy.get_param("~auto_select_random_behavior", True))
        self.selection_fallback_delay = float(rospy.get_param("~selection_fallback_delay", 2.0))
        self.vehicle_config_topic = str(
            rospy.get_param("~vehicle_config_topic", "/carla/vehicle_config")
        )
        self.path_service_name = str(
            rospy.get_param("~path_service", "/carla_waypoint_publisher/get_path")
        )

        self.scene_radius = float(rospy.get_param("~scene_radius", 35.0))
        self.sl_limit = float(rospy.get_param("~sl_lateral_limit", 1.5))
        self.sl_baseline_samples = int(rospy.get_param("~sl_baseline_samples", 10))
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        self.overspeed_threshold = float(rospy.get_param("~overspeed_threshold", 15.0))
        self.slow_speed_threshold = float(rospy.get_param("~slow_speed_threshold", 6.0))
        self.stop_line_lateral_gate = float(
            rospy.get_param("~stop_line_lateral_gate", 8.0)
        )
        self.stop_line_cross_margin = float(
            rospy.get_param("~stop_line_cross_margin", 0.5)
        )

        self.scenario_path = str(rospy.get_param("~scenario_path", "scenario"))
        self.start_wall_time = time.monotonic()
        self.target_id: Optional[int] = (
            self.requested_vehicle_id if self.requested_vehicle_id >= 0 else None
        )
        self.target_role = self.requested_role_name
        self.configs: Dict[int, VehicleConfig] = {}
        self.current_vehicles: Dict[int, Tuple[float, float]] = {}
        self.last_target_position: Optional[Tuple[float, float]] = None
        self.reference_path: Optional[ReferencePath] = None
        self.last_path_attempt = 0.0
        self.traffic_lights = []
        self.shutdown_requested = False

        self.times: deque = deque()
        self.x_history: deque = deque()
        self.y_history: deque = deque()
        self.s_history: deque = deque()
        self.l_history: deque = deque()
        self.delta_l_history: deque = deque()
        self.speed_history: deque = deque()
        self.accel_history: deque = deque()
        self.st_distance_history: deque = deque()
        self.st_state_history: deque = deque()
        self.sl_baseline: Optional[float] = None

        self.fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
        grid = self.fig.add_gridspec(2, 2, hspace=0.32, wspace=0.25)
        self.ax_scene = self.fig.add_subplot(grid[0, 0])
        self.ax_st = self.fig.add_subplot(grid[0, 1])
        self.ax_sl = self.fig.add_subplot(grid[1, 0])
        self.ax_at = self.fig.add_subplot(grid[1, 1])
        self.ax_at_speed = self.ax_at.twinx()
        self.fig.suptitle("Reckless-driving expected-channel monitor", fontsize=14)
        self.fig.set_size_inches(self.gif_width / 100.0, self.gif_height / 100.0)
        self.fig.canvas.draw()

        self.gif_pipe: Optional[GifPipe] = None
        self.last_gif_frame_time = -1e9
        if self.save_gif:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            scenario_name = Path(self.scenario_path).stem or "scenario"
            gif_path = Path(self.gif_output_dir) / f"{scenario_name}_{stamp}_channels.gif"
            try:
                self.gif_pipe = GifPipe(
                    gif_path, self.gif_width, self.gif_height, self.gif_fps, self.ffmpeg_bin
                )
                rospy.loginfo("Channel GIF output: %s", gif_path)
            except (OSError, ValueError) as exc:
                rospy.logerr("Unable to start GIF recorder: %s", exc)
                self.gif_pipe = None

        rospy.Subscriber(
            "/veh_state_sequences",
            VehStateSequenceArray,
            self.vehicle_state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.vehicle_config_topic,
            VehicleConfig,
            self.vehicle_config_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/traffic_light/phases",
            TrafficLightPhaseArray,
            self.traffic_light_callback,
            queue_size=1,
        )
        rospy.Subscriber("/scenario_status", String, self.scenario_status_callback, queue_size=10)

        self.path_client = rospy.ServiceProxy(self.path_service_name, PathWithOptionsService)
        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def quaternion_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def vehicle_config_callback(self, msg: VehicleConfig) -> None:
        with self.lock:
            self.configs[int(msg.carla_id)] = msg

            if self.target_id is not None:
                if int(msg.carla_id) == self.target_id:
                    self.target_role = msg.role_name
                return

            if self.requested_role_name and msg.role_name == self.requested_role_name:
                self.select_target(int(msg.carla_id), msg.role_name)
                return

            random_type = getattr(msg.random_behavior, "type", "")
            is_random = bool(getattr(msg, "is_random_behavior_vehicle", False))
            is_random = is_random or (bool(random_type) and random_type.lower() != "none")
            if self.auto_select_random and is_random:
                self.select_target(int(msg.carla_id), msg.role_name)

    def select_target(self, vehicle_id: int, role_name: str) -> None:
        if self.target_id == vehicle_id:
            return
        self.target_id = vehicle_id
        self.target_role = role_name
        self.reference_path = None
        self.sl_baseline = None
        self.clear_histories()
        rospy.loginfo("Channel visualizer selected vehicle %s (%d)", role_name, vehicle_id)

    def clear_histories(self) -> None:
        for history in (
            self.times,
            self.x_history,
            self.y_history,
            self.s_history,
            self.l_history,
            self.delta_l_history,
            self.speed_history,
            self.accel_history,
            self.st_distance_history,
            self.st_state_history,
        ):
            history.clear()
        self.last_target_position = None

    def traffic_light_callback(self, msg: TrafficLightPhaseArray) -> None:
        with self.lock:
            self.traffic_lights = list(msg.phases)

    def scenario_status_callback(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if any(token in text for token in ("finished", "completed", "scenario_end", "ended", "结束")):
            rospy.loginfo("Scenario completion received: %s", msg.data)
            self.shutdown_requested = True

    def fallback_target_selection(self) -> None:
        if self.target_id is not None or not self.configs:
            return
        if time.monotonic() - self.start_wall_time < self.selection_fallback_delay:
            return
        vehicle_id = sorted(self.configs.keys())[0]
        config = self.configs[vehicle_id]
        self.select_target(vehicle_id, config.role_name)
        rospy.logwarn(
            "No random-behavior vehicle was marked; visualizer fell back to %s (%d).",
            config.role_name,
            vehicle_id,
        )

    def ensure_reference_path(self) -> None:
        with self.lock:
            if self.reference_path is not None or self.target_id is None:
                return
            config = self.configs.get(self.target_id)
        if config is None:
            return

        now = time.monotonic()
        if now - self.last_path_attempt < 1.0:
            return
        self.last_path_attempt = now

        try:
            rospy.wait_for_service(self.path_service_name, timeout=0.25)
            response = self.path_client(
                config.role_name,
                config.spawn_point.pose,
                config.goal_point.pose,
            )
            if not response.success:
                rospy.logwarn_throttle(2.0, "Reference path request failed: %s", response.message)
                return
            points = [
                (waypoint.pose.position.x, waypoint.pose.position.y)
                for waypoint in response.path.waypoints
            ]
            reference_path = ReferencePath(points)
            with self.lock:
                self.reference_path = reference_path
            rospy.loginfo(
                "Frozen expected path loaded for %s (%d): %d points.",
                config.role_name,
                self.target_id,
                len(points),
            )
        except (rospy.ROSException, rospy.ServiceException, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for expected path: %s", exc)

    def vehicle_state_callback(self, msg: VehStateSequenceArray) -> None:
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()

        with self.lock:
            self.current_vehicles = {}
            for vehicle in msg.vehicles:
                if vehicle.pose:
                    pose = vehicle.pose[-1]
                    self.current_vehicles[int(vehicle.id)] = (
                        float(pose.position.x),
                        float(pose.position.y),
                    )

            if self.target_id is None:
                return

            target = next((v for v in msg.vehicles if int(v.id) == self.target_id), None)
            if target is None or not target.pose:
                return

            pose = target.pose[-1]
            x = float(pose.position.x)
            y = float(pose.position.y)
            if self.last_target_position is not None:
                if math.hypot(x - self.last_target_position[0], y - self.last_target_position[1]) < 1e-4:
                    return
            self.last_target_position = (x, y)

            speed = float(target.speed[-1]) if target.speed else 0.0
            acceleration = 0.0
            if target.accel:
                accel = target.accel[-1].linear
                yaw = self.quaternion_yaw(pose.orientation)
                acceleration = float(accel.x * math.cos(yaw) + accel.y * math.sin(yaw))

            s_value = float("nan")
            lateral = float("nan")
            delta_l = float("nan")
            st_distance = float("nan")
            st_state = "unknown"

            if self.reference_path is not None:
                projection = self.reference_path.project(x, y)
                s_value = projection.s
                lateral = projection.lateral

                baseline_values = [
                    value for value in self.l_history if np.isfinite(value)
                ][: self.sl_baseline_samples - 1]
                if self.sl_baseline is None:
                    baseline_values.append(lateral)
                    if len(baseline_values) >= max(1, self.sl_baseline_samples):
                        self.sl_baseline = float(np.median(baseline_values))
                baseline = self.sl_baseline if self.sl_baseline is not None else 0.0
                delta_l = lateral - baseline
                st_distance, st_state = self.compute_st_value(projection)

            relative_time = stamp
            if self.times:
                relative_time = max(stamp, self.times[-1])
            self.times.append(relative_time)
            self.x_history.append(x)
            self.y_history.append(y)
            self.s_history.append(s_value)
            self.l_history.append(lateral)
            self.delta_l_history.append(delta_l)
            self.speed_history.append(speed)
            self.accel_history.append(acceleration)
            self.st_distance_history.append(st_distance)
            self.st_state_history.append(st_state)
            self.prune_histories()

    def compute_st_value(self, vehicle_projection: Projection) -> Tuple[float, str]:
        if self.reference_path is None or not self.traffic_lights:
            return float("nan"), "inactive"

        best = None
        for light in self.traffic_lights:
            stop = light.stop_location
            stop_projection = self.reference_path.project(float(stop.x), float(stop.y))
            lateral_gap = abs(stop_projection.lateral)
            longitudinal = stop_projection.s - vehicle_projection.s
            if lateral_gap > self.stop_line_lateral_gate:
                continue
            if longitudinal < -8.0:
                continue
            score = abs(longitudinal) + 2.0 * lateral_gap
            if best is None or score < best[0]:
                best = (score, longitudinal, str(light.state).lower())

        if best is None:
            return float("nan"), "inactive"
        return float(best[1]), best[2]

    def prune_histories(self) -> None:
        if not self.times:
            return
        latest = self.times[-1]
        while len(self.times) > 2 and latest - self.times[0] > self.max_history_seconds:
            for history in (
                self.times,
                self.x_history,
                self.y_history,
                self.s_history,
                self.l_history,
                self.delta_l_history,
                self.speed_history,
                self.accel_history,
                self.st_distance_history,
                self.st_state_history,
            ):
                history.popleft()

    def relative_times(self) -> np.ndarray:
        if not self.times:
            return np.empty(0)
        values = np.asarray(self.times, dtype=np.float64)
        return values - values[0]

    def draw_scene(self) -> None:
        self.ax_scene.clear()
        self.ax_scene.set_title("Scenario — live top-down view")
        self.ax_scene.set_xlabel("Map x (m)")
        self.ax_scene.set_ylabel("Map y (m)")
        self.ax_scene.grid(True, linestyle="--", alpha=0.25)
        self.ax_scene.set_aspect("equal", adjustable="box")

        if self.reference_path is not None:
            points = self.reference_path.points
            self.ax_scene.plot(points[:, 0], points[:, 1], "--", linewidth=1.3, label="Frozen expected path")

        if self.x_history:
            self.ax_scene.plot(self.x_history, self.y_history, linewidth=2.0, label="Target trajectory")

        for vehicle_id, (x, y) in self.current_vehicles.items():
            marker = "*" if vehicle_id == self.target_id else "o"
            size = 90 if vehicle_id == self.target_id else 25
            self.ax_scene.scatter([x], [y], marker=marker, s=size)
            if vehicle_id == self.target_id:
                self.ax_scene.annotate(f"ID {vehicle_id}", (x, y), xytext=(5, 5), textcoords="offset points")

        for light in self.traffic_lights:
            self.ax_scene.scatter(
                [light.stop_location.x],
                [light.stop_location.y],
                marker="s",
                s=22,
            )

        if self.x_history:
            cx, cy = self.x_history[-1], self.y_history[-1]
            radius = self.scene_radius
            self.ax_scene.set_xlim(cx - radius, cx + radius)
            self.ax_scene.set_ylim(cy - radius, cy + radius)
        elif self.current_vehicles:
            xs = [p[0] for p in self.current_vehicles.values()]
            ys = [p[1] for p in self.current_vehicles.values()]
            self.ax_scene.set_xlim(min(xs) - 10.0, max(xs) + 10.0)
            self.ax_scene.set_ylim(min(ys) - 10.0, max(ys) + 10.0)

        handles, labels = self.ax_scene.get_legend_handles_labels()
        if handles:
            self.ax_scene.legend(loc="upper right", fontsize=8)

    def draw_st(self, times: np.ndarray) -> None:
        self.ax_st.clear()
        self.ax_st.set_title("ST channel — distance to active stop line")
        self.ax_st.set_xlabel("Time (s)")
        self.ax_st.set_ylabel("Longitudinal distance (m)")
        self.ax_st.grid(True, linestyle="--", alpha=0.25)
        self.ax_st.axhline(0.0, linestyle="--", linewidth=1.2, label="Stop line")

        if len(times) == 0:
            return

        distance = np.asarray(self.st_distance_history, dtype=np.float64)
        valid = np.isfinite(distance)
        if np.any(valid):
            self.ax_st.plot(times[valid], distance[valid], linewidth=2.0, label="Vehicle → stop line")
            state_colors = {"red": "tab:red", "yellow": "goldenrod", "green": "tab:green"}
            states = np.asarray(list(self.st_state_history), dtype=object)
            for state, color in state_colors.items():
                mask = valid & np.asarray([state in item for item in states])
                if np.any(mask):
                    self.ax_st.scatter(times[mask], distance[mask], s=18, color=color, label=state.title())
            current_state = self.st_state_history[-1]
            current_distance = distance[-1]
            violation = (
                np.isfinite(current_distance)
                and current_distance < -self.stop_line_cross_margin
                and ("red" in current_state or "yellow" in current_state)
            )
            self.ax_st.set_title(
                f"ST channel — {'VIOLATION' if violation else current_state.upper()}"
            )
        else:
            self.ax_st.text(
                0.5, 0.5, "No active signal-controlled stop line",
                transform=self.ax_st.transAxes, ha="center", va="center"
            )
        self.ax_st.legend(loc="best", fontsize=8)

    def draw_sl(self, times: np.ndarray) -> None:
        self.ax_sl.clear()
        self.ax_sl.set_title("SL channel — true lateral shift on frozen path")
        self.ax_sl.set_xlabel("Time (s)")
        self.ax_sl.set_ylabel(r"Lateral shift $\Delta l$ (m)")
        self.ax_sl.grid(True, linestyle="--", alpha=0.25)
        self.ax_sl.axhspan(-self.sl_limit, self.sl_limit, alpha=0.12, label="Expected lateral channel")
        self.ax_sl.axhline(self.sl_limit, linestyle="--", linewidth=1.0)
        self.ax_sl.axhline(-self.sl_limit, linestyle="--", linewidth=1.0)

        if len(times) == 0:
            return
        shift = np.asarray(self.delta_l_history, dtype=np.float64)
        valid = np.isfinite(shift)
        if np.any(valid):
            self.ax_sl.plot(times[valid], shift[valid], linewidth=2.0, label=r"$\Delta l(t)$")
            violation_mask = valid & (np.abs(shift) > self.sl_limit)
            if np.any(violation_mask):
                self.ax_sl.scatter(
                    times[violation_mask],
                    shift[violation_mask],
                    marker="x",
                    s=35,
                    label="Outside channel",
                )
            current = shift[valid][-1]
            self.ax_sl.set_title(
                f"SL channel — {'VIOLATION' if abs(current) > self.sl_limit else 'NORMAL'}"
            )
        else:
            self.ax_sl.text(
                0.5, 0.5, "Waiting for frozen expected path",
                transform=self.ax_sl.transAxes, ha="center", va="center"
            )
        self.ax_sl.legend(loc="best", fontsize=8)

    def draw_at(self, times: np.ndarray) -> None:
        self.ax_at.clear()
        self.ax_at_speed.clear()
        self.ax_at.set_title("AT channel — acceleration and speed")
        self.ax_at.set_xlabel("Time (s)")
        self.ax_at.set_ylabel("Longitudinal acceleration (m/s²)")
        self.ax_at_speed.set_ylabel("Speed (m/s)")
        self.ax_at.grid(True, linestyle="--", alpha=0.25)
        self.ax_at.axhspan(-self.max_accel, self.max_accel, alpha=0.10)
        self.ax_at.axhline(self.max_accel, linestyle="--", linewidth=1.0)
        self.ax_at.axhline(-self.max_accel, linestyle="--", linewidth=1.0)
        self.ax_at_speed.axhline(self.overspeed_threshold, linestyle=":", linewidth=1.2)
        self.ax_at_speed.axhline(self.slow_speed_threshold, linestyle=":", linewidth=1.2)

        if len(times) == 0:
            return
        accel = np.asarray(self.accel_history, dtype=np.float64)
        speed = np.asarray(self.speed_history, dtype=np.float64)
        accel_line, = self.ax_at.plot(times, accel, linewidth=2.0, label="Acceleration")
        speed_line, = self.ax_at_speed.plot(times, speed, linewidth=1.8, linestyle="-.", label="Speed")

        violation = abs(accel[-1]) > self.max_accel or speed[-1] > self.overspeed_threshold
        self.ax_at.set_title(f"AT channel — {'VIOLATION' if violation else 'NORMAL'}")
        self.ax_at.legend([accel_line, speed_line], ["Acceleration", "Speed"], loc="best", fontsize=8)

    def draw(self) -> None:
        with self.lock:
            times = self.relative_times()
            self.draw_scene()
            self.draw_st(times)
            self.draw_sl(times)
            self.draw_at(times)
            target_text = (
                f"Target: {self.target_role or 'unresolved'}"
                + (f" / ID {self.target_id}" if self.target_id is not None else "")
            )
            self.fig.suptitle(f"Expected-channel monitor — {target_text}", fontsize=14)
            self.fig.canvas.draw()

            now = time.monotonic()
            if self.gif_pipe is not None and now - self.last_gif_frame_time >= 1.0 / self.gif_fps:
                width, height = self.fig.canvas.get_width_height()
                frame = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
                frame = frame.reshape(height, width, 3)
                if width == self.gif_width and height == self.gif_height:
                    self.gif_pipe.write(frame)
                    self.last_gif_frame_time = now

        if self.show_window and os.environ.get("DISPLAY"):
            plt.pause(0.001)

    def shutdown(self) -> None:
        if self.gif_pipe is not None:
            rospy.loginfo("Finalizing channel GIF...")
            self.gif_pipe.close()
            self.gif_pipe = None
        plt.close(self.fig)

    def spin(self) -> None:
        rate = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            self.fallback_target_selection()
            self.ensure_reference_path()
            self.draw()
            if self.shutdown_requested:
                rospy.signal_shutdown("scenario completed")
                break
            rate.sleep()


def main() -> None:
    rospy.init_node("realtime_channel_visualizer", anonymous=False)
    visualizer = RealtimeChannelVisualizer()
    visualizer.spin()


if __name__ == "__main__":
    main()
