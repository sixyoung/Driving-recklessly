#!/usr/bin/env python3
"""One-row CARLA/ST/SL/AT monitor for the fixed reckless vehicle."""
from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Sequence, Tuple

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String
from behavior_identification.msg import VehStateSequenceArray
from driver_models_types.msg import VehicleConfig
from driver_models_types.srv import PathWithOptionsService, PathWithOptionsServiceRequest
from road_side_system.msg import TrafficLightPhaseArray

try:
    import mss  # type: ignore
except ImportError:
    mss = None
try:
    from PIL import ImageGrab
except ImportError:
    ImageGrab = None


class ReferencePath:
    def __init__(self, points: Sequence[Tuple[float, float]]) -> None:
        raw = np.asarray(points, dtype=float)
        if raw.ndim != 2 or raw.shape[1] != 2 or len(raw) < 2:
            raise ValueError("invalid reference path")
        delta = raw[1:] - raw[:-1]
        length = np.linalg.norm(delta, axis=1)
        valid = length > 1e-4
        if not np.any(valid):
            raise ValueError("reference path has no valid segment")
        self.start = raw[:-1][valid]
        self.delta = delta[valid]
        self.length = length[valid]
        self.length_sq = self.length ** 2
        self.tangent = self.delta / self.length[:, None]
        self.segment_s = np.concatenate(([0.0], np.cumsum(self.length)))[:-1]

    def project(self, x: float, y: float, hint_s: Optional[float] = None):
        point = np.asarray([x, y], dtype=float)
        indices = np.arange(len(self.start))
        if hint_s is not None and np.isfinite(hint_s):
            mask = np.abs(self.segment_s - hint_s) <= 35.0
            if np.any(mask):
                indices = indices[mask]
        start = self.start[indices]
        delta = self.delta[indices]
        ratio = np.sum((point - start) * delta, axis=1) / self.length_sq[indices]
        ratio = np.clip(ratio, 0.0, 1.0)
        projected = start + ratio[:, None] * delta
        local = int(np.argmin(np.sum((point - projected) ** 2, axis=1)))
        index = int(indices[local])
        tangent = self.tangent[index]
        normal = np.asarray([-tangent[1], tangent[0]])
        lateral = float(np.dot(point - projected[local], normal))
        s_value = float(self.segment_s[index] + ratio[local] * self.length[index])
        return s_value, lateral


class CarlaWindowCapture:
    def __init__(self) -> None:
        self.title = str(rospy.get_param("~carla_window_title", "CarlaUE4"))
        self.auto = bool(rospy.get_param("~screen_auto_window", True))
        self.fallback = (
            int(rospy.get_param("~screen_left", 0)),
            int(rospy.get_param("~screen_top", 0)),
            int(rospy.get_param("~screen_width", 1280)),
            int(rospy.get_param("~screen_height", 720)),
        )
        self.bbox = self.fallback
        self.last_search = 0.0
        self.mss_client = mss.mss() if mss is not None else None

    def _detect(self) -> Optional[Tuple[int, int, int, int]]:
        if not self.auto or shutil.which("xdotool") is None:
            return None
        try:
            result = subprocess.run(
                ["xdotool", "search", "--name", self.title],
                capture_output=True, text=True, timeout=1.0, check=False)
            ids = [item for item in result.stdout.split() if item]
            if not ids:
                return None
            result = subprocess.run(
                ["xdotool", "getwindowgeometry", "--shell", ids[-1]],
                capture_output=True, text=True, timeout=1.0, check=False)
            values = dict(re.findall(r"^(X|Y|WIDTH|HEIGHT)=(-?\d+)$", result.stdout, re.M))
            if len(values) != 4:
                return None
            bbox = tuple(int(values[key]) for key in ("X", "Y", "WIDTH", "HEIGHT"))
            if bbox[2] <= 0 or bbox[3] <= 0:
                return None
            return bbox
        except (OSError, subprocess.SubprocessError, ValueError):
            return None

    def grab(self) -> Optional[np.ndarray]:
        if time.monotonic() - self.last_search > 2.0:
            self.last_search = time.monotonic()
            self.bbox = self._detect() or self.fallback
        left, top, width, height = self.bbox
        try:
            if self.mss_client is not None:
                bgra = np.asarray(self.mss_client.grab(
                    {"left": left, "top": top, "width": width, "height": height}))
                return np.ascontiguousarray(bgra[:, :, :3][:, :, ::-1])
            if ImageGrab is not None:
                image = ImageGrab.grab(
                    bbox=(left, top, left + width, top + height),
                    xdisplay=os.environ.get("DISPLAY"))
                return np.asarray(image.convert("RGB"))
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "CARLA capture failed: %s", exc)
        return None


class GifWriter:
    def __init__(self, path: Path, width: int, height: int, fps: float, ffmpeg: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        filters = (
            f"fps={fps},scale={width}:{height}:flags=lanczos,split[a][b];"
            "[a]palettegen[p];[b][p]paletteuse=dither=sierra2_4a")
        self.process = subprocess.Popen([
            ffmpeg, "-loglevel", "warning", "-y", "-f", "rawvideo",
            "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps),
            "-i", "-", "-vf", filters, "-loop", "0", str(path)
        ], stdin=subprocess.PIPE)
        self.closed = False

    def write(self, frame: np.ndarray) -> None:
        if self.closed or self.process.stdin is None:
            return
        try:
            self.process.stdin.write(np.ascontiguousarray(frame).tobytes())
        except (BrokenPipeError, OSError) as exc:
            rospy.logerr_throttle(3.0, "GIF writer stopped: %s", exc)
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            if self.process.stdin is not None:
                self.process.stdin.close()
            self.process.wait(timeout=20)
        except (BrokenPipeError, OSError):
            pass
        except subprocess.TimeoutExpired:
            self.process.terminate()


class Monitor:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.closed = False
        self.node_start_wall = time.monotonic()
        self.hz = max(float(rospy.get_param("~update_hz", 10.0)), 1.0)
        self.gif_fps = max(float(rospy.get_param("~gif_fps", 5.0)), 1.0)
        self.width = max(int(rospy.get_param("~gif_width", 1920)), 960)
        self.height = max(int(rospy.get_param("~gif_height", 540)), 320)
        self.max_seconds = max(float(rospy.get_param("~max_history_seconds", 45.0)), 5.0)
        self.show = bool(rospy.get_param("~show_window", True))
        self.save_gif = bool(rospy.get_param("~save_gif", False))
        self.output_dir = Path(os.path.expanduser(str(
            rospy.get_param("~gif_output_dir", "~/scenario_gifs"))))
        self.ffmpeg = str(rospy.get_param("~ffmpeg_bin", "/usr/bin/ffmpeg"))
        self.scenario_path = str(rospy.get_param("~scenario_path", "scenario"))
        self.completion_grace = max(
            float(rospy.get_param("~completion_status_grace_seconds", 3.0)), 0.0)

        self.requested_id = int(rospy.get_param("~vehicle_id", -1))
        self.requested_role = str(rospy.get_param("~role_name", ""))
        self.path_service = str(rospy.get_param(
            "~path_service", "/carla_waypoint_publisher/get_path"))
        self.sl_limit = float(rospy.get_param("~sl_lateral_limit", 1.5))
        self.baseline_count = max(int(rospy.get_param("~sl_baseline_samples", 10)), 1)
        self.max_accel = float(rospy.get_param("~max_accel", 3.0))
        self.max_speed = float(rospy.get_param("~overspeed_threshold", 15.0))
        self.stop_gate = float(rospy.get_param("~stop_line_lateral_gate", 8.0))

        self.target_id: Optional[int] = self.requested_id if self.requested_id >= 0 else None
        self.target_role = self.requested_role
        self.target_config: Optional[VehicleConfig] = None
        self.started = False
        self.shutdown_requested = False
        self.path: Optional[ReferencePath] = None
        self.last_s: Optional[float] = None
        self.last_path_attempt = 0.0
        self.lights = []
        self.scene_capture = CarlaWindowCapture()
        self.scene_frame: Optional[np.ndarray] = None
        self.gif: Optional[GifWriter] = None
        self.last_gif_time = 0.0

        self.time: Deque[float] = deque()
        self.speed: Deque[float] = deque()
        self.accel: Deque[float] = deque()
        self.lateral: Deque[float] = deque()
        self.stop_distance: Deque[float] = deque()
        self.stop_state: Deque[str] = deque()
        self.baseline_values = []
        self.baseline: Optional[float] = None

        if self.show and os.environ.get("DISPLAY"):
            plt.ion()
        self.fig, axes = plt.subplots(
            1, 4, figsize=(self.width / 100.0, self.height / 100.0), dpi=100,
            gridspec_kw={"wspace": 0.34})
        self.ax_scene, self.ax_st, self.ax_sl, self.ax_at = axes
        self.ax_speed = self.ax_at.twinx()
        self.draw_waiting()
        self.fig.canvas.draw()
        if self.show and os.environ.get("DISPLAY"):
            plt.show(block=False)
            plt.pause(0.05)

        self.path_client = rospy.ServiceProxy(self.path_service, PathWithOptionsService)
        rospy.Subscriber("/carla/vehicle_config", VehicleConfig, self.config_cb, queue_size=50)
        rospy.Subscriber("/veh_state_sequences", VehStateSequenceArray, self.state_cb, queue_size=1)
        rospy.Subscriber("/traffic_light/phases", TrafficLightPhaseArray, self.light_cb, queue_size=1)
        rospy.Subscriber("/scenario_status", String, self.status_cb, queue_size=10)
        rospy.on_shutdown(self.close)

    def draw_waiting(self) -> None:
        for axis, title in zip(
                (self.ax_scene, self.ax_st, self.ax_sl, self.ax_at),
                ("CARLA scene", "ST channel", "SL channel", "AT channel")):
            axis.clear()
            axis.set_title(title)
            axis.set_xticks([])
            axis.set_yticks([])
            axis.text(0.5, 0.5, "Waiting for fixed\nreckless-driving vehicle...",
                      ha="center", va="center", transform=axis.transAxes)
        self.fig.suptitle("Waiting — no channel sampling yet")

    def config_cb(self, msg: VehicleConfig) -> None:
        is_random = bool(getattr(msg, "is_random_behavior_vehicle", False))
        matches = (
            (self.requested_id >= 0 and int(msg.carla_id) == self.requested_id)
            or (self.requested_role and msg.role_name == self.requested_role)
            or (self.requested_id < 0 and not self.requested_role and is_random))
        if not matches:
            return
        with self.lock:
            if self.target_id == int(msg.carla_id) and self.target_config is not None:
                return
            self.target_id = int(msg.carla_id)
            self.target_role = msg.role_name
            self.target_config = msg
            self.started = False
            self.shutdown_requested = False
            self.path = None
            self.last_s = None
            self.baseline = None
            self.baseline_values.clear()
            self.clear_data_no_lock()
        rospy.loginfo("Locked fixed reckless vehicle: %s (%d), random=%d",
                      msg.role_name, msg.carla_id, int(is_random))

    def state_cb(self, msg: VehStateSequenceArray) -> None:
        with self.lock:
            target_id = self.target_id
        if target_id is None:
            rospy.loginfo_throttle(
                3.0,
                "Receiving /veh_state_sequences with %d vehicles; waiting for random VehicleConfig.",
                len(msg.vehicles))
            return
        target = next((v for v in msg.vehicles if int(v.id) == target_id), None)
        if target is None or not target.pose:
            rospy.logwarn_throttle(
                3.0, "Target %d not in state IDs: %s",
                target_id, [int(v.id) for v in msg.vehicles])
            return

        with self.lock:
            stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
            if self.time and stamp <= self.time[-1]:
                stamp = self.time[-1] + 1.0 / self.hz
            speed = self.get_speed(target)
            accel = self.get_accel_no_lock(target, stamp, speed)
            pose = target.pose[-1].position
            lateral = float("nan")
            stop_distance = float("nan")
            stop_state = "inactive"
            if self.path is not None:
                s_value, raw_lateral = self.path.project(pose.x, pose.y, self.last_s)
                self.last_s = s_value
                if self.baseline is None:
                    self.baseline_values.append(raw_lateral)
                    if len(self.baseline_values) >= self.baseline_count:
                        self.baseline = float(np.median(self.baseline_values))
                if self.baseline is not None:
                    lateral = raw_lateral - self.baseline
                stop_distance, stop_state = self.compute_stop_no_lock(s_value)

            first_sample = not self.started
            if first_sample:
                self.started = True
            self.time.append(stamp)
            self.speed.append(speed)
            self.accel.append(accel)
            self.lateral.append(lateral)
            self.stop_distance.append(stop_distance)
            self.stop_state.append(stop_state)
            self.prune_no_lock()
            target_role = self.target_role
            target_id = self.target_id

        if first_sample:
            rospy.loginfo("Target appeared in /veh_state_sequences; drawing and GIF start now.")
            self.start_gif()
        rospy.loginfo_throttle(
            2.0,
            "Channel input %s(%d): speed=%.3f accel=%.3f pose=%d speed_n=%d accel_n=%d",
            target_role, target_id, speed, accel,
            len(target.pose), len(target.speed), len(target.accel))

    @staticmethod
    def get_speed(target) -> float:
        if target.speed:
            value = float(target.speed[-1])
            if np.isfinite(value):
                return value
        if target.twist:
            v = target.twist[-1].linear
            return float(math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))
        return float("nan")

    def get_accel_no_lock(self, target, stamp: float, speed: float) -> float:
        if target.accel:
            value = float(target.accel[-1].linear.x)
            if np.isfinite(value):
                return value
        if self.time and self.speed and np.isfinite(speed):
            return float((speed - self.speed[-1]) / max(stamp - self.time[-1], 1e-3))
        return float("nan")

    def ensure_path(self) -> None:
        with self.lock:
            if self.path is not None or self.target_config is None:
                return
            config = self.target_config
            if time.monotonic() - self.last_path_attempt < 1.0:
                return
            self.last_path_attempt = time.monotonic()
        try:
            rospy.wait_for_service(self.path_service, timeout=0.25)
            request = PathWithOptionsServiceRequest()
            request.role_name = config.role_name
            request.start = config.spawn_point.pose
            request.goal = config.goal_point.pose
            response = self.path_client(request)
            if response.success:
                new_path = ReferencePath([
                    (wp.pose.position.x, wp.pose.position.y)
                    for wp in response.path.waypoints])
                with self.lock:
                    if self.target_config is config:
                        self.path = new_path
                        self.last_s = None
                rospy.loginfo("Frozen expected path loaded for %s.", config.role_name)
            else:
                rospy.logwarn_throttle(2.0, "Reference path request failed: %s", response.message)
        except (rospy.ROSException, rospy.ServiceException, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for expected path: %s", exc)

    def compute_stop_no_lock(self, vehicle_s: float):
        if self.path is None:
            return float("nan"), "inactive"
        best = None
        for light in self.lights:
            stop_s, stop_l = self.path.project(light.stop_location.x, light.stop_location.y)
            distance = stop_s - vehicle_s
            if abs(stop_l) > self.stop_gate or distance < -8.0:
                continue
            score = abs(distance) + 2.0 * abs(stop_l)
            if best is None or score < best[0]:
                best = (score, distance, str(light.state).lower())
        return (float("nan"), "inactive") if best is None else (best[1], best[2])

    def light_cb(self, msg: TrafficLightPhaseArray) -> None:
        with self.lock:
            self.lights = list(msg.phases)

    def status_cb(self, msg: String) -> None:
        text = " ".join(msg.data.strip().lower().split())
        with self.lock:
            started = self.started
        if not started:
            rospy.loginfo_throttle(
                5.0, "Ignore scenario status before target sampling starts: %s", msg.data)
            return
        if time.monotonic() - self.node_start_wall < self.completion_grace:
            return
        exact_end_states = {
            "finished", "completed", "ended", "scenario_end", "scenario ended",
            "scenario finished", "scenario completed", "结束", "场景结束",
        }
        if text in exact_end_states:
            rospy.loginfo("Accepted scenario completion status: %s", msg.data)
            with self.lock:
                self.shutdown_requested = True

    def start_gif(self) -> None:
        with self.lock:
            if not self.save_gif or self.gif is not None:
                return
            name = Path(self.scenario_path).stem or "scenario"
            path = self.output_dir / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}_channels.gif"
        try:
            writer = GifWriter(path, self.width, self.height, self.gif_fps, self.ffmpeg)
            with self.lock:
                if self.gif is None:
                    self.gif = writer
                else:
                    writer.close()
            rospy.loginfo("GIF recording started: %s", path)
        except OSError as exc:
            rospy.logerr("Cannot start GIF recording: %s", exc)

    def clear_data_no_lock(self) -> None:
        for values in (
                self.time, self.speed, self.accel, self.lateral,
                self.stop_distance, self.stop_state):
            values.clear()

    def prune_no_lock(self) -> None:
        while len(self.time) > 2 and self.time[-1] - self.time[0] > self.max_seconds:
            for values in (
                    self.time, self.speed, self.accel, self.lateral,
                    self.stop_distance, self.stop_state):
                values.popleft()

    def snapshot(self):
        with self.lock:
            if not self.started or not self.time:
                return None
            lengths = [
                len(self.time), len(self.speed), len(self.accel),
                len(self.lateral), len(self.stop_distance), len(self.stop_state)
            ]
            size = min(lengths)
            if size <= 0:
                return None
            if len(set(lengths)) != 1:
                rospy.logwarn_throttle(
                    2.0, "Channel buffers temporarily differ in length: %s; using %d samples.",
                    lengths, size)
            times = np.asarray(list(self.time)[-size:], dtype=float)
            return {
                "t": times - times[0],
                "speed": np.asarray(list(self.speed)[-size:], dtype=float),
                "accel": np.asarray(list(self.accel)[-size:], dtype=float),
                "lateral": np.asarray(list(self.lateral)[-size:], dtype=float),
                "stop_distance": np.asarray(list(self.stop_distance)[-size:], dtype=float),
                "stop_state": list(self.stop_state)[-size:],
                "target_role": self.target_role,
                "target_id": self.target_id,
            }

    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return
        try:
            self.draw_scene()
            self.draw_st(data["t"], data["stop_distance"], data["stop_state"])
            self.draw_sl(data["t"], data["lateral"])
            self.draw_at(data["t"], data["accel"], data["speed"])
            self.fig.suptitle(
                f"Reckless vehicle: {data['target_role']} / ID {data['target_id']}")
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
                2.0, "Channel visualizer draw error (node kept alive): %s", exc)
        finally:
            self.pump_gui()

    def pump_gui(self) -> None:
        if not self.show or not os.environ.get("DISPLAY"):
            return
        try:
            if not plt.fignum_exists(self.fig.number):
                rospy.logwarn_throttle(3.0, "Visualization window was closed by the user.")
                return
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Matplotlib GUI event error: %s", exc)

    def draw_scene(self) -> None:
        self.ax_scene.clear()
        self.ax_scene.set_title("CARLA real scene")
        self.ax_scene.set_xticks([])
        self.ax_scene.set_yticks([])
        captured = self.scene_capture.grab()
        if captured is not None:
            self.scene_frame = captured
        if self.scene_frame is None:
            self.ax_scene.text(
                0.5, 0.5, "Waiting for CarlaUE4 window",
                ha="center", va="center", transform=self.ax_scene.transAxes)
        else:
            self.ax_scene.imshow(self.scene_frame)

    def draw_st(self, t, values, states) -> None:
        self.ax_st.clear()
        self.ax_st.set_title("ST — stop-line distance")
        self.ax_st.set_xlabel("Time (s)")
        self.ax_st.set_ylabel("Distance (m)")
        self.ax_st.grid(True, linestyle="--", alpha=0.25)
        self.ax_st.axhline(0.0, linestyle="--", linewidth=1.0)
        valid = np.isfinite(values)
        if np.any(valid):
            self.ax_st.plot(t[valid], values[valid], linewidth=2.0)
            self.ax_st.set_title(f"ST — {states[-1].upper()}")
        else:
            self.ax_st.text(
                0.5, 0.5, "No active signal stop line",
                ha="center", va="center", transform=self.ax_st.transAxes)

    def draw_sl(self, t, values) -> None:
        self.ax_sl.clear()
        self.ax_sl.set_title("SL — lateral shift")
        self.ax_sl.set_xlabel("Time (s)")
        self.ax_sl.set_ylabel(r"$\Delta l$ (m)")
        self.ax_sl.grid(True, linestyle="--", alpha=0.25)
        self.ax_sl.axhspan(-self.sl_limit, self.sl_limit, alpha=0.12)
        valid = np.isfinite(values)
        if np.any(valid):
            self.ax_sl.plot(t[valid], values[valid], linewidth=2.0)
            self.ax_sl.set_title(
                f"SL — {'VIOLATION' if abs(values[valid][-1]) > self.sl_limit else 'NORMAL'}")
        else:
            self.ax_sl.text(
                0.5, 0.5, "Building path baseline",
                ha="center", va="center", transform=self.ax_sl.transAxes)

    def draw_at(self, t, accel, speed) -> None:
        self.ax_at.clear()
        self.ax_speed.clear()
        self.ax_at.set_title("AT — acceleration / speed")
        self.ax_at.set_xlabel("Time (s)")
        self.ax_at.set_ylabel("Acceleration (m/s²)")
        self.ax_speed.set_ylabel("Speed (m/s)")
        self.ax_at.grid(True, linestyle="--", alpha=0.25)
        self.ax_at.axhline(self.max_accel, linestyle="--", linewidth=1.0)
        self.ax_at.axhline(-self.max_accel, linestyle="--", linewidth=1.0)
        self.ax_speed.axhline(self.max_speed, linestyle=":", linewidth=1.0)
        av = np.isfinite(accel)
        sv = np.isfinite(speed)
        lines, labels = [], []
        if np.any(av):
            line, = self.ax_at.plot(t[av], accel[av], linewidth=2.0)
            lines.append(line)
            labels.append("Acceleration")
        if np.any(sv):
            line, = self.ax_speed.plot(t[sv], speed[sv], linestyle="-.", linewidth=1.8)
            lines.append(line)
            labels.append("Speed")
        if lines:
            self.ax_at.legend(lines, labels, fontsize=7, loc="best")

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        with self.lock:
            writer = self.gif
            self.gif = None
        if writer is not None:
            writer.close()
        try:
            plt.close(self.fig)
        except Exception:
            pass

    def spin(self) -> None:
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            try:
                self.ensure_path()
                self.draw()
                with self.lock:
                    should_shutdown = self.shutdown_requested
                if should_shutdown:
                    rospy.signal_shutdown("scenario completed")
                    break
            except Exception as exc:
                rospy.logerr_throttle(
                    2.0, "Channel visualizer loop error (node kept alive): %s", exc)
                self.pump_gui()
            rate.sleep()


def main() -> None:
    rospy.init_node("realtime_channel_visualizer", anonymous=False)
    Monitor().spin()


if __name__ == "__main__":
    main()
