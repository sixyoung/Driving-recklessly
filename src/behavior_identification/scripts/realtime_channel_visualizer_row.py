#!/usr/bin/env python3
"""CARLA scene and full-run ST/SL/AT expected-channel visualization.

Layout:
    CARLA real scene | ST longitudinal-time | SL lateral | AT acceleration
    channel-based behavior identification strip

ST is an S-T channel: x=time and y=longitudinal travelled distance. It is not
an instantaneous speed plot. The expected S-T corridor is anchored when the
startup exemption ends and is generated from the configured lower/upper speed
bounds.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import signal
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
from matplotlib import font_manager
from matplotlib.font_manager import FontProperties
import numpy as np
import rospy
from std_msgs.msg import String

from behavior_identification.msg import VehStateSequenceArray
from driver_models_types.msg import VehicleConfig
from driver_models_types.srv import PathWithOptionsService, PathWithOptionsServiceRequest

try:
    import mss  # type: ignore
except ImportError:
    mss = None

try:
    from PIL import ImageGrab
except ImportError:
    ImageGrab = None


def _find_font(candidates: Sequence[str]) -> Tuple[FontProperties, str, bool]:
    entries = list(font_manager.fontManager.ttflist)
    by_name = {entry.name.lower(): entry for entry in entries}
    for candidate in candidates:
        entry = by_name.get(candidate.lower())
        if entry is not None:
            return FontProperties(fname=entry.fname), entry.name, True
    for candidate in candidates:
        token = candidate.lower()
        for entry in entries:
            if token in entry.name.lower():
                return FontProperties(fname=entry.fname), entry.name, True
    fallback = font_manager.findfont("DejaVu Serif", fallback_to_default=True)
    return FontProperties(fname=fallback), "DejaVu Serif", False


CN_FONT, CN_FONT_NAME, CN_FONT_OK = _find_font(
    (
        "SimSun",
        "NSimSun",
        "Songti SC",
        "STSong",
        "AR PL UMing CN",
        "Noto Serif CJK SC",
        "Source Han Serif SC",
        "Source Han Serif CN",
    )
)
EN_FONT, EN_FONT_NAME, _ = _find_font(
    ("Times New Roman", "Liberation Serif", "Nimbus Roman", "Times")
)

matplotlib.rcParams.update(
    {
        "font.family": EN_FONT_NAME,
        "font.serif": [EN_FONT_NAME],
        "mathtext.fontset": "stix",
        "axes.unicode_minus": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.transparent": False,
        "text.color": "#111827",
        "axes.labelcolor": "#111827",
        "axes.edgecolor": "#111827",
        "xtick.color": "#111827",
        "ytick.color": "#111827",
    }
)

BLUE = "#2563eb"
RED = "#dc2626"
GREEN = "#16a34a"
GREEN_FILL = "#dcfce7"
GREY_FILL = "#e5e7eb"
DARK = "#111827"


class ReferencePath:
    """Polyline path with local signed Frenet projection."""

    def __init__(self, points: Sequence[Tuple[float, float]]) -> None:
        raw = np.asarray(points, dtype=np.float64)
        if raw.ndim != 2 or raw.shape[1] != 2 or len(raw) < 2:
            raise ValueError("reference path requires at least two 2-D points")
        delta = raw[1:] - raw[:-1]
        length = np.linalg.norm(delta, axis=1)
        valid = length > 1e-4
        if not np.any(valid):
            raise ValueError("reference path has no valid segment")
        self.starts = raw[:-1][valid]
        self.delta = delta[valid]
        self.length = length[valid]
        self.length_sq = self.length * self.length
        self.tangent = self.delta / self.length[:, None]
        self.segment_s = np.concatenate(([0.0], np.cumsum(self.length)))[:-1]

    def project(
        self, x: float, y: float, hint_s: Optional[float] = None
    ) -> Tuple[float, float]:
        point = np.asarray([x, y], dtype=np.float64)
        indices = np.arange(len(self.starts))
        if hint_s is not None and np.isfinite(hint_s):
            mask = np.abs(self.segment_s - hint_s) <= 35.0
            if np.any(mask):
                indices = indices[mask]
        starts = self.starts[indices]
        delta = self.delta[indices]
        ratio = np.sum((point - starts) * delta, axis=1) / self.length_sq[indices]
        ratio = np.clip(ratio, 0.0, 1.0)
        projected = starts + ratio[:, None] * delta
        local_index = int(np.argmin(np.sum((point - projected) ** 2, axis=1)))
        segment_index = int(indices[local_index])
        tangent = self.tangent[segment_index]
        normal = np.asarray([-tangent[1], tangent[0]])
        lateral = float(np.dot(point - projected[local_index], normal))
        s_value = float(
            self.segment_s[segment_index]
            + ratio[local_index] * self.length[segment_index]
        )
        return s_value, lateral


class CarlaWindowCapture:
    """Capture CarlaUE4 by window title, with a fixed-rectangle fallback."""

    def __init__(self) -> None:
        self.title = str(rospy.get_param("~carla_window_title", "CarlaUE4"))
        self.auto_window = bool(rospy.get_param("~screen_auto_window", True))
        self.fallback = (
            int(rospy.get_param("~screen_left", 0)),
            int(rospy.get_param("~screen_top", 0)),
            max(int(rospy.get_param("~screen_width", 1280)), 1),
            max(int(rospy.get_param("~screen_height", 720)), 1),
        )
        self.bbox = self.fallback
        self.last_search = 0.0
        self.mss_client = None
        if mss is not None:
            try:
                self.mss_client = mss.mss()
            except Exception as exc:
                rospy.logwarn("Unable to initialize mss: %s", exc)

    def _detect_window(self) -> Optional[Tuple[int, int, int, int]]:
        if not self.auto_window or shutil.which("xdotool") is None:
            return None
        try:
            found = subprocess.run(
                ["xdotool", "search", "--name", self.title],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            ids = [item for item in found.stdout.split() if item]
            if not ids:
                return None
            geometry = subprocess.run(
                ["xdotool", "getwindowgeometry", "--shell", ids[-1]],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            values = dict(
                re.findall(r"^(X|Y|WIDTH|HEIGHT)=(-?\d+)$", geometry.stdout, re.M)
            )
            if len(values) != 4:
                return None
            bbox = tuple(int(values[key]) for key in ("X", "Y", "WIDTH", "HEIGHT"))
            if bbox[2] <= 0 or bbox[3] <= 0:
                return None
            return bbox
        except (OSError, subprocess.SubprocessError, ValueError):
            return None

    def grab(self) -> Optional[np.ndarray]:
        if not os.environ.get("DISPLAY"):
            return None
        now = time.monotonic()
        if now - self.last_search >= 2.0:
            self.last_search = now
            self.bbox = self._detect_window() or self.fallback
        left, top, width, height = self.bbox
        try:
            if self.mss_client is not None:
                bgra = np.asarray(
                    self.mss_client.grab(
                        {"left": left, "top": top, "width": width, "height": height}
                    )
                )
                return np.ascontiguousarray(bgra[:, :, :3][:, :, ::-1])
            if ImageGrab is not None:
                image = ImageGrab.grab(
                    bbox=(left, top, left + width, top + height),
                    xdisplay=os.environ.get("DISPLAY"),
                )
                return np.asarray(image.convert("RGB"))
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "CARLA capture failed: %s", exc)
        return None


class GifWriter:
    """Record lossless temporary video, then create a palette-optimized GIF."""

    def __init__(
        self,
        output_path: Path,
        width: int,
        height: int,
        fps: float,
        ffmpeg_bin: str,
    ) -> None:
        self.output_path = output_path
        self.temp_video = output_path.with_name(output_path.stem + ".part.mkv")
        self.temp_gif = output_path.with_name(output_path.stem + ".part.gif")
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.ffmpeg_bin = ffmpeg_bin
        self.frame_count = 0
        self.last_frame: Optional[np.ndarray] = None
        self.closed = False
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._unlink(self.temp_video)
        self._unlink(self.temp_gif)

        command = [
            ffmpeg_bin,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            str(self.fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "ffv1",
            "-level",
            "3",
            "-g",
            "1",
            "-pix_fmt",
            "bgr0",
            str(self.temp_video),
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    @staticmethod
    def _unlink(path: Path) -> None:
        try:
            path.unlink(missing_ok=True)
        except TypeError:
            if path.exists():
                path.unlink()

    def write(self, frame: np.ndarray) -> bool:
        if self.closed or self.process.stdin is None or self.process.poll() is not None:
            return False
        rgb = np.asarray(frame, dtype=np.uint8)
        if rgb.shape != (self.height, self.width, 3):
            rospy.logerr_throttle(
                2.0,
                "Skip GIF frame %s; expected (%d, %d, 3)",
                rgb.shape,
                self.height,
                self.width,
            )
            return False
        try:
            contiguous = np.ascontiguousarray(rgb)
            self.process.stdin.write(contiguous.tobytes())
            self.last_frame = contiguous.copy()
            self.frame_count += 1
            return True
        except (BrokenPipeError, OSError) as exc:
            rospy.logerr_throttle(2.0, "GIF recording pipe failed: %s", exc)
            return False

    def close(self) -> Optional[Path]:
        if self.closed:
            return self.output_path if self.output_path.exists() else None
        if self.frame_count == 1 and self.last_frame is not None:
            self.write(self.last_frame)
        self.closed = True
        try:
            if self.process.stdin is not None:
                self.process.stdin.close()
        except (BrokenPipeError, OSError):
            pass

        try:
            record_code = self.process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            rospy.logwarn("Temporary GIF video did not close in 45 s; terminating.")
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            record_code = self.process.wait(timeout=5)

        if (
            record_code != 0
            or self.frame_count < 2
            or not self.temp_video.exists()
            or self.temp_video.stat().st_size <= 128
        ):
            rospy.logerr(
                "GIF recording failed: code=%s frames=%d temp_video=%s",
                record_code,
                self.frame_count,
                self.temp_video,
            )
            self._unlink(self.temp_gif)
            return None

        filter_graph = (
            "[0:v]split[a][b];"
            "[a]palettegen=stats_mode=full:reserve_transparent=0[p];"
            "[b][p]paletteuse=dither=sierra2_4a"
        )
        convert_command = [
            self.ffmpeg_bin,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(self.temp_video),
            "-filter_complex",
            filter_graph,
            "-loop",
            "0",
            "-f",
            "gif",
            str(self.temp_gif),
        ]
        try:
            converted = subprocess.run(
                convert_command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=120,
                check=False,
                start_new_session=True,
            )
            error_text = converted.stderr.decode("utf-8", errors="replace").strip()
        except subprocess.TimeoutExpired:
            rospy.logerr("GIF palette conversion timed out after 120 s.")
            return None

        valid = (
            converted.returncode == 0
            and self.temp_gif.exists()
            and self.temp_gif.stat().st_size > 128
        )
        if not valid:
            rospy.logerr(
                "GIF palette conversion failed: code=%s error=%s",
                converted.returncode,
                error_text or "no ffmpeg error text",
            )
            return None

        os.replace(str(self.temp_gif), str(self.output_path))
        self._unlink(self.temp_video)
        rospy.loginfo(
            "GIF saved: %s (%d frames, %.1f KiB)",
            self.output_path,
            self.frame_count,
            self.output_path.stat().st_size / 1024.0,
        )
        return self.output_path


class ExpectedChannelMonitor:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.closed = False
        self.node_start_wall = time.monotonic()

        self.update_hz = max(float(rospy.get_param("~update_hz", 10.0)), 1.0)
        self.gif_fps = max(float(rospy.get_param("~gif_fps", 5.0)), 1.0)
        self.figure_width = max(int(rospy.get_param("~gif_width", 3000)), 1600)
        self.figure_height = max(int(rospy.get_param("~gif_height", 620)), 440)
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.save_gif = bool(rospy.get_param("~save_gif", False))
        self.gif_output_dir = Path(
            os.path.expanduser(str(rospy.get_param("~gif_output_dir", "~/scenario_gifs")))
        )
        self.ffmpeg_bin = str(rospy.get_param("~ffmpeg_bin", "/usr/bin/ffmpeg"))
        self.scenario_path = str(rospy.get_param("~scenario_path", "scenario"))
        self.completion_grace = max(
            float(rospy.get_param("~completion_status_grace_seconds", 3.0)), 0.0
        )

        self.requested_vehicle_id = int(rospy.get_param("~vehicle_id", -1))
        self.requested_role = str(rospy.get_param("~role_name", ""))
        self.path_service_name = str(
            rospy.get_param("~path_service", "/carla_waypoint_publisher/get_path")
        )

        self.min_speed = float(rospy.get_param("~slow_speed_threshold", 6.0))
        self.max_speed = float(rospy.get_param("~overspeed_threshold", 15.0))
        if self.min_speed > self.max_speed:
            self.min_speed, self.max_speed = self.max_speed, self.min_speed
        self.speed_startup_grace = max(
            float(rospy.get_param("~speed_startup_grace_seconds", 5.0)), 0.0
        )
        self.require_previous_normal_speed = bool(
            rospy.get_param("~slow_require_previous_normal_speed", True)
        )
        self.max_position_step = max(
            float(rospy.get_param("~max_position_step", 20.0)), 1.0
        )
        self.sl_limit = abs(float(rospy.get_param("~sl_lateral_limit", 1.5)))
        self.baseline_sample_count = max(
            int(rospy.get_param("~sl_baseline_samples", 10)), 1
        )
        self.max_accel = abs(float(rospy.get_param("~max_accel", 3.0)))
        self.result_hold_seconds = max(
            float(rospy.get_param("~result_hold_seconds", 1.5)), 0.0
        )

        self.target_id: Optional[int] = (
            self.requested_vehicle_id if self.requested_vehicle_id >= 0 else None
        )
        self.target_role = self.requested_role
        self.target_config: Optional[VehicleConfig] = None
        self.target_started = False
        self.target_start_stamp: Optional[float] = None
        self.shutdown_requested = False
        self.seen_normal_speed = False

        self.sl_enabled = False
        self.sl_mode_label = "waiting for vehicle configuration"
        self.reference_path: Optional[ReferencePath] = None
        self.last_path_attempt = 0.0
        self.last_path_s: Optional[float] = None
        self.baseline_values = []
        self.lateral_baseline: Optional[float] = None

        self.last_position: Optional[Tuple[float, float]] = None
        self.cumulative_distance = 0.0
        self.timestamps: Deque[float] = deque()
        self.longitudinal_distances: Deque[float] = deque()
        self.speeds: Deque[float] = deque()
        self.accelerations: Deque[float] = deque()
        self.lateral_offsets: Deque[float] = deque()
        self.speed_evaluation_enabled: Deque[bool] = deque()

        self.scene_capture = CarlaWindowCapture()
        self.scene_frame: Optional[np.ndarray] = None
        self.gif_writer: Optional[GifWriter] = None
        self.last_gif_frame_wall = -1e9
        self.last_speed_abnormal_wall = -1e9
        self.last_lane_abnormal_wall = -1e9
        self.last_accel_abnormal_wall = -1e9

        if self.show_window and os.environ.get("DISPLAY"):
            plt.ion()

        self.fig = plt.figure(
            figsize=(self.figure_width / 100.0, self.figure_height / 100.0), dpi=100
        )
        grid = self.fig.add_gridspec(
            2,
            4,
            height_ratios=[8.0, 1.35],
            width_ratios=[0.82, 1.75, 1.65, 1.65],
            hspace=0.20,
            wspace=0.27,
        )
        self.ax_scene = self.fig.add_subplot(grid[0, 0])
        self.ax_st = self.fig.add_subplot(grid[0, 1])
        self.ax_sl = self.fig.add_subplot(grid[0, 2])
        self.ax_at = self.fig.add_subplot(grid[0, 3])
        self.ax_result = self.fig.add_subplot(grid[1, :])
        self.fig.subplots_adjust(left=0.025, right=0.992, top=0.90, bottom=0.055)
        try:
            self.fig.canvas.manager.set_window_title("CARLA Expected Channels")
        except Exception:
            pass
        self.draw_waiting()
        self.fig.canvas.draw()
        if self.show_window and os.environ.get("DISPLAY"):
            plt.show(block=False)
            plt.pause(0.05)

        self.path_client = rospy.ServiceProxy(
            self.path_service_name, PathWithOptionsService
        )
        rospy.Subscriber(
            "/carla/vehicle_config",
            VehicleConfig,
            self.vehicle_config_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/veh_state_sequences",
            VehStateSequenceArray,
            self.vehicle_state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/scenario_status", String, self.scenario_status_callback, queue_size=10
        )
        rospy.on_shutdown(self.close)

        rospy.loginfo(
            "Expected-channel monitor started: CN font=%s, Latin font=%s, show=%d, gif=%d",
            CN_FONT_NAME,
            EN_FONT_NAME,
            int(self.show_window),
            int(self.save_gif),
        )
        if not CN_FONT_OK:
            rospy.logwarn(
                "No Song/Ming CJK font found. Install fonts-noto-cjk or fonts-arphic-uming."
            )

    def draw_waiting(self) -> None:
        titles = (
            "CARLA real scene",
            "ST — longitudinal expected channel",
            "SL — straight-road lateral channel",
            "AT — acceleration expected channel",
        )
        for axis, title in zip(
            (self.ax_scene, self.ax_st, self.ax_sl, self.ax_at), titles
        ):
            axis.clear()
            axis.set_title(title, fontproperties=EN_FONT)
            axis.set_xticks([])
            axis.set_yticks([])
            axis.text(
                0.5,
                0.5,
                "Waiting for fixed\nreckless-driving vehicle...",
                ha="center",
                va="center",
                transform=axis.transAxes,
                fontproperties=EN_FONT,
            )
        self.draw_result_bar(False, False, False, waiting=True)
        self.fig.suptitle("Waiting — no channel sampling yet", fontproperties=EN_FONT)

    @staticmethod
    def config_is_random(msg: VehicleConfig) -> bool:
        explicit = bool(getattr(msg, "is_random_behavior_vehicle", False))
        random_type = str(getattr(getattr(msg, "random_behavior", None), "type", ""))
        return explicit or (bool(random_type) and random_type.lower() != "none")

    def target_matches(self, msg: VehicleConfig) -> bool:
        if self.requested_vehicle_id >= 0:
            return int(msg.carla_id) == self.requested_vehicle_id
        if self.requested_role:
            return msg.role_name == self.requested_role
        return self.config_is_random(msg)

    def vehicle_config_callback(self, msg: VehicleConfig) -> None:
        if not self.target_matches(msg):
            return
        with self.lock:
            vehicle_id = int(msg.carla_id)
            if self.target_id == vehicle_id and self.target_config is not None:
                self.target_config = msg
                return
            self.target_id = vehicle_id
            self.target_role = msg.role_name
            self.target_config = msg
            self.target_started = False
            self.target_start_stamp = None
            self.shutdown_requested = False
            self.seen_normal_speed = False
            self.reference_path = None
            self.last_path_s = None
            self.baseline_values = []
            self.lateral_baseline = None
            self.last_position = None
            self.cumulative_distance = 0.0
            self.clear_samples_no_lock()
            self.configure_sl_mode_no_lock(msg)
        rospy.loginfo(
            "Locked reckless vehicle: %s (%d), SL=%s",
            msg.role_name,
            msg.carla_id,
            self.sl_mode_label,
        )

    def configure_sl_mode_no_lock(self, msg: VehicleConfig) -> None:
        option = int(getattr(msg, "road_option", -1))
        description = " ".join(
            (
                str(getattr(msg, "scene_type", "")),
                str(getattr(msg, "scene_class", "")),
                str(getattr(msg, "spawn_key", "")),
            )
        ).lower()
        explicit_turn = any(
            token in description
            for token in (
                "left_turn",
                "right_turn",
                "turn_left",
                "turn_right",
                "左转",
                "右转",
            )
        )
        self.sl_enabled = option == 0 or (option not in (1, 2) and not explicit_turn)
        if self.sl_enabled:
            self.sl_mode_label = "straight route enabled"
        elif option == 1 or "left" in description or "左转" in description:
            self.sl_mode_label = "left-turn scene: SL not evaluated"
        elif option == 2 or "right" in description or "右转" in description:
            self.sl_mode_label = "right-turn scene: SL not evaluated"
        else:
            self.sl_mode_label = "non-straight scene: SL not evaluated"

    def vehicle_state_callback(self, msg: VehStateSequenceArray) -> None:
        with self.lock:
            target_id = self.target_id
        if target_id is None:
            rospy.loginfo_throttle(
                3.0,
                "Receiving /veh_state_sequences (%d vehicles); waiting for reckless VehicleConfig.",
                len(msg.vehicles),
            )
            return
        target = next(
            (vehicle for vehicle in msg.vehicles if int(vehicle.id) == target_id), None
        )
        if target is None or not target.pose:
            rospy.logwarn_throttle(3.0, "Target %d is absent from state data.", target_id)
            return

        with self.lock:
            stamp = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
            if self.timestamps and stamp <= self.timestamps[-1]:
                stamp = self.timestamps[-1] + 1.0 / self.update_hz
            if self.target_start_stamp is None:
                self.target_start_stamp = stamp

            speed = self.extract_speed(target)
            acceleration = self.extract_acceleration_no_lock(target, stamp, speed)
            if np.isfinite(speed) and speed >= self.min_speed:
                self.seen_normal_speed = True
            elapsed = max(stamp - self.target_start_stamp, 0.0)
            speed_eval = elapsed >= self.speed_startup_grace and (
                self.seen_normal_speed or not self.require_previous_normal_speed
            )

            pose = target.pose[-1].position
            current_position = (float(pose.x), float(pose.y))
            if self.last_position is not None:
                step = math.hypot(
                    current_position[0] - self.last_position[0],
                    current_position[1] - self.last_position[1],
                )
                if step <= self.max_position_step:
                    self.cumulative_distance += step
                else:
                    rospy.logwarn_throttle(
                        2.0, "Ignore implausible position jump %.2f m in ST channel.", step
                    )
            self.last_position = current_position

            lateral = float("nan")
            if self.sl_enabled and self.reference_path is not None:
                s_value, raw_lateral = self.reference_path.project(
                    current_position[0], current_position[1], self.last_path_s
                )
                self.last_path_s = s_value
                if self.lateral_baseline is None:
                    self.baseline_values.append(raw_lateral)
                    if len(self.baseline_values) >= self.baseline_sample_count:
                        self.lateral_baseline = float(np.median(self.baseline_values))
                if self.lateral_baseline is not None:
                    lateral = raw_lateral - self.lateral_baseline

            first_sample = not self.target_started
            self.target_started = True
            self.timestamps.append(stamp)
            self.longitudinal_distances.append(self.cumulative_distance)
            self.speeds.append(speed)
            self.accelerations.append(acceleration)
            self.lateral_offsets.append(lateral)
            self.speed_evaluation_enabled.append(speed_eval)
            role_name = self.target_role
            sample_count = len(self.timestamps)

        if first_sample:
            rospy.loginfo("Target appeared; full-run channel drawing starts now.")
        rospy.loginfo_throttle(
            2.0,
            "Channel input %s(%d): speed=%.3f accel=%.3f s=%.2f eval=%d samples=%d",
            role_name,
            target_id,
            speed,
            acceleration,
            self.cumulative_distance,
            int(speed_eval),
            sample_count,
        )

    @staticmethod
    def extract_speed(target) -> float:
        if target.speed:
            speed = float(target.speed[-1])
            if np.isfinite(speed):
                return speed
        if target.twist:
            velocity = target.twist[-1].linear
            return float(math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2))
        return float("nan")

    def extract_acceleration_no_lock(
        self, target, stamp: float, speed: float
    ) -> float:
        if target.accel:
            value = float(target.accel[-1].linear.x)
            if np.isfinite(value):
                return value
        if self.timestamps and self.speeds and np.isfinite(speed):
            dt = max(stamp - self.timestamps[-1], 1e-3)
            return float((speed - self.speeds[-1]) / dt)
        return float("nan")

    def ensure_reference_path(self) -> None:
        with self.lock:
            if (
                not self.sl_enabled
                or self.reference_path is not None
                or self.target_config is None
                or time.monotonic() - self.last_path_attempt < 1.0
            ):
                return
            self.last_path_attempt = time.monotonic()
            config = self.target_config
        try:
            rospy.wait_for_service(self.path_service_name, timeout=0.25)
            request = PathWithOptionsServiceRequest()
            request.role_name = config.role_name
            request.start = config.spawn_point.pose
            request.goal = config.goal_point.pose
            response = self.path_client(request)
            if not response.success:
                rospy.logwarn_throttle(2.0, "Reference path failed: %s", response.message)
                return
            path = ReferencePath(
                [
                    (waypoint.pose.position.x, waypoint.pose.position.y)
                    for waypoint in response.path.waypoints
                ]
            )
            with self.lock:
                if self.target_config is config:
                    self.reference_path = path
                    self.last_path_s = None
                    self.baseline_values = []
                    self.lateral_baseline = None
            rospy.loginfo("Frozen straight reference path loaded for %s.", config.role_name)
        except (rospy.ROSException, rospy.ServiceException, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for straight reference path: %s", exc)

    def scenario_status_callback(self, msg: String) -> None:
        text = " ".join(msg.data.strip().lower().split())
        with self.lock:
            started = self.target_started
        if not started or time.monotonic() - self.node_start_wall < self.completion_grace:
            return
        if text in {
            "finished",
            "completed",
            "ended",
            "scenario_end",
            "scenario ended",
            "scenario finished",
            "scenario completed",
            "结束",
            "场景结束",
        }:
            with self.lock:
                self.shutdown_requested = True

    def clear_samples_no_lock(self) -> None:
        for values in (
            self.timestamps,
            self.longitudinal_distances,
            self.speeds,
            self.accelerations,
            self.lateral_offsets,
            self.speed_evaluation_enabled,
        ):
            values.clear()

    def snapshot(self):
        with self.lock:
            if not self.target_started or not self.timestamps:
                return None
            size = min(
                len(self.timestamps),
                len(self.longitudinal_distances),
                len(self.speeds),
                len(self.accelerations),
                len(self.lateral_offsets),
                len(self.speed_evaluation_enabled),
            )
            timestamps = np.asarray(list(self.timestamps)[-size:], dtype=np.float64)
            return {
                "time": timestamps - timestamps[0],
                "distance": np.asarray(
                    list(self.longitudinal_distances)[-size:], dtype=np.float64
                ),
                "speed": np.asarray(list(self.speeds)[-size:], dtype=np.float64),
                "acceleration": np.asarray(
                    list(self.accelerations)[-size:], dtype=np.float64
                ),
                "lateral": np.asarray(
                    list(self.lateral_offsets)[-size:], dtype=np.float64
                ),
                "speed_eval": np.asarray(
                    list(self.speed_evaluation_enabled)[-size:], dtype=bool
                ),
                "target_id": self.target_id,
                "target_role": self.target_role,
                "sl_enabled": self.sl_enabled,
                "sl_mode_label": self.sl_mode_label,
            }

    @staticmethod
    def plot_abnormal_overlay(
        axis,
        times: np.ndarray,
        values: np.ndarray,
        valid: np.ndarray,
        abnormal: np.ndarray,
        label: str,
    ) -> None:
        axis.plot(
            times[valid], values[valid], linewidth=1.7, color=BLUE, label=label, zorder=3
        )
        if np.any(abnormal):
            masked = np.ma.masked_where(~abnormal, values)
            axis.plot(
                times,
                masked,
                linewidth=4.2,
                color=RED,
                solid_capstyle="round",
                label="Outside expected channel",
                zorder=6,
            )

    def draw_scene(self) -> None:
        self.ax_scene.clear()
        self.ax_scene.set_facecolor("white")
        self.ax_scene.set_title("CARLA real scene", fontproperties=EN_FONT)
        self.ax_scene.set_xticks([])
        self.ax_scene.set_yticks([])
        captured = self.scene_capture.grab()
        if captured is not None:
            self.scene_frame = captured
        if self.scene_frame is None:
            self.ax_scene.text(
                0.5,
                0.5,
                "CARLA capture unavailable\n(check DISPLAY / xdotool / mss)",
                ha="center",
                va="center",
                transform=self.ax_scene.transAxes,
                fontproperties=EN_FONT,
            )
        else:
            self.ax_scene.imshow(self.scene_frame)

    def draw_st(
        self,
        times: np.ndarray,
        distance: np.ndarray,
        speed: np.ndarray,
        evaluation_enabled: np.ndarray,
    ) -> bool:
        """Draw S-T channel: time versus longitudinal travelled distance."""
        self.ax_st.clear()
        self.ax_st.set_facecolor("white")
        self.ax_st.set_xlabel("Time from target appearance (s)", fontproperties=EN_FONT)
        self.ax_st.set_ylabel("Longitudinal displacement s (m)", fontproperties=EN_FONT)
        self.ax_st.grid(True, linestyle="--", alpha=0.22)

        valid = np.isfinite(distance)
        if not np.any(valid):
            self.ax_st.set_title("ST — waiting for displacement", fontproperties=EN_FONT)
            return False

        exempt = valid & ~evaluation_enabled
        if np.any(exempt):
            startup_end = float(times[np.where(exempt)[0][-1]])
            self.ax_st.axvspan(
                0.0,
                startup_end,
                alpha=0.28,
                color=GREY_FILL,
                label="Startup exemption",
                zorder=0,
            )

        eval_indices = np.where(valid & evaluation_enabled)[0]
        abnormal = np.zeros_like(valid, dtype=bool)
        lower = np.full_like(distance, np.nan, dtype=np.float64)
        upper = np.full_like(distance, np.nan, dtype=np.float64)
        if eval_indices.size:
            anchor_index = int(eval_indices[0])
            dt = times - times[anchor_index]
            active = valid & evaluation_enabled & (dt >= 0.0)
            lower[active] = distance[anchor_index] + self.min_speed * dt[active]
            upper[active] = distance[anchor_index] + self.max_speed * dt[active]
            self.ax_st.fill_between(
                times[active],
                lower[active],
                upper[active],
                color=GREEN_FILL,
                alpha=0.82,
                label="Expected S-T channel",
                zorder=0,
            )
            self.ax_st.plot(times[active], lower[active], "--", linewidth=1.1, color=GREEN)
            self.ax_st.plot(times[active], upper[active], "--", linewidth=1.1, color=GREEN)
            abnormal = active & ((distance < lower) | (distance > upper))

        self.plot_abnormal_overlay(
            self.ax_st, times, distance, valid, abnormal, "Actual longitudinal progress"
        )
        current_index = int(np.where(valid)[0][-1])
        if not bool(evaluation_enabled[current_index]):
            state = "STARTUP EXEMPT"
        elif (
            abnormal[current_index]
            and np.isfinite(lower[current_index])
            and distance[current_index] < lower[current_index]
        ):
            state = "PROGRESS TOO SLOW"
        elif abnormal[current_index]:
            state = "PROGRESS TOO FAST"
        else:
            state = "NORMAL"
        current_speed = speed[current_index] if np.isfinite(speed[current_index]) else float("nan")
        self.ax_st.set_title(
            f"ST — {state} (s={distance[current_index]:.1f} m, v={current_speed:.2f} m/s)",
            fontproperties=EN_FONT,
        )
        self.ax_st.legend(loc="best", fontsize=7, prop=EN_FONT)
        return bool(abnormal[current_index])

    def draw_sl(
        self,
        times: np.ndarray,
        lateral: np.ndarray,
        enabled: bool,
        mode_label: str,
    ) -> bool:
        self.ax_sl.clear()
        self.ax_sl.set_facecolor("white")
        self.ax_sl.set_xlabel("Time from target appearance (s)", fontproperties=EN_FONT)
        self.ax_sl.set_ylabel(r"Lateral offset $\Delta l$ (m)", fontproperties=EN_FONT)
        self.ax_sl.grid(True, linestyle="--", alpha=0.22)
        if not enabled:
            self.ax_sl.set_title("SL — NOT EVALUATED", fontproperties=EN_FONT)
            self.ax_sl.text(
                0.5,
                0.5,
                mode_label,
                ha="center",
                va="center",
                transform=self.ax_sl.transAxes,
                fontproperties=EN_FONT,
            )
            return False
        self.ax_sl.axhspan(
            -self.sl_limit,
            self.sl_limit,
            color=GREEN_FILL,
            alpha=0.82,
            label="Expected straight-road channel",
        )
        self.ax_sl.axhline(self.sl_limit, linestyle="--", linewidth=1.1, color=GREEN)
        self.ax_sl.axhline(-self.sl_limit, linestyle="--", linewidth=1.1, color=GREEN)
        valid = np.isfinite(lateral)
        if not np.any(valid):
            self.ax_sl.set_title("SL — building route baseline", fontproperties=EN_FONT)
            return False
        abnormal = valid & (np.abs(lateral) > self.sl_limit)
        self.plot_abnormal_overlay(
            self.ax_sl, times, lateral, valid, abnormal, "Actual lateral offset"
        )
        current_index = int(np.where(valid)[0][-1])
        state = "VIOLATION" if abnormal[current_index] else "NORMAL"
        self.ax_sl.set_title(
            f"SL — {state} ({lateral[current_index]:.2f} m)", fontproperties=EN_FONT
        )
        self.ax_sl.legend(loc="best", fontsize=7, prop=EN_FONT)
        return bool(abnormal[current_index])

    def draw_at(self, times: np.ndarray, acceleration: np.ndarray) -> bool:
        self.ax_at.clear()
        self.ax_at.set_facecolor("white")
        self.ax_at.set_xlabel("Time from target appearance (s)", fontproperties=EN_FONT)
        self.ax_at.set_ylabel("Acceleration (m/s²)", fontproperties=EN_FONT)
        self.ax_at.grid(True, linestyle="--", alpha=0.22)
        self.ax_at.axhspan(
            -self.max_accel,
            self.max_accel,
            color=GREEN_FILL,
            alpha=0.82,
            label="Expected acceleration channel",
        )
        self.ax_at.axhline(self.max_accel, linestyle="--", linewidth=1.1, color=GREEN)
        self.ax_at.axhline(-self.max_accel, linestyle="--", linewidth=1.1, color=GREEN)
        valid = np.isfinite(acceleration)
        if not np.any(valid):
            self.ax_at.set_title("AT — waiting for acceleration", fontproperties=EN_FONT)
            return False
        abnormal = valid & (np.abs(acceleration) > self.max_accel)
        self.plot_abnormal_overlay(
            self.ax_at, times, acceleration, valid, abnormal, "Actual acceleration"
        )
        current_index = int(np.where(valid)[0][-1])
        state = "VIOLATION" if abnormal[current_index] else "NORMAL"
        self.ax_at.set_title(
            f"AT — {state} ({acceleration[current_index]:.2f} m/s²)",
            fontproperties=EN_FONT,
        )
        self.ax_at.legend(loc="best", fontsize=7, prop=EN_FONT)
        return bool(abnormal[current_index])

    def draw_result_bar(
        self,
        speed_abnormal: bool,
        lane_abnormal: bool,
        accel_abnormal: bool,
        waiting: bool = False,
    ) -> None:
        self.ax_result.clear()
        self.ax_result.set_facecolor("white")
        self.ax_result.set_xlim(0.0, 4.0)
        self.ax_result.set_ylim(0.0, 1.0)
        self.ax_result.axis("off")
        self.ax_result.text(
            0.02,
            0.92,
            "实时行为辨识结果（通道判定）",
            transform=self.ax_result.transAxes,
            fontsize=11,
            fontweight="bold",
            va="top",
            fontproperties=CN_FONT,
            color=DARK,
        )
        if waiting:
            states = [False, False, False, False]
        else:
            now = time.monotonic()
            if speed_abnormal:
                self.last_speed_abnormal_wall = now
            if lane_abnormal:
                self.last_lane_abnormal_wall = now
            if accel_abnormal:
                self.last_accel_abnormal_wall = now
            speed_active = now - self.last_speed_abnormal_wall <= self.result_hold_seconds
            lane_active = now - self.last_lane_abnormal_wall <= self.result_hold_seconds
            accel_active = now - self.last_accel_abnormal_wall <= self.result_hold_seconds
            states = [
                not (speed_active or lane_active or accel_active),
                speed_active,
                lane_active,
                accel_active,
            ]

        labels = ("正常", "速度异常", "换道异常", "加速度异常")
        for index, (label, active) in enumerate(zip(labels, states)):
            if waiting:
                face, text_color, status = GREY_FILL, "#6b7280", "等待"
            elif active and index == 0:
                face, text_color, status = GREEN, "white", "当前"
            elif active:
                face, text_color, status = RED, "white", "检出"
            else:
                face, text_color, status = GREY_FILL, "#4b5563", "—"
            self.ax_result.text(
                index + 0.5,
                0.36,
                f"{label}  {status}",
                ha="center",
                va="center",
                fontsize=12,
                fontweight="bold" if active else "normal",
                color=text_color,
                fontproperties=CN_FONT,
                bbox={
                    "boxstyle": "round,pad=0.55",
                    "facecolor": face,
                    "edgecolor": face,
                    "linewidth": 1.5,
                },
            )

    def start_gif_from_frame(self, frame: np.ndarray) -> None:
        if not self.save_gif or self.gif_writer is not None:
            return
        height, width = frame.shape[:2]
        scenario_name = Path(self.scenario_path).stem or "scenario"
        output_path = self.gif_output_dir / (
            f"{scenario_name}_{time.strftime('%Y%m%d_%H%M%S')}_channels.gif"
        )
        try:
            self.gif_writer = GifWriter(
                output_path, width, height, self.gif_fps, self.ffmpeg_bin
            )
            rospy.loginfo(
                "Animated GIF recorder opened: %s (%dx%d, %.1f fps)",
                output_path,
                width,
                height,
                self.gif_fps,
            )
        except OSError as exc:
            rospy.logerr("Cannot start GIF recorder: %s", exc)

    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return
        try:
            self.draw_scene()
            speed_abnormal = self.draw_st(
                data["time"], data["distance"], data["speed"], data["speed_eval"]
            )
            lane_abnormal = self.draw_sl(
                data["time"],
                data["lateral"],
                bool(data["sl_enabled"]),
                str(data["sl_mode_label"]),
            )
            accel_abnormal = self.draw_at(data["time"], data["acceleration"])
            self.draw_result_bar(speed_abnormal, lane_abnormal, accel_abnormal)
            self.fig.suptitle(
                f"Reckless vehicle: {data['target_role']} / ID {data['target_id']}",
                fontproperties=EN_FONT,
                color=DARK,
            )
            self.fig.canvas.draw()
            frame = np.asarray(self.fig.canvas.buffer_rgba(), dtype=np.uint8)[:, :, :3]
            frame = np.ascontiguousarray(frame)
            if self.save_gif and self.gif_writer is None:
                self.start_gif_from_frame(frame)
            now = time.monotonic()
            if (
                self.gif_writer is not None
                and now - self.last_gif_frame_wall >= 1.0 / self.gif_fps
                and self.gif_writer.write(frame)
            ):
                self.last_gif_frame_wall = now
                rospy.loginfo_throttle(
                    5.0, "GIF recording active: %d frames", self.gif_writer.frame_count
                )
        except Exception as exc:
            rospy.logerr_throttle(2.0, "Expected-channel draw error: %s", exc)
        finally:
            self.pump_gui()

    def pump_gui(self) -> None:
        if not self.show_window or not os.environ.get("DISPLAY"):
            return
        try:
            if plt.fignum_exists(self.fig.number):
                self.fig.canvas.flush_events()
                plt.pause(0.001)
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Matplotlib GUI event error: %s", exc)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        writer = self.gif_writer
        self.gif_writer = None
        if writer is not None:
            rospy.loginfo(
                "Finalizing high-quality animated GIF with %d frames...",
                writer.frame_count,
            )
            writer.close()
        elif self.save_gif:
            rospy.logwarn("No GIF was created because no complete frame was rendered.")
        try:
            plt.close(self.fig)
        except Exception:
            pass

    def spin(self) -> None:
        rate = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            try:
                self.ensure_reference_path()
                self.draw()
                with self.lock:
                    should_shutdown = self.shutdown_requested
                if should_shutdown:
                    rospy.signal_shutdown("scenario completed")
                    break
            except Exception as exc:
                rospy.logerr_throttle(2.0, "Expected-channel loop error: %s", exc)
                self.pump_gui()
            rate.sleep()


def main() -> None:
    rospy.init_node("realtime_channel_visualizer", anonymous=False)
    monitor = ExpectedChannelMonitor()
    try:
        monitor.spin()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
