#!/usr/bin/env python3
"""CARLA scene and full-run ST/SL/AT expected-channel visualization.

Layout:
    CARLA real scene | ST longitudinal-time | SL lateral | AT acceleration
    channel-based behavior identification strip

ST is an S-T channel: x=time and y=longitudinal travelled distance. It is not
an instantaneous speed plot. Short rolling-horizon corridors are generated
from the measured state after the target first reaches a legal speed. Signal
phases can bend and cap those corridors before a matched stop line.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
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
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib import font_manager
from matplotlib.font_manager import FontProperties
from matplotlib.figure import Figure
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

try:
    import carla  # type: ignore
except ImportError:
    carla = None


def infer_scenario_target_role(scenario_path: Path) -> str:
    """Return the best target role, falling back to a normal vehicle."""
    try:
        payload = json.loads(Path(scenario_path).read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError):
        return ""

    vehicles = []
    for item in payload.get("objects", []):
        if isinstance(item, dict) and item.get("name") == "vehicles":
            data = item.get("data", [])
            if isinstance(data, list):
                vehicles.extend(
                    vehicle for vehicle in data if isinstance(vehicle, dict)
                )

    all_roles = []
    designated_roles = []
    abnormal_roles = []
    behavior_roles = []
    for vehicle in vehicles:
        role = str(vehicle.get("role_name", "")).strip()
        if not role:
            continue
        all_roles.append(role)
        behavior = vehicle.get("random_behavior")
        behavior = behavior if isinstance(behavior, dict) else {}
        parameters = behavior.get("parameters")
        parameters = parameters if isinstance(parameters, dict) else {}
        if bool(vehicle.get("is_random_behavior_vehicle", False)):
            designated_roles.append(role)
        if bool(parameters.get("abnormal", False)):
            abnormal_roles.append(role)
        behavior_type = str(behavior.get("type", "")).strip().lower()
        if behavior_type and behavior_type != "none":
            behavior_roles.append(role)

    unique_abnormal_roles = list(dict.fromkeys(abnormal_roles))
    if unique_abnormal_roles:
        return unique_abnormal_roles[0]

    for roles in (designated_roles, behavior_roles, all_roles):
        unique_roles = list(dict.fromkeys(roles))
        if unique_roles:
            return unique_roles[0]
    return ""


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


def _font_variant(family: str, size: float, weight: str = "bold") -> FontProperties:
    """Create a consistently sized/weighted variant of the selected font."""
    return FontProperties(family=family, size=size, weight=weight)


# GIFs are commonly reduced to roughly one quarter of their native width in
# presentation slides.  Use explicit sizes and bold weights so labels remain
# readable after that reduction.
EN_TITLE_FONT = _font_variant(EN_FONT_NAME, 22)
EN_LABEL_FONT = _font_variant(EN_FONT_NAME, 18)
EN_TICK_FONT = _font_variant(EN_FONT_NAME, 15)
EN_LEGEND_FONT = _font_variant(EN_FONT_NAME, 14)
EN_ANNOTATION_FONT = _font_variant(EN_FONT_NAME, 18)
CN_RESULT_TITLE_FONT = _font_variant(CN_FONT_NAME, 20)
CN_RESULT_FONT = _font_variant(CN_FONT_NAME, 18)

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


@dataclass(frozen=True)
class STChannelSegment:
    """A corridor prediction created from one observed vehicle state."""

    start_stamp: float
    start_distance: float
    reference_speed: float
    traffic_state: str
    stop_line_distance: float
    reference_acceleration: float = 0.0
    following_constraint: bool = False


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


class CarlaFollowCamera:
    """Attach an RGB camera to the selected CARLA vehicle."""

    def __init__(self, frame_callback) -> None:
        self.frame_callback = frame_callback
        self.enabled = bool(rospy.get_param("~follow_camera_enabled", True))
        self.host = str(rospy.get_param("~carla_host", "localhost"))
        self.port = int(rospy.get_param("~carla_port", 2000))
        self.width = max(int(rospy.get_param("~follow_camera_width", 720)), 160)
        self.height = max(int(rospy.get_param("~follow_camera_height", 720)), 90)
        self.fov = float(rospy.get_param("~follow_camera_fov", 75.0))
        self.sensor_tick = max(
            float(rospy.get_param("~follow_camera_sensor_tick", 0.1)), 0.0
        )
        self.offset_x = float(rospy.get_param("~follow_camera_offset_x", 0.0))
        self.offset_z = float(rospy.get_param("~follow_camera_offset_z", 35.0))
        self.pitch = float(rospy.get_param("~follow_camera_pitch", -90.0))
        self.client = None
        self.world = None
        self.sensor = None
        self.target_id: Optional[int] = None
        self.last_attempt = -1e9

    @staticmethod
    def select_attachment_type(attachment_types):
        for name in ("Rigid", "SpringArmGhost", "SpringArm"):
            value = getattr(attachment_types, name, None)
            if value is not None:
                return value
        raise RuntimeError("CARLA exposes no supported camera attachment type")

    def ensure(self, target_id: Optional[int]) -> None:
        if not self.enabled or carla is None or target_id is None:
            return
        if (
            self.target_id == int(target_id)
            and self.sensor is not None
            and bool(getattr(self.sensor, "is_alive", False))
        ):
            return
        now = time.monotonic()
        if now - self.last_attempt < 2.0:
            return
        self.last_attempt = now
        self.close()
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(1.5)
            self.world = self.client.get_world()
            target = self.world.get_actor(int(target_id))
            if target is None:
                raise RuntimeError(f"CARLA actor {target_id} is not available")
            blueprint = self.world.get_blueprint_library().find("sensor.camera.rgb")
            blueprint.set_attribute("image_size_x", str(self.width))
            blueprint.set_attribute("image_size_y", str(self.height))
            blueprint.set_attribute("fov", str(self.fov))
            blueprint.set_attribute("sensor_tick", str(self.sensor_tick))
            transform = carla.Transform(
                carla.Location(x=self.offset_x, z=self.offset_z),
                carla.Rotation(pitch=self.pitch),
            )
            self.sensor = self.world.spawn_actor(
                blueprint,
                transform,
                attach_to=target,
                attachment_type=self.select_attachment_type(carla.AttachmentType),
            )
            self.sensor.listen(self._on_image)
            self.target_id = int(target_id)
            rospy.loginfo(
                "CARLA follow camera attached to actor %d (%dx%d).",
                self.target_id,
                self.width,
                self.height,
            )
        except Exception as exc:
            self.close()
            rospy.logwarn_throttle(3.0, "CARLA follow camera unavailable: %s", exc)

    def _on_image(self, image) -> None:
        try:
            bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                image.height, image.width, 4
            )
            frame = np.ascontiguousarray(bgra[:, :, :3][:, :, ::-1])
            self.frame_callback(frame)
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "CARLA follow camera frame failed: %s", exc)

    def close(self) -> None:
        sensor = self.sensor
        self.sensor = None
        self.target_id = None
        self.world = None
        self.client = None
        if sensor is None:
            return
        try:
            sensor.stop()
        except (RuntimeError, AttributeError):
            pass
        try:
            sensor.destroy()
        except RuntimeError:
            pass


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
        self.frame_timestamps: List[float] = []
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

    def can_write(self, frame: np.ndarray) -> bool:
        return (
            not self.closed
            and self.process.stdin is not None
            and self.process.poll() is None
            and np.asarray(frame).shape == (self.height, self.width, 3)
        )

    def write(
        self, frame: np.ndarray, timestamp_seconds: Optional[float] = None
    ) -> bool:
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
            timestamp = (
                float(timestamp_seconds)
                if timestamp_seconds is not None
                and math.isfinite(timestamp_seconds)
                else float("nan")
            )
            if not math.isfinite(timestamp):
                timestamp = (
                    self.frame_timestamps[-1] + 1.0 / self.fps
                    if self.frame_timestamps
                    else 0.0
                )
            elif self.frame_timestamps and timestamp <= self.frame_timestamps[-1]:
                timestamp = self.frame_timestamps[-1] + 1.0 / self.fps
            self.frame_timestamps.append(timestamp)
            self.frame_count += 1
            return True
        except (BrokenPipeError, OSError) as exc:
            rospy.logerr_throttle(2.0, "GIF recording pipe failed: %s", exc)
            return False

    def close(self, duration_seconds: Optional[float] = None) -> Optional[Path]:
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

        nominal_duration = self.frame_count / self.fps
        playback_duration = nominal_duration
        if duration_seconds is not None and math.isfinite(duration_seconds):
            requested_duration = float(duration_seconds)
            if requested_duration > 0.0:
                playback_duration = requested_duration
        relative_timestamps = np.asarray(self.frame_timestamps, dtype=np.float64)
        timestamp_filter = ""
        if (
            len(relative_timestamps) == self.frame_count
            and self.frame_count >= 2
            and np.all(np.isfinite(relative_timestamps))
        ):
            relative_timestamps -= relative_timestamps[0]
            timestamp_deltas = np.diff(relative_timestamps)
            if np.all(timestamp_deltas > 0.0):
                source_duration = relative_timestamps[-1] + timestamp_deltas[-1]
                output_timestamps = (
                    relative_timestamps * playback_duration / source_duration
                )
                timestamp_terms = "+".join(
                    f"{value:.9f}*eq(N\\,{index})"
                    for index, value in enumerate(output_timestamps)
                )
                timestamp_filter = f"setpts=({timestamp_terms})/TB,"
        if not timestamp_filter:
            time_scale = playback_duration / nominal_duration
            if abs(time_scale - 1.0) > 0.001:
                timestamp_filter = f"setpts={time_scale:.9f}*PTS,"
        filter_graph = (
            f"[0:v]{timestamp_filter}split[a][b];"
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
            "-vsync",
            "vfr",
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
            "GIF saved: %s (%d frames, target duration %.2f s, %.1f KiB)",
            self.output_path,
            self.frame_count,
            playback_duration,
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
        self.gif_playback_duration_seconds = max(
            float(rospy.get_param("~gif_playback_duration_seconds", 4.5)), 0.0
        )
        self.figure_width = max(int(rospy.get_param("~gif_width", 3000)), 1600)
        self.figure_height = max(int(rospy.get_param("~gif_height", 620)), 440)
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.save_gif = bool(rospy.get_param("~save_gif", False))
        self.show_all_gifs = bool(rospy.get_param("~show_all_gifs", False))
        self.gif_output_dir = Path(
            os.path.expanduser(str(rospy.get_param("~gif_output_dir", "~/scenario_gifs")))
        )
        self.ffmpeg_bin = str(rospy.get_param("~ffmpeg_bin", "/usr/bin/ffmpeg"))
        self.scenario_path = str(rospy.get_param("~scenario_path", "scenario"))
        self.completion_grace = max(
            float(rospy.get_param("~completion_status_grace_seconds", 3.0)), 0.0
        )

        self.requested_vehicle_id = int(rospy.get_param("~vehicle_id", -1))
        self.requested_role = str(rospy.get_param("~role_name", "")).strip()
        if self.requested_vehicle_id < 0 and not self.requested_role:
            self.requested_role = infer_scenario_target_role(Path(self.scenario_path))
            if self.requested_role:
                rospy.loginfo(
                    "Scenario-selected target role: %s (CARLA ID will be resolved at runtime)",
                    self.requested_role,
                )
            else:
                rospy.logwarn(
                    "No unique behavior-test vehicle was found in scenario %s; "
                    "falling back to runtime random-vehicle flags.",
                    self.scenario_path,
                )
        self.path_service_name = str(
            rospy.get_param("~path_service", "/carla_waypoint_publisher/get_path")
        )

        self.min_speed = float(rospy.get_param("~slow_speed_threshold", 6.0))
        self.max_speed = float(rospy.get_param("~overspeed_threshold", 15.0))
        if self.min_speed > self.max_speed:
            self.min_speed, self.max_speed = self.max_speed, self.min_speed
        self.st_activation_speed = float(
            rospy.get_param("~st_activation_speed", self.min_speed)
        )
        self.st_segment_seconds = max(
            float(rospy.get_param("~st_segment_seconds", 1.5)), 0.2
        )
        self.st_half_width = max(
            float(rospy.get_param("~st_channel_half_width", 1.5)), 0.05
        )
        self.st_tracking_correction_gain = float(
            np.clip(rospy.get_param("~st_tracking_correction_gain", 1.0), 0.0, 1.0)
        )
        self.st_reference_accel_limit = max(
            float(rospy.get_param("~st_reference_accel_limit", 3.0)), 0.0
        )
        self.st_signal_braking_distance = max(
            float(rospy.get_param("~st_signal_braking_distance", 80.0)), 0.0
        )
        self.st_slow_violation_duration = max(
            float(rospy.get_param("~st_slow_violation_duration", 2.0)), 0.0
        )
        self.st_lead_detection_distance = max(
            float(rospy.get_param("~st_lead_detection_distance", 70.0)), 1.0
        )
        self.st_lead_lateral_gate = max(
            float(rospy.get_param("~st_lead_lateral_gate", 2.2)), 0.1
        )
        self.st_lead_vehicle_length = max(
            float(rospy.get_param("~st_lead_vehicle_length", 4.5)), 0.0
        )
        self.st_follow_time_headway = max(
            float(rospy.get_param("~st_follow_time_headway", 1.5)), 0.0
        )
        self.st_follow_standstill_gap = max(
            float(rospy.get_param("~st_follow_standstill_gap", 4.0)), 0.0
        )
        self.st_follow_gap_tolerance = max(
            float(rospy.get_param("~st_follow_gap_tolerance", 8.0)), 0.0
        )
        self.st_follow_comfortable_decel = max(
            float(rospy.get_param("~st_follow_comfortable_decel", 3.0)), 0.1
        )
        self.st_follow_hold_seconds = max(
            float(rospy.get_param("~st_follow_hold_seconds", 2.0)), 0.0
        )
        heading_tolerance = float(
            rospy.get_param("~st_lead_heading_tolerance_degrees", 50.0)
        )
        self.st_lead_heading_cosine = math.cos(
            math.radians(float(np.clip(heading_tolerance, 0.0, 90.0)))
        )
        self.speed_startup_grace = max(
            float(rospy.get_param("~speed_startup_grace_seconds", 0.0)), 0.0
        )
        self.start_at_spawn_for_acceleration = bool(
            rospy.get_param("~start_at_spawn_for_acceleration", True)
        )
        self.traffic_light_timeout = max(
            float(rospy.get_param("~traffic_light_msg_timeout", 1.0)), 0.1
        )
        self.use_scene_type_for_signal_control = bool(
            rospy.get_param("~use_scene_type_for_signal_control", True)
        )
        self.signalized_scene_type = str(
            rospy.get_param("~signalized_scene_type_value", "tl_intersection")
        ).strip().lower()
        self.stop_line_lateral_gate = max(
            float(rospy.get_param("~stop_line_lateral_gate", 8.0)), 0.1
        )
        self.stop_line_cross_margin = max(
            float(rospy.get_param("~stop_line_cross_margin", 0.5)), 0.0
        )
        self.stop_line_buffer = max(
            float(rospy.get_param("~stop_line_buffer", 2.0)), 0.0
        )
        self.traffic_stop_decel = max(
            float(rospy.get_param("~traffic_stop_decel", 3.0)), 0.1
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
        self.fixed_channel_axes = bool(
            rospy.get_param("~fixed_channel_axes", True)
        )
        self.channel_time_axis_seconds = max(
            float(rospy.get_param("~channel_time_axis_seconds", 60.0)), 1.0
        )
        self.st_axis_min_distance = float(
            rospy.get_param("~st_axis_min_distance", -10.0)
        )
        self.st_axis_max_distance = float(
            rospy.get_param("~st_axis_max_distance", 1000.0)
        )
        if self.st_axis_min_distance >= self.st_axis_max_distance:
            self.st_axis_min_distance, self.st_axis_max_distance = -10.0, 1000.0
        self.sl_axis_abs_limit = max(
            abs(float(rospy.get_param("~sl_axis_abs_limit", 3.0))),
            self.sl_limit + 0.1,
        )
        self.at_axis_abs_limit = max(
            abs(float(rospy.get_param("~at_axis_abs_limit", 5.0))),
            self.max_accel + 0.1,
        )

        self.target_id: Optional[int] = (
            self.requested_vehicle_id if self.requested_vehicle_id >= 0 else None
        )
        self.target_role = self.requested_role
        self.target_config: Optional[VehicleConfig] = None
        self.target_started = False
        self.target_start_stamp: Optional[float] = None
        self.shutdown_requested = False
        self.st_channel_active = False
        self.st_segments: List[STChannelSegment] = []

        self.sl_enabled = False
        self.sl_mode_label = "waiting for vehicle configuration"
        self.reference_path: Optional[ReferencePath] = None
        self.last_path_attempt = 0.0
        self.last_path_s: Optional[float] = None
        self.baseline_values = []
        self.lateral_baseline: Optional[float] = None
        self.traffic_lights = []
        self.last_traffic_light_wall = -1e9

        self.last_position: Optional[Tuple[float, float]] = None
        self.last_position_stamp: Optional[float] = None
        self.last_distance_speed = float("nan")
        self.cumulative_distance = 0.0
        self.timestamps: Deque[float] = deque()
        self.longitudinal_distances: Deque[float] = deque()
        self.speeds: Deque[float] = deque()
        self.accelerations: Deque[float] = deque()
        self.lateral_offsets: Deque[float] = deque()
        self.speed_evaluation_enabled: Deque[bool] = deque()
        self.st_lower_bounds: Deque[float] = deque()
        self.st_upper_bounds: Deque[float] = deque()
        self.stop_line_distances: Deque[float] = deque()
        self.traffic_states: Deque[str] = deque()
        self.following_constraints: Deque[bool] = deque()
        self.lead_vehicle_gaps: Deque[float] = deque()
        self.last_following_constraint_stamp: Optional[float] = None
        self.last_following_lead_id: Optional[int] = None

        self.scene_capture = CarlaWindowCapture()
        self.scene_frame: Optional[np.ndarray] = None
        self.scene_frame_is_follow_camera = False
        self.follow_camera = CarlaFollowCamera(self.set_scene_frame)
        self.camera_gif_writer: Optional[GifWriter] = None
        self.channel_gif_writer: Optional[GifWriter] = None
        self.expected_channel_gif_writer: Optional[GifWriter] = None
        self.gif_recording_failed = False
        self.gif_recording_started_wall: Optional[float] = None
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
            width_ratios=[1.35, 1.75, 1.65, 1.65],
            hspace=0.20,
            wspace=0.27,
        )
        self.ax_scene = self.fig.add_subplot(grid[0, 0])
        self.ax_st = self.fig.add_subplot(grid[0, 1])
        self.ax_sl = self.fig.add_subplot(grid[0, 2])
        self.ax_at = self.fig.add_subplot(grid[0, 3])
        self.ax_result = self.fig.add_subplot(grid[1, :])
        self.fig.subplots_adjust(left=0.025, right=0.992, top=0.90, bottom=0.055)

        self.channel_gif_fig: Optional[Figure] = None
        self.channel_gif_axes = ()
        self.expected_channel_gif_fig: Optional[Figure] = None
        self.expected_channel_gif_axes = ()
        if self.save_gif:
            self.channel_gif_fig = Figure(
                figsize=(self.figure_width / 100.0, self.figure_height / 100.0),
                dpi=100,
            )
            FigureCanvasAgg(self.channel_gif_fig)
            axes = self.channel_gif_fig.subplots(1, 3)
            self.channel_gif_axes = tuple(np.asarray(axes).reshape(-1))
            self.channel_gif_fig.subplots_adjust(
                left=0.055, right=0.992, top=0.90, bottom=0.20, wspace=0.30
            )
            if self.show_all_gifs:
                self.expected_channel_gif_fig = Figure(
                    figsize=(self.figure_width / 100.0, self.figure_height / 100.0),
                    dpi=100,
                )
                FigureCanvasAgg(self.expected_channel_gif_fig)
                expected_axes = self.expected_channel_gif_fig.subplots(1, 3)
                self.expected_channel_gif_axes = tuple(
                    np.asarray(expected_axes).reshape(-1)
                )
                self.expected_channel_gif_fig.subplots_adjust(
                    left=0.055,
                    right=0.992,
                    top=0.90,
                    bottom=0.20,
                    wspace=0.30,
                )
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
            "/traffic_light/phases",
            TrafficLightPhaseArray,
            self.traffic_light_callback,
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
        for axis in (
            self.ax_scene,
            self.ax_st,
            self.ax_sl,
            self.ax_at,
            self.ax_result,
        ):
            axis.clear()
            axis.set_axis_off()
        self.fig.suptitle("")

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
            self.reference_path = None
            self.last_path_s = None
            self.baseline_values = []
            self.lateral_baseline = None
            self.last_position = None
            self.last_position_stamp = None
            self.last_distance_speed = float("nan")
            self.last_following_constraint_stamp = None
            self.last_following_lead_id = None
            self.cumulative_distance = 0.0
            self.clear_samples_no_lock()
            self.configure_sl_mode_no_lock(msg)
        self.follow_camera.close()
        with self.lock:
            self.scene_frame = None
            self.scene_frame_is_follow_camera = False
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

    def starts_at_spawn_no_lock(self) -> bool:
        """Acceleration scenarios need AT/ST history before legal speed."""
        if not getattr(self, "start_at_spawn_for_acceleration", True):
            return False
        config = getattr(self, "target_config", None)
        if config is None:
            return False
        description = " ".join(
            (
                str(getattr(config, "scene_class", "")),
                str(getattr(config, "scene_type", "")),
                str(getattr(config, "random_behavior", "")),
            )
        ).lower()
        return (
            "\u52a0\u901f\u5ea6" in description
            or "acceleration" in description
            or "sudden_accel" in description
        )

    def traffic_light_callback(self, msg: TrafficLightPhaseArray) -> None:
        with self.lock:
            self.traffic_lights = list(msg.phases)
            self.last_traffic_light_wall = time.monotonic()

    @staticmethod
    def normalize_traffic_state(value: str) -> str:
        text = str(value).strip().lower()
        for state in ("red", "yellow", "green"):
            if state in text:
                return state
        return "unknown"

    def scene_is_signalized_no_lock(self) -> bool:
        if not self.use_scene_type_for_signal_control:
            return True
        if self.target_config is None:
            return False
        scene_type = str(getattr(self.target_config, "scene_type", ""))
        return scene_type.strip().lower() == self.signalized_scene_type

    def active_stop_line_no_lock(self, vehicle_s: Optional[float]) -> Tuple[float, str]:
        if (
            not self.scene_is_signalized_no_lock()
            or vehicle_s is None
            or self.reference_path is None
            or not self.traffic_lights
            or time.monotonic() - self.last_traffic_light_wall
            > self.traffic_light_timeout
        ):
            return float("nan"), "none"

        best = None
        for light in self.traffic_lights:
            stop = light.stop_location
            stop_s, lateral = self.reference_path.project(float(stop.x), float(stop.y))
            longitudinal = stop_s - vehicle_s
            if (
                abs(lateral) > self.stop_line_lateral_gate
                or longitudinal < -8.0
            ):
                continue
            score = abs(longitudinal) + 2.0 * abs(lateral)
            if best is None or score < best[0]:
                best = (
                    score,
                    self.cumulative_distance + longitudinal,
                    self.normalize_traffic_state(light.state),
                )
        if best is None:
            return float("nan"), "none"
        return float(best[1]), str(best[2])

    def segment_bounds(
        self, segment: STChannelSegment, stamps: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return a fixed-width rolling corridor, capped by a red stop line."""
        dt = np.clip(
            np.asarray(stamps, dtype=np.float64) - segment.start_stamp,
            0.0,
            self.st_segment_seconds,
        )
        progress = segment.reference_speed * dt
        constrained = (
            segment.traffic_state in {"red", "yellow"}
            and np.isfinite(segment.stop_line_distance)
        )
        if np.isfinite(segment.reference_acceleration):
            acceleration = segment.reference_acceleration
            if acceleration > 1e-6:
                time_to_max = max(
                    (self.max_speed - segment.reference_speed) / acceleration,
                    0.0,
                )
                accelerated_progress = (
                    segment.reference_speed * time_to_max
                    + 0.5 * acceleration * time_to_max**2
                    + self.max_speed * np.maximum(dt - time_to_max, 0.0)
                )
                progress = np.where(
                    dt <= time_to_max,
                    segment.reference_speed * dt + 0.5 * acceleration * dt**2,
                    accelerated_progress,
                )
            elif acceleration < -1e-6:
                speed_floor = 0.0 if constrained else self.min_speed
                time_to_floor = max(
                    (segment.reference_speed - speed_floor) / -acceleration,
                    0.0,
                )
                decelerated_progress = (
                    segment.reference_speed * time_to_floor
                    + 0.5 * acceleration * time_to_floor**2
                    + speed_floor * np.maximum(dt - time_to_floor, 0.0)
                )
                progress = np.where(
                    dt <= time_to_floor,
                    segment.reference_speed * dt + 0.5 * acceleration * dt**2,
                    decelerated_progress,
                )
        stop_center = float("inf")
        if constrained:
            stop_center = (
                segment.stop_line_distance
                - self.stop_line_buffer
                - self.st_half_width
            )
            available = max(stop_center - segment.start_distance, 0.0)
            braking_distance = (
                segment.reference_speed**2 / (2.0 * self.traffic_stop_decel)
            )
            cruise_distance = max(available - braking_distance, 0.0)
            cruise_time = cruise_distance / max(segment.reference_speed, 1e-6)
            braking_time = np.maximum(dt - cruise_time, 0.0)
            braking_progress = (
                cruise_distance
                + segment.reference_speed * braking_time
                - 0.5 * self.traffic_stop_decel * braking_time**2
            )
            progress = np.where(
                dt <= cruise_time,
                progress,
                np.minimum(progress, braking_progress),
            )
            progress = np.clip(progress, 0.0, available)

        center = segment.start_distance + progress
        if constrained and stop_center >= segment.start_distance:
            center = np.minimum(center, stop_center)
        return center - self.st_half_width, center + self.st_half_width

    def update_st_segment_no_lock(
        self,
        stamp: float,
        speed: float,
        stop_line_distance: float,
        traffic_state: str,
        acceleration: float = float("nan"),
        following_constraint: bool = False,
    ) -> Tuple[float, float]:
        current = self.st_segments[-1] if self.st_segments else None
        constraint_changed = current is not None and (
            current.traffic_state != traffic_state
            or current.following_constraint != following_constraint
            or (
                np.isfinite(current.stop_line_distance)
                != np.isfinite(stop_line_distance)
            )
            or (
                np.isfinite(current.stop_line_distance)
                and abs(current.stop_line_distance - stop_line_distance) > 1.0
            )
        )
        if (
            current is None
            or stamp - current.start_stamp >= self.st_segment_seconds
            or constraint_changed
        ):
            if np.isfinite(speed):
                observed_speed = speed
            elif current is not None:
                observed_speed = current.reference_speed
            else:
                observed_speed = self.min_speed
            stop_gap = stop_line_distance - self.cumulative_distance
            signal_braking = (
                traffic_state in {"red", "yellow"}
                and np.isfinite(stop_line_distance)
                and -self.stop_line_cross_margin <= stop_gap
                and stop_gap <= self.st_signal_braking_distance
            )
            speed_floor = (
                0.0 if signal_braking or following_constraint else self.min_speed
            )
            reference_speed = float(np.clip(observed_speed, speed_floor, self.max_speed))
            start_distance = self.cumulative_distance
            if current is not None:
                previous_lower, previous_upper = self.segment_bounds(
                    current, np.asarray([stamp])
                )
                start_distance = float(
                    0.5 * (previous_lower[0] + previous_upper[0])
                )
                if following_constraint:
                    # A real lead vehicle changes the feasible longitudinal
                    # motion. Re-anchor the next short prediction to the
                    # measured state instead of preserving a free-flow error.
                    start_distance = self.cumulative_distance
                else:
                    tracking_error = self.cumulative_distance - start_distance
                    correction = (
                        self.st_tracking_correction_gain
                        * tracking_error
                        / self.st_segment_seconds
                    )
                    reference_speed = float(
                        np.clip(
                            reference_speed + correction,
                            speed_floor,
                            self.max_speed,
                        )
                    )
            reference_acceleration = (
                float(
                    np.clip(
                        acceleration,
                        -self.st_reference_accel_limit,
                        self.st_reference_accel_limit,
                    )
                )
                if np.isfinite(acceleration)
                else 0.0
            )
            if (
                not signal_braking
                and not following_constraint
                and reference_speed <= self.min_speed
                and reference_acceleration < 0.0
            ):
                reference_acceleration = 0.0
            if reference_speed >= self.max_speed and reference_acceleration > 0.0:
                reference_acceleration = 0.0
            current = STChannelSegment(
                start_stamp=stamp,
                start_distance=start_distance,
                reference_speed=reference_speed,
                traffic_state=traffic_state,
                stop_line_distance=stop_line_distance,
                reference_acceleration=reference_acceleration,
                following_constraint=following_constraint,
            )
            self.st_segments.append(current)
            rospy.loginfo(
                "Generated ST segment: s=%.2f, v_ref=%.2f, signal=%s, "
                "stop=%.2f, following=%d",
                current.start_distance,
                current.reference_speed,
                current.traffic_state,
                current.stop_line_distance,
                int(current.following_constraint),
            )
        lower, upper = self.segment_bounds(current, np.asarray([stamp]))
        return float(lower[0]), float(upper[0])

    def st_violation_masks(
        self,
        distance: np.ndarray,
        speed: np.ndarray,
        evaluation_enabled: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        stop_line: np.ndarray,
        traffic_state: np.ndarray,
        times: Optional[np.ndarray] = None,
        following_constraint: Optional[np.ndarray] = None,
    ) -> Dict[str, np.ndarray]:
        """Evaluate ST position, slope and signal constraints consistently."""
        valid = np.isfinite(distance)
        active = valid & evaluation_enabled & np.isfinite(lower) & np.isfinite(upper)
        stop_valid = np.isfinite(stop_line)
        stop_gap = stop_line - distance
        constrained = np.asarray(
            [state in {"red", "yellow"} for state in traffic_state], dtype=bool
        )
        following = (
            np.zeros_like(active, dtype=bool)
            if following_constraint is None
            else np.asarray(following_constraint, dtype=bool)
        )
        signal_braking = (
            active
            & constrained
            & stop_valid
            & (stop_gap >= -self.stop_line_cross_margin)
            & (stop_gap <= self.st_signal_braking_distance)
        )
        raw_speed_slow = (
            active
            & np.isfinite(speed)
            & (speed < self.min_speed)
            & ~signal_braking
            & ~following
        )
        speed_slow = raw_speed_slow.copy()
        if times is not None and self.st_slow_violation_duration > 0.0:
            speed_slow[:] = False
            run_start: Optional[int] = None
            for index, is_slow in enumerate(raw_speed_slow):
                if not is_slow:
                    run_start = None
                    continue
                if run_start is None:
                    run_start = index
                if times[index] - times[run_start] >= self.st_slow_violation_duration:
                    speed_slow[index] = True
        masks = {
            "position_slow": active & (distance < lower) & ~following,
            "position_fast": active & (distance > upper),
            "speed_slow": speed_slow,
            "speed_fast": active & np.isfinite(speed) & (speed > self.max_speed),
            "stop_crossing": (
                active
                & constrained
                & stop_valid
                & (distance > stop_line + self.stop_line_cross_margin)
            ),
        }
        masks["abnormal"] = np.logical_or.reduce(tuple(masks.values()))
        masks["signal_braking"] = signal_braking
        masks["constrained"] = constrained
        masks["following"] = following
        return masks

    def update_cumulative_distance_no_lock(
        self,
        current_position: Tuple[float, float],
        stamp: float,
        speed: float,
    ) -> None:
        if self.last_position is not None:
            step = math.hypot(
                current_position[0] - self.last_position[0],
                current_position[1] - self.last_position[1],
            )
            dt = (
                max(stamp - self.last_position_stamp, 0.0)
                if self.last_position_stamp is not None
                else 0.0
            )
            valid_speeds = [
                value
                for value in (self.last_distance_speed, speed)
                if np.isfinite(value) and value >= 0.0
            ]
            peak_speed = max(valid_speeds, default=0.0)
            dynamic_step_limit = max(
                self.max_position_step,
                1.5 * peak_speed * dt + 3.0,
            )
            if step <= dynamic_step_limit:
                self.cumulative_distance += step
            else:
                integrated_speed = (
                    float(np.mean(valid_speeds)) if valid_speeds else 0.0
                )
                integrated_step = integrated_speed * dt
                self.cumulative_distance += integrated_step
                rospy.logwarn_throttle(
                    2.0,
                    "Replace implausible position jump %.2f m (limit %.2f m) "
                    "with speed-integrated progress %.2f m.",
                    step,
                    dynamic_step_limit,
                    integrated_step,
                )
        self.last_position = current_position
        self.last_position_stamp = stamp
        self.last_distance_speed = speed

    @staticmethod
    def pose_heading(pose) -> Tuple[float, float]:
        orientation = getattr(pose, "orientation", None)
        if orientation is None:
            return 1.0, 0.0
        x = float(getattr(orientation, "x", 0.0))
        y = float(getattr(orientation, "y", 0.0))
        z = float(getattr(orientation, "z", 0.0))
        w = float(getattr(orientation, "w", 1.0))
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return math.cos(yaw), math.sin(yaw)

    def find_lead_vehicle_no_lock(
        self,
        vehicles,
        target,
        target_route_s: Optional[float],
        target_route_lateral: Optional[float],
        target_speed: float,
        stamp: float,
    ) -> Tuple[int, float, float, bool]:
        """Return the nearest credible same-lane lead and its constraint state."""
        target_pose = target.pose[-1]
        target_position = target_pose.position
        target_heading = self.pose_heading(target_pose)
        nearest = None

        for vehicle in vehicles:
            if int(vehicle.id) == int(target.id) or not vehicle.pose:
                continue
            candidate_pose = vehicle.pose[-1]
            candidate_heading = self.pose_heading(candidate_pose)
            heading_dot = (
                target_heading[0] * candidate_heading[0]
                + target_heading[1] * candidate_heading[1]
            )
            if heading_dot < self.st_lead_heading_cosine:
                continue

            candidate_position = candidate_pose.position
            if self.reference_path is not None and target_route_s is not None:
                candidate_s, candidate_lateral = self.reference_path.project(
                    float(candidate_position.x), float(candidate_position.y)
                )
                longitudinal_gap = candidate_s - target_route_s
                lateral_gap = abs(
                    candidate_lateral - float(target_route_lateral or 0.0)
                )
            else:
                delta_x = float(candidate_position.x - target_position.x)
                delta_y = float(candidate_position.y - target_position.y)
                longitudinal_gap = (
                    delta_x * target_heading[0] + delta_y * target_heading[1]
                )
                lateral_gap = abs(
                    -delta_x * target_heading[1] + delta_y * target_heading[0]
                )

            bumper_gap = longitudinal_gap - self.st_lead_vehicle_length
            if (
                bumper_gap < 0.0
                or bumper_gap > self.st_lead_detection_distance
                or lateral_gap > self.st_lead_lateral_gate
            ):
                continue
            candidate_speed = self.extract_speed(vehicle)
            score = (bumper_gap, lateral_gap)
            if nearest is None or score < nearest[0]:
                nearest = (
                    score,
                    int(vehicle.id),
                    float(bumper_gap),
                    float(candidate_speed),
                )

        if nearest is None:
            return -1, float("nan"), float("nan"), False

        _, lead_id, lead_gap, lead_speed = nearest
        ego_speed = max(float(target_speed), 0.0) if np.isfinite(target_speed) else 0.0
        comparison_speed = lead_speed if np.isfinite(lead_speed) else ego_speed
        closing_braking_distance = max(
            (ego_speed**2 - max(comparison_speed, 0.0) ** 2)
            / (2.0 * self.st_follow_comfortable_decel),
            0.0,
        )
        constraint_distance = (
            self.st_follow_standstill_gap
            + self.st_follow_time_headway * ego_speed
            + closing_braking_distance
            + self.st_follow_gap_tolerance
        )
        following = lead_gap <= constraint_distance
        if following:
            self.last_following_constraint_stamp = stamp
            self.last_following_lead_id = lead_id
        elif (
            self.last_following_constraint_stamp is not None
            and self.last_following_lead_id == lead_id
            and stamp - self.last_following_constraint_stamp
            <= self.st_follow_hold_seconds
        ):
            following = True
        return lead_id, lead_gap, lead_speed, following

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
            elapsed = max(stamp - self.target_start_stamp, 0.0)

            pose = target.pose[-1].position
            current_position = (float(pose.x), float(pose.y))
            self.update_cumulative_distance_no_lock(current_position, stamp, speed)

            route_s: Optional[float] = None
            route_lateral: Optional[float] = None
            lateral = float("nan")
            if self.reference_path is not None:
                s_value, raw_lateral = self.reference_path.project(
                    current_position[0], current_position[1], self.last_path_s
                )
                self.last_path_s = s_value
                route_s = s_value
                route_lateral = raw_lateral
                if self.sl_enabled:
                    if self.lateral_baseline is None:
                        self.baseline_values.append(raw_lateral)
                        if len(self.baseline_values) >= self.baseline_sample_count:
                            self.lateral_baseline = float(
                                np.median(self.baseline_values)
                            )
                    if self.lateral_baseline is not None:
                        lateral = raw_lateral - self.lateral_baseline

            legal_speed = (
                np.isfinite(speed)
                and speed >= self.st_activation_speed
                and speed <= self.max_speed
            )
            activation_speed_reached = (
                np.isfinite(speed) and speed >= self.st_activation_speed
            )
            start_at_spawn = self.starts_at_spawn_no_lock()
            activated_now = False
            if not self.st_channel_active and start_at_spawn:
                self.activate_channel_no_lock(stamp)
                activated_now = True
            elif (
                not self.st_channel_active
                and elapsed >= self.speed_startup_grace
                and legal_speed
            ):
                self.activate_channel_no_lock(stamp)
                activated_now = True
            if not self.st_channel_active:
                rospy.loginfo_throttle(
                    2.0,
                    "Target %s(%d) waiting for legal speed: %.3f m/s.",
                    self.target_role,
                    target_id,
                    speed,
                )
                return

            stop_line_distance, traffic_state = self.active_stop_line_no_lock(route_s)
            lead_id, lead_gap, lead_speed, following = (
                self.find_lead_vehicle_no_lock(
                    msg.vehicles,
                    target,
                    route_s,
                    route_lateral,
                    speed,
                    stamp,
                )
            )
            channel_ready = (
                not start_at_spawn
                or bool(self.st_segments)
                or activation_speed_reached
            )
            if channel_ready:
                lower, upper = self.update_st_segment_no_lock(
                    stamp,
                    speed,
                    stop_line_distance,
                    traffic_state,
                    acceleration,
                    following_constraint=following,
                )
            else:
                lower = float("nan")
                upper = float("nan")

            self.target_started = True
            self.timestamps.append(stamp)
            self.longitudinal_distances.append(self.cumulative_distance)
            self.speeds.append(speed)
            self.accelerations.append(acceleration)
            self.lateral_offsets.append(lateral)
            evaluation_enabled = channel_ready
            self.speed_evaluation_enabled.append(evaluation_enabled)
            self.st_lower_bounds.append(lower)
            self.st_upper_bounds.append(upper)
            self.stop_line_distances.append(stop_line_distance)
            self.traffic_states.append(traffic_state)
            self.following_constraints.append(following)
            self.lead_vehicle_gaps.append(lead_gap)
            role_name = self.target_role
            sample_count = len(self.timestamps)

        if activated_now:
            if start_at_spawn:
                rospy.loginfo(
                    "Full visualization activated at vehicle birth (initial speed %.2f m/s); "
                    "time and ST displacement reset to zero.",
                    speed,
                )
            else:
                rospy.loginfo(
                    "Full visualization activated at legal speed %.2f m/s "
                    "(threshold %.2f m/s); time and ST displacement reset to zero.",
                    speed,
                    self.st_activation_speed,
                )
        rospy.loginfo_throttle(
            2.0,
            "Channel input %s(%d): speed=%.3f accel=%.3f s=%.2f eval=%d "
            "lead=%d gap=%.2f following=%d samples=%d",
            role_name,
            target_id,
            speed,
            acceleration,
            self.cumulative_distance,
            int(evaluation_enabled),
            lead_id,
            lead_gap,
            int(following),
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
        if np.isfinite(speed):
            # There is no prior state at vehicle birth from which to estimate
            # acceleration. Record a neutral zero so AT visibly starts at t=0.
            return 0.0
        return float("nan")

    def ensure_reference_path(self) -> None:
        with self.lock:
            if (
                self.reference_path is not None
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
            rospy.loginfo("Frozen reference path loaded for %s.", config.role_name)
        except (rospy.ROSException, rospy.ServiceException, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for reference path: %s", exc)

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

    def clear_history_no_lock(self) -> None:
        for values in (
            self.timestamps,
            self.longitudinal_distances,
            self.speeds,
            self.accelerations,
            self.lateral_offsets,
            self.speed_evaluation_enabled,
            self.st_lower_bounds,
            self.st_upper_bounds,
            self.stop_line_distances,
            self.traffic_states,
            self.following_constraints,
            self.lead_vehicle_gaps,
        ):
            values.clear()

    def activate_channel_no_lock(self, stamp: float) -> None:
        self.clear_history_no_lock()
        self.st_segments.clear()
        self.st_channel_active = True
        self.target_started = False
        self.target_start_stamp = stamp
        self.cumulative_distance = 0.0

    def clear_samples_no_lock(self) -> None:
        self.clear_history_no_lock()
        self.st_channel_active = False
        self.st_segments.clear()

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
                len(self.st_lower_bounds),
                len(self.st_upper_bounds),
                len(self.stop_line_distances),
                len(self.traffic_states),
                len(self.following_constraints),
                len(self.lead_vehicle_gaps),
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
                "st_lower": np.asarray(
                    list(self.st_lower_bounds)[-size:], dtype=np.float64
                ),
                "st_upper": np.asarray(
                    list(self.st_upper_bounds)[-size:], dtype=np.float64
                ),
                "stop_line": np.asarray(
                    list(self.stop_line_distances)[-size:], dtype=np.float64
                ),
                "traffic_state": np.asarray(
                    list(self.traffic_states)[-size:], dtype=object
                ),
                "following": np.asarray(
                    list(self.following_constraints)[-size:], dtype=bool
                ),
                "lead_gap": np.asarray(
                    list(self.lead_vehicle_gaps)[-size:], dtype=np.float64
                ),
                "st_segments": list(self.st_segments),
                "first_stamp": float(timestamps[0]),
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
                linewidth=2.4,
                linestyle="--",
                marker="x",
                markersize=3.8,
                markeredgewidth=0.9,
                color=DARK,
                solid_capstyle="round",
                label="Outside expected channel",
                zorder=6,
            )

    def draw_scene(self) -> None:
        self.ax_scene.clear()
        self.ax_scene.set_facecolor("white")
        self.ax_scene.set_title("CARLA bird's-eye follow view", fontproperties=EN_FONT)
        self.ax_scene.set_xticks([])
        self.ax_scene.set_yticks([])
        self.follow_camera.ensure(self.target_id)
        with self.lock:
            frame = self.scene_frame
        if frame is None:
            captured = self.scene_capture.grab()
            if captured is not None:
                self.set_scene_frame(captured, is_follow_camera=False)
                frame = captured
        if frame is None:
            self.ax_scene.text(
                0.5,
                0.5,
                "Waiting for target follow camera...",
                ha="center",
                va="center",
                transform=self.ax_scene.transAxes,
                fontproperties=EN_FONT,
            )
        else:
            self.ax_scene.imshow(frame)

    def set_scene_frame(
        self, frame: np.ndarray, is_follow_camera: bool = True
    ) -> None:
        with self.lock:
            self.scene_frame = np.ascontiguousarray(frame)
            self.scene_frame_is_follow_camera = bool(is_follow_camera)

    def apply_fixed_axes(self, axis, y_min: float, y_max: float) -> None:
        if not self.fixed_channel_axes:
            return
        axis.set_xlim(0.0, self.channel_time_axis_seconds)
        axis.set_ylim(y_min, y_max)

    @staticmethod
    def style_axis_text(axis) -> None:
        """Apply presentation-sized bold tick labels after each axis redraw."""
        axis.tick_params(
            axis="both", which="major", labelsize=15, width=1.2, length=6
        )
        for label in axis.get_xticklabels() + axis.get_yticklabels():
            label.set_fontproperties(EN_TICK_FONT)

    def draw_st(
        self,
        times: np.ndarray,
        distance: np.ndarray,
        speed: np.ndarray,
        evaluation_enabled: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        stop_line: np.ndarray,
        traffic_state: np.ndarray,
        segments: Sequence[STChannelSegment],
        first_stamp: float,
        following_constraint: Optional[np.ndarray] = None,
        lead_gap: Optional[np.ndarray] = None,
    ) -> bool:
        """Draw rolling S-T predictions and signal-aware stop constraints."""
        self.ax_st.clear()
        self.ax_st.set_facecolor("white")
        self.ax_st.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_st.set_ylabel(
            "Longitudinal displacement s (m)",
            fontproperties=EN_LABEL_FONT,
            labelpad=8,
        )
        self.ax_st.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_st, self.st_axis_min_distance, self.st_axis_max_distance
        )
        self.style_axis_text(self.ax_st)

        valid = np.isfinite(distance)
        if not np.any(valid):
            self.ax_st.set_title(
                "ST — waiting for displacement",
                fontproperties=EN_TITLE_FONT,
                pad=12,
            )
            return False

        waiting = valid & ~evaluation_enabled
        if np.any(waiting):
            startup_end = float(times[np.where(waiting)[0][-1]])
            self.ax_st.axvspan(
                0.0,
                startup_end,
                alpha=0.28,
                color=GREY_FILL,
                label="Waiting for legal speed",
                zorder=0,
            )

        active = valid & evaluation_enabled & np.isfinite(lower) & np.isfinite(upper)
        if np.any(active):
            channel_times = times[active]
            channel_lower = lower[active]
            channel_upper = upper[active]
            if segments:
                latest = segments[-1]
                future_stamp = latest.start_stamp + self.st_segment_seconds
                future_time = future_stamp - first_stamp
                if future_time > channel_times[-1] + 1e-6:
                    future_lower, future_upper = self.segment_bounds(
                        latest, np.asarray([future_stamp])
                    )
                    channel_times = np.append(channel_times, future_time)
                    channel_lower = np.append(channel_lower, future_lower[0])
                    channel_upper = np.append(channel_upper, future_upper[0])
            self.ax_st.fill_between(
                channel_times,
                channel_lower,
                channel_upper,
                color=GREEN_FILL,
                alpha=0.82,
                label="Continuous real-time expected S-T channel",
                zorder=0,
            )
            self.ax_st.plot(
                channel_times, channel_lower, "--", linewidth=1.1, color=GREEN
            )
            self.ax_st.plot(
                channel_times, channel_upper, "--", linewidth=1.1, color=GREEN
            )

        following = (
            np.zeros_like(valid, dtype=bool)
            if following_constraint is None
            else np.asarray(following_constraint, dtype=bool)
        )
        lead_gaps = (
            np.full(distance.shape, float("nan"), dtype=np.float64)
            if lead_gap is None
            else np.asarray(lead_gap, dtype=np.float64)
        )
        violations = self.st_violation_masks(
            distance,
            speed,
            evaluation_enabled,
            lower,
            upper,
            stop_line,
            traffic_state,
            times,
            following,
        )
        abnormal = violations["abnormal"]
        crossed_stop_line = violations["stop_crossing"]

        stop_valid = np.isfinite(stop_line)
        signal_colors = {
            "red": RED,
            "yellow": "#ca8a04",
        }
        for signal, color in signal_colors.items():
            signal_mask = stop_valid & (traffic_state == signal)
            if not np.any(signal_mask):
                continue
            self.ax_st.plot(
                times,
                np.ma.masked_where(~signal_mask, stop_line),
                "-.",
                linewidth=1.35,
                color=color,
                label=f"{signal.title()} stop line",
                zorder=2,
            )

        self.ax_st.plot(
            times[valid],
            distance[valid],
            linewidth=1.7,
            color=BLUE,
            label="Actual longitudinal progress",
            zorder=3,
        )
        if np.any(abnormal):
            self.ax_st.plot(
                times,
                np.ma.masked_where(~abnormal, distance),
                linewidth=2.4,
                linestyle="--",
                color=DARK,
                label="Outside expected channel",
                zorder=5,
            )
        current_index = int(np.where(valid)[0][-1])
        current_signal = str(traffic_state[current_index]).upper()
        if not bool(evaluation_enabled[current_index]):
            state = "WAITING FOR LEGAL SPEED"
        elif crossed_stop_line[current_index]:
            state = "STOP-LINE VIOLATION"
        elif violations["speed_slow"][current_index]:
            state = "SPEED TOO SLOW"
        elif violations["speed_fast"][current_index]:
            state = "SPEED TOO FAST"
        elif violations["position_fast"][current_index]:
            state = "PROGRESS TOO FAST"
        elif following[current_index]:
            current_gap = lead_gaps[current_index]
            gap_text = f", gap={current_gap:.1f} m" if np.isfinite(current_gap) else ""
            state = f"FOLLOWING LEAD VEHICLE{gap_text}"
        elif (
            violations["position_slow"][current_index]
        ):
            state = "PROGRESS TOO SLOW"
        elif current_signal in {"RED", "YELLOW"}:
            state = f"{current_signal}: STOP BEFORE LINE"
        elif current_signal == "GREEN":
            state = "GREEN: PROCEED"
        else:
            state = "NORMAL"
        current_speed = speed[current_index] if np.isfinite(speed[current_index]) else float("nan")
        self.ax_st.set_title(
            f"ST — {state} (s={distance[current_index]:.1f} m, v={current_speed:.2f} m/s)",
            fontproperties=EN_TITLE_FONT,
            pad=12,
        )
        self.ax_st.legend(
            loc="upper left", bbox_to_anchor=(0.01, 0.99), borderaxespad=0.0,
            prop=EN_LEGEND_FONT
        )
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
        self.ax_sl.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_sl.set_ylabel(
            r"Lateral offset $\Delta l$ (m)",
            fontproperties=EN_LABEL_FONT,
            labelpad=8,
        )
        self.ax_sl.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_sl, -self.sl_axis_abs_limit, self.sl_axis_abs_limit
        )
        self.style_axis_text(self.ax_sl)
        if not enabled:
            self.ax_sl.set_title(
                "SL — NOT EVALUATED", fontproperties=EN_TITLE_FONT, pad=12
            )
            self.ax_sl.text(
                0.5,
                0.5,
                mode_label,
                ha="center",
                va="center",
                transform=self.ax_sl.transAxes,
                fontproperties=EN_ANNOTATION_FONT,
            )
            return False
        valid = np.isfinite(lateral)
        if not np.any(valid):
            self.ax_sl.set_title(
                "SL — building route baseline",
                fontproperties=EN_TITLE_FONT,
                pad=12,
            )
            return False
        channel_end = max(float(times[np.where(valid)[0][-1]]), 0.0)
        channel_times = np.asarray([0.0, channel_end])
        self.ax_sl.fill_between(
            channel_times,
            -self.sl_limit,
            self.sl_limit,
            color=GREEN_FILL,
            alpha=0.82,
            label="Expected straight-road channel",
        )
        self.ax_sl.plot(
            channel_times,
            np.full(2, self.sl_limit),
            linestyle="--",
            linewidth=1.1,
            color=GREEN,
        )
        self.ax_sl.plot(
            channel_times,
            np.full(2, -self.sl_limit),
            linestyle="--",
            linewidth=1.1,
            color=GREEN,
        )
        abnormal = valid & (np.abs(lateral) > self.sl_limit)
        self.plot_abnormal_overlay(
            self.ax_sl, times, lateral, valid, abnormal, "Actual lateral offset"
        )
        current_index = int(np.where(valid)[0][-1])
        state = "VIOLATION" if abnormal[current_index] else "NORMAL"
        self.ax_sl.set_title(
            f"SL — {state} ({lateral[current_index]:.2f} m)",
            fontproperties=EN_TITLE_FONT,
            pad=12,
        )
        self.ax_sl.legend(
            loc="upper left", bbox_to_anchor=(0.01, 0.99), borderaxespad=0.0,
            prop=EN_LEGEND_FONT
        )
        return bool(abnormal[current_index])

    def draw_at(self, times: np.ndarray, acceleration: np.ndarray) -> bool:
        self.ax_at.clear()
        self.ax_at.set_facecolor("white")
        self.ax_at.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_at.set_ylabel(
            "Acceleration (m/s²)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_at.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_at, -self.at_axis_abs_limit, self.at_axis_abs_limit
        )
        self.style_axis_text(self.ax_at)
        valid = np.isfinite(acceleration)
        if not np.any(valid):
            self.ax_at.set_title(
                "AT — waiting for acceleration",
                fontproperties=EN_TITLE_FONT,
                pad=12,
            )
            return False
        channel_end = max(float(times[np.where(valid)[0][-1]]), 0.0)
        channel_times = np.asarray([0.0, channel_end])
        self.ax_at.fill_between(
            channel_times,
            -self.max_accel,
            self.max_accel,
            color=GREEN_FILL,
            alpha=0.82,
            label="Expected acceleration channel",
        )
        self.ax_at.plot(
            channel_times,
            np.full(2, self.max_accel),
            linestyle="--",
            linewidth=1.1,
            color=GREEN,
        )
        self.ax_at.plot(
            channel_times,
            np.full(2, -self.max_accel),
            linestyle="--",
            linewidth=1.1,
            color=GREEN,
        )
        abnormal = valid & (np.abs(acceleration) > self.max_accel)
        self.plot_abnormal_overlay(
            self.ax_at, times, acceleration, valid, abnormal, "Actual acceleration"
        )
        current_index = int(np.where(valid)[0][-1])
        state = "VIOLATION" if abnormal[current_index] else "NORMAL"
        self.ax_at.set_title(
            f"AT — {state} ({acceleration[current_index]:.2f} m/s²)",
            fontproperties=EN_TITLE_FONT,
            pad=12,
        )
        self.ax_at.legend(
            loc="upper left", bbox_to_anchor=(0.01, 0.99), borderaxespad=0.0,
            prop=EN_LEGEND_FONT
        )
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
            va="top",
            fontproperties=CN_RESULT_TITLE_FONT,
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
                fontweight="bold",
                color=text_color,
                fontproperties=CN_RESULT_FONT,
                bbox={
                    "boxstyle": "round,pad=0.55",
                    "facecolor": face,
                    "edgecolor": face,
                    "linewidth": 1.5,
                },
            )

    def draw_expected_st(self, data: Dict[str, object]) -> None:
        """Draw the generated longitudinal channel without identification output."""
        times = data["time"]
        distance = data["distance"]
        speed_eval = data["speed_eval"]
        lower = data["st_lower"]
        upper = data["st_upper"]
        segments = data["st_segments"]

        self.ax_st.clear()
        self.ax_st.set_facecolor("white")
        self.ax_st.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_st.set_ylabel(
            "Longitudinal displacement s (m)",
            fontproperties=EN_LABEL_FONT,
            labelpad=8,
        )
        self.ax_st.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_st, self.st_axis_min_distance, self.st_axis_max_distance
        )
        self.style_axis_text(self.ax_st)

        valid = np.isfinite(distance)
        active = valid & speed_eval & np.isfinite(lower) & np.isfinite(upper)
        if np.any(active):
            channel_times = times[active]
            channel_lower = lower[active]
            channel_upper = upper[active]
            if segments:
                latest = segments[-1]
                future_stamp = latest.start_stamp + self.st_segment_seconds
                future_time = future_stamp - data["first_stamp"]
                if future_time > channel_times[-1] + 1e-6:
                    future_lower, future_upper = self.segment_bounds(
                        latest, np.asarray([future_stamp])
                    )
                    channel_times = np.append(channel_times, future_time)
                    channel_lower = np.append(channel_lower, future_lower[0])
                    channel_upper = np.append(channel_upper, future_upper[0])
            self.ax_st.fill_between(
                channel_times,
                channel_lower,
                channel_upper,
                color=GREEN_FILL,
                alpha=0.82,
                label="Generated expected S-T channel",
                zorder=0,
            )
            self.ax_st.plot(
                channel_times, channel_lower, "--", linewidth=1.5, color=GREEN
            )
            self.ax_st.plot(
                channel_times, channel_upper, "--", linewidth=1.5, color=GREEN
            )
        if np.any(valid):
            self.ax_st.plot(
                times[valid],
                distance[valid],
                linewidth=2.0,
                color=BLUE,
                label="Real-time vehicle state",
                zorder=3,
            )
        self.ax_st.set_title(
            "ST expected channel", fontproperties=EN_TITLE_FONT, pad=12
        )
        if self.ax_st.get_legend_handles_labels()[0]:
            self.ax_st.legend(loc="upper left", prop=EN_LEGEND_FONT)

    def draw_expected_sl(self, data: Dict[str, object]) -> None:
        """Draw the generated lateral channel without violation classification."""
        times = data["time"]
        lateral = data["lateral"]
        enabled = bool(data["sl_enabled"])

        self.ax_sl.clear()
        self.ax_sl.set_facecolor("white")
        self.ax_sl.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_sl.set_ylabel(
            r"Lateral offset $\Delta l$ (m)",
            fontproperties=EN_LABEL_FONT,
            labelpad=8,
        )
        self.ax_sl.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_sl, -self.sl_axis_abs_limit, self.sl_axis_abs_limit
        )
        self.style_axis_text(self.ax_sl)

        valid = np.isfinite(lateral)
        if enabled and np.any(valid):
            channel_end = max(float(times[np.where(valid)[0][-1]]), 0.0)
            channel_times = np.asarray([0.0, channel_end])
            self.ax_sl.fill_between(
                channel_times,
                -self.sl_limit,
                self.sl_limit,
                color=GREEN_FILL,
                alpha=0.82,
                label="Generated expected S-L channel",
            )
            for boundary in (-self.sl_limit, self.sl_limit):
                self.ax_sl.plot(
                    channel_times,
                    np.full(2, boundary),
                    linestyle="--",
                    linewidth=1.5,
                    color=GREEN,
                )
            self.ax_sl.plot(
                times[valid],
                lateral[valid],
                linewidth=2.0,
                color=BLUE,
                label="Real-time vehicle state",
                zorder=3,
            )
        elif not enabled:
            self.ax_sl.text(
                0.5,
                0.5,
                "Not applicable to this route",
                ha="center",
                va="center",
                transform=self.ax_sl.transAxes,
                fontproperties=EN_ANNOTATION_FONT,
            )
        self.ax_sl.set_title(
            "SL expected channel", fontproperties=EN_TITLE_FONT, pad=12
        )
        if self.ax_sl.get_legend_handles_labels()[0]:
            self.ax_sl.legend(loc="upper left", prop=EN_LEGEND_FONT)

    def draw_expected_at(self, data: Dict[str, object]) -> None:
        """Draw the generated acceleration channel without violation classification."""
        times = data["time"]
        acceleration = data["acceleration"]

        self.ax_at.clear()
        self.ax_at.set_facecolor("white")
        self.ax_at.set_xlabel(
            "Time from channel start (s)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_at.set_ylabel(
            "Acceleration (m/s²)", fontproperties=EN_LABEL_FONT, labelpad=8
        )
        self.ax_at.grid(True, linestyle="--", alpha=0.22)
        self.apply_fixed_axes(
            self.ax_at, -self.at_axis_abs_limit, self.at_axis_abs_limit
        )
        self.style_axis_text(self.ax_at)

        valid = np.isfinite(acceleration)
        if np.any(valid):
            channel_end = max(float(times[np.where(valid)[0][-1]]), 0.0)
            channel_times = np.asarray([0.0, channel_end])
            self.ax_at.fill_between(
                channel_times,
                -self.max_accel,
                self.max_accel,
                color=GREEN_FILL,
                alpha=0.82,
                label="Generated expected A-T channel",
            )
            for boundary in (-self.max_accel, self.max_accel):
                self.ax_at.plot(
                    channel_times,
                    np.full(2, boundary),
                    linestyle="--",
                    linewidth=1.5,
                    color=GREEN,
                )
            self.ax_at.plot(
                times[valid],
                acceleration[valid],
                linewidth=2.0,
                color=BLUE,
                label="Real-time vehicle state",
                zorder=3,
            )
        self.ax_at.set_title(
            "AT expected channel", fontproperties=EN_TITLE_FONT, pad=12
        )
        if self.ax_at.get_legend_handles_labels()[0]:
            self.ax_at.legend(loc="upper left", prop=EN_LEGEND_FONT)

    def render_expected_channel_gif_frame(
        self, data: Dict[str, object]
    ) -> Optional[np.ndarray]:
        if (
            self.expected_channel_gif_fig is None
            or len(self.expected_channel_gif_axes) != 3
        ):
            return None
        display_axes = (self.ax_st, self.ax_sl, self.ax_at)
        self.ax_st, self.ax_sl, self.ax_at = self.expected_channel_gif_axes
        try:
            self.draw_expected_st(data)
            self.draw_expected_sl(data)
            self.draw_expected_at(data)
            self.expected_channel_gif_fig.canvas.draw()
            frame = np.asarray(
                self.expected_channel_gif_fig.canvas.buffer_rgba(), dtype=np.uint8
            )[:, :, :3]
            return np.ascontiguousarray(frame)
        finally:
            self.ax_st, self.ax_sl, self.ax_at = display_axes

    def render_channel_gif_frame(self, data: Dict[str, object]) -> Optional[np.ndarray]:
        if self.channel_gif_fig is None or len(self.channel_gif_axes) != 3:
            return None
        display_axes = (self.ax_st, self.ax_sl, self.ax_at)
        self.ax_st, self.ax_sl, self.ax_at = self.channel_gif_axes
        try:
            self.draw_st(
                data["time"],
                data["distance"],
                data["speed"],
                data["speed_eval"],
                data["st_lower"],
                data["st_upper"],
                data["stop_line"],
                data["traffic_state"],
                data["st_segments"],
                data["first_stamp"],
                data.get("following"),
                data.get("lead_gap"),
            )
            self.draw_sl(
                data["time"],
                data["lateral"],
                bool(data["sl_enabled"]),
                str(data["sl_mode_label"]),
            )
            self.draw_at(data["time"], data["acceleration"])
            self.channel_gif_fig.canvas.draw()
            frame = np.asarray(
                self.channel_gif_fig.canvas.buffer_rgba(), dtype=np.uint8
            )[:, :, :3]
            return np.ascontiguousarray(frame)
        finally:
            self.ax_st, self.ax_sl, self.ax_at = display_axes

    def start_synchronized_gif_writers(
        self, camera_frame: np.ndarray, channel_frame: np.ndarray
    ) -> None:
        if (
            not self.save_gif
            or self.camera_gif_writer is not None
            or self.channel_gif_writer is not None
        ):
            return
        scenario_name = Path(self.scenario_path).stem or "scenario"
        session_name = f"{scenario_name}_{time.strftime('%Y%m%d_%H%M%S')}"
        camera_path = self.gif_output_dir / f"{session_name}_camera.gif"
        channel_path = self.gif_output_dir / f"{session_name}_channels.gif"
        expected_path = self.gif_output_dir / f"{session_name}_expected_channels.gif"
        camera_height, camera_width = camera_frame.shape[:2]
        channel_height, channel_width = channel_frame.shape[:2]
        try:
            self.camera_gif_writer = GifWriter(
                camera_path,
                camera_width,
                camera_height,
                self.gif_fps,
                self.ffmpeg_bin,
            )
            self.channel_gif_writer = GifWriter(
                channel_path,
                channel_width,
                channel_height,
                self.gif_fps,
                self.ffmpeg_bin,
            )
            if self.show_all_gifs:
                self.expected_channel_gif_writer = GifWriter(
                    expected_path,
                    channel_width,
                    channel_height,
                    self.gif_fps,
                    self.ffmpeg_bin,
                )
            rospy.loginfo(
                "Synchronized GIF recorders opened: camera=%s (%dx%d), "
                "channels=%s (%dx%d), expected=%s, fps=%.1f",
                camera_path,
                camera_width,
                camera_height,
                channel_path,
                channel_width,
                channel_height,
                expected_path if self.show_all_gifs else "disabled",
                self.gif_fps,
            )
        except OSError as exc:
            self.gif_recording_failed = True
            rospy.logerr("Cannot start synchronized GIF recorders: %s", exc)

    def write_synchronized_gif_pair(
        self,
        camera_frame: np.ndarray,
        channel_frame: np.ndarray,
        data: Dict[str, object],
        frame_time: float,
    ) -> bool:
        camera_writer = self.camera_gif_writer
        channel_writer = self.channel_gif_writer
        expected_writer = self.expected_channel_gif_writer
        if (
            self.gif_recording_failed
            or camera_writer is None
            or channel_writer is None
            or (self.show_all_gifs and expected_writer is None)
        ):
            return False
        if not camera_writer.can_write(camera_frame) or not channel_writer.can_write(
            channel_frame
        ):
            self.gif_recording_failed = True
            rospy.logerr(
                "Synchronized GIF recording stopped before an unpaired frame was written."
            )
            return False
        expected_frame = None
        if self.show_all_gifs:
            expected_frame = self.render_expected_channel_gif_frame(data)
            if (
                expected_frame is None
                or expected_writer is None
                or not expected_writer.can_write(expected_frame)
            ):
                self.gif_recording_failed = True
                rospy.logerr(
                    "Synchronized expected-channel GIF frame was not writable."
                )
                return False
        camera_ok = camera_writer.write(camera_frame, frame_time)
        channel_ok = channel_writer.write(channel_frame, frame_time)
        expected_ok = True
        if self.show_all_gifs:
            expected_ok = (
                expected_frame is not None
                and expected_writer is not None
                and expected_writer.write(expected_frame, frame_time)
            )
        synchronized = (
            camera_ok
            and channel_ok
            and expected_ok
            and camera_writer.frame_count == channel_writer.frame_count
            and (
                not self.show_all_gifs
                or expected_writer is not None
                and expected_writer.frame_count == camera_writer.frame_count
            )
        )
        if not synchronized:
            self.gif_recording_failed = True
            rospy.logerr(
                "Synchronized GIF recording failed: camera_frames=%d channel_frames=%d",
                camera_writer.frame_count,
                channel_writer.frame_count,
            )
        return synchronized

    def record_synchronized_gifs(self, data: Dict[str, object]) -> None:
        if not self.save_gif or self.gif_recording_failed:
            return
        self.follow_camera.ensure(self.target_id)
        now = time.monotonic()
        if now - self.last_gif_frame_wall < 1.0 / self.gif_fps:
            return
        with self.lock:
            if self.scene_frame is None or not self.scene_frame_is_follow_camera:
                return
            camera_frame = np.ascontiguousarray(self.scene_frame)
        sample_times = np.asarray(data.get("time", []), dtype=np.float64)
        frame_time = (
            float(sample_times[-1])
            if len(sample_times) and np.isfinite(sample_times[-1])
            else now
        )
        if (
            self.camera_gif_writer is not None
            and self.camera_gif_writer.frame_timestamps
            and frame_time <= self.camera_gif_writer.frame_timestamps[-1]
        ):
            return
        channel_frame = self.render_channel_gif_frame(data)
        if channel_frame is None:
            return
        if self.camera_gif_writer is None and self.channel_gif_writer is None:
            self.start_synchronized_gif_writers(camera_frame, channel_frame)
        if self.write_synchronized_gif_pair(
            camera_frame, channel_frame, data, frame_time
        ):
            if self.gif_recording_started_wall is None:
                self.gif_recording_started_wall = now
            self.last_gif_frame_wall = now
            rospy.loginfo_throttle(
                5.0,
                "Synchronized GIF recording active: %d paired frames",
                self.camera_gif_writer.frame_count,
            )

    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return
        try:
            if self.show_window:
                self.draw_scene()
                speed_abnormal = self.draw_st(
                    data["time"],
                    data["distance"],
                    data["speed"],
                    data["speed_eval"],
                    data["st_lower"],
                    data["st_upper"],
                    data["stop_line"],
                    data["traffic_state"],
                    data["st_segments"],
                    data["first_stamp"],
                    data.get("following"),
                    data.get("lead_gap"),
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
                    f"Target vehicle: {data['target_role']} / ID {data['target_id']}",
                    fontproperties=EN_FONT,
                    color=DARK,
                )
                self.fig.canvas.draw()
            self.record_synchronized_gifs(data)
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
        camera_writer = self.camera_gif_writer
        channel_writer = self.channel_gif_writer
        expected_writer = self.expected_channel_gif_writer
        recording_duration = None
        if self.gif_recording_started_wall is not None:
            recording_duration = max(
                time.monotonic() - self.gif_recording_started_wall,
                0.0,
            )
        self.camera_gif_writer = None
        self.channel_gif_writer = None
        self.expected_channel_gif_writer = None
        if camera_writer is not None and channel_writer is not None:
            paired_frames = min(
                camera_writer.frame_count, channel_writer.frame_count
            )
            playback_duration = self.gif_playback_duration_seconds
            if playback_duration <= 0.0:
                playback_duration = recording_duration
            rospy.loginfo(
                "Finalizing synchronized camera/channel GIFs with %d paired frames "
                "over %.2f s (playback %.2f s)...",
                paired_frames,
                recording_duration or 0.0,
                playback_duration or 0.0,
            )
            camera_path = camera_writer.close(playback_duration)
            channel_path = channel_writer.close(playback_duration)
            expected_path = (
                expected_writer.close(playback_duration)
                if expected_writer is not None
                else None
            )
            if (
                camera_path is None
                or channel_path is None
                or (self.show_all_gifs and expected_path is None)
                or camera_writer.frame_count != channel_writer.frame_count
                or (
                    self.show_all_gifs
                    and expected_writer is not None
                    and expected_writer.frame_count != camera_writer.frame_count
                )
            ):
                rospy.logerr(
                    "Synchronized GIF output is incomplete: camera_frames=%d "
                    "channel_frames=%d expected_frames=%d",
                    camera_writer.frame_count,
                    channel_writer.frame_count,
                    expected_writer.frame_count if expected_writer is not None else 0,
                )
                if camera_path is not None:
                    GifWriter._unlink(camera_path)
                if channel_path is not None:
                    GifWriter._unlink(channel_path)
                if expected_path is not None:
                    GifWriter._unlink(expected_path)
        elif camera_writer is not None or channel_writer is not None or expected_writer is not None:
            writers = [writer for writer in (camera_writer, channel_writer, expected_writer) if writer is not None]
            rospy.logerr("Incomplete synchronized GIF output; discarding partial files.")
            for writer in writers:
                partial_path = writer.close()
                if partial_path is not None:
                    GifWriter._unlink(partial_path)
        elif self.save_gif:
            rospy.logwarn(
                "No synchronized GIF pair was created because no complete camera/channel "
                "frame pair was rendered."
            )
        self.follow_camera.close()
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
