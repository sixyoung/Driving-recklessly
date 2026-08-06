#!/usr/bin/env python3
"""Expected-channel monitor: CARLA scene + ST speed + straight-road SL + AT acceleration."""

from __future__ import annotations

import atexit
import signal
import time
from typing import Optional

import numpy as np
import rospy

from driver_models_types.msg import VehicleConfig
from realtime_channel_visualizer_row import Monitor as BaseMonitor


class ExpectedChannelMonitor(BaseMonitor):
    """Display expected bands while limiting SL judgement to straight roads."""

    def __init__(self) -> None:
        # BaseMonitor subscribes during construction, so callback state must
        # exist before super().__init__() is entered.
        self.sl_enabled = False
        self.sl_mode_label = "waiting for vehicle configuration"

        super().__init__()

        self.min_speed = float(rospy.get_param("~slow_speed_threshold", 6.0))
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
            "SL — straight-road lateral channel",
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

    def config_cb(self, msg: VehicleConfig) -> None:
        previous = self.target_config
        super().config_cb(msg)
        if self.target_config is previous or self.target_config is None:
            return

        option = int(getattr(self.target_config, "road_option", -1))
        description = " ".join(
            (
                str(getattr(self.target_config, "scene_type", "")),
                str(getattr(self.target_config, "scene_class", "")),
                str(getattr(self.target_config, "spawn_key", "")),
            )
        ).lower()

        # VehicleConfig convention: 0 straight, 1 left turn, 2 right turn.
        # Unknown option is treated as straight unless the scenario text
        # explicitly identifies a turn, which preserves older straight scenes.
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
            self.sl_mode_label = "left-turn scene: SL judgement disabled"
        elif option == 2 or "right" in description or "右转" in description:
            self.sl_mode_label = "right-turn scene: SL judgement disabled"
        else:
            self.sl_mode_label = "non-straight scene: SL judgement disabled"

        rospy.loginfo(
            "SL mode for %s (%d): enabled=%d, road_option=%d, %s",
            self.target_config.role_name,
            self.target_config.carla_id,
            int(self.sl_enabled),
            option,
            self.sl_mode_label,
        )

    def snapshot(self):
        data = super().snapshot()
        if data is None:
            return None
        data["sl_enabled"] = self.sl_enabled
        data["sl_mode_label"] = self.sl_mode_label
        return data

    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return

        try:
            self.draw_scene()
            self.draw_st_speed(data["t"], data["speed"])
            self.draw_sl_straight(
                data["t"],
                data["lateral"],
                bool(data["sl_enabled"]),
                str(data["sl_mode_label"]),
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
            self.min_speed,
            linestyle="--",
            linewidth=1.1,
            label=f"Lower {self.min_speed:.1f}",
        )
        self.ax_st.axhline(
            self.max_speed,
            linestyle="--",
            linewidth=1.1,
            label=f"Upper {self.max_speed:.1f}",
        )

        valid = np.isfinite(speed)
        if not np.any(valid):
            self.ax_st.set_title("ST — waiting for speed")
            return

        self.ax_st.plot(t[valid], speed[valid], linewidth=2.0, label="Actual speed")
        too_slow = valid & (speed < self.min_speed)
        too_fast = valid & (speed > self.max_speed)
        if np.any(too_slow):
            self.ax_st.scatter(
                t[too_slow], speed[too_slow], marker="v", s=24, label="Too slow"
            )
        if np.any(too_fast):
            self.ax_st.scatter(
                t[too_fast], speed[too_fast], marker="^", s=24, label="Too fast"
            )

        current = float(speed[valid][-1])
        if current < self.min_speed:
            state = "TOO SLOW"
        elif current > self.max_speed:
            state = "TOO FAST"
        else:
            state = "NORMAL"
        self.ax_st.set_title(f"ST — {state} ({current:.2f} m/s)")
        self.ax_st.legend(loc="best", fontsize=7)

    def draw_sl_straight(
        self,
        t: np.ndarray,
        lateral: np.ndarray,
        enabled: bool,
        mode_label: str,
    ) -> None:
        self.ax_sl.clear()
        self.ax_sl.set_xlabel("Time (s)")
        self.ax_sl.set_ylabel(r"Lateral offset $\Delta l$ (m)")
        self.ax_sl.grid(True, linestyle="--", alpha=0.25)

        if not enabled:
            self.ax_sl.set_title("SL — NOT EVALUATED")
            self.ax_sl.text(
                0.5,
                0.5,
                mode_label,
                ha="center",
                va="center",
                transform=self.ax_sl.transAxes,
            )
            return

        self.ax_sl.axhspan(
            -self.sl_limit,
            self.sl_limit,
            alpha=0.16,
            label="Expected straight-road channel",
        )
        self.ax_sl.axhline(self.sl_limit, linestyle="--", linewidth=1.1)
        self.ax_sl.axhline(-self.sl_limit, linestyle="--", linewidth=1.1)

        valid = np.isfinite(lateral)
        if not np.any(valid):
            self.ax_sl.set_title("SL — building straight-route baseline")
            return

        self.ax_sl.plot(
            t[valid], lateral[valid], linewidth=2.0, label="Actual lateral offset"
        )
        abnormal = valid & (np.abs(lateral) > self.sl_limit)
        if np.any(abnormal):
            self.ax_sl.scatter(
                t[abnormal],
                lateral[abnormal],
                marker="x",
                s=28,
                label="Outside channel",
            )

        current = float(lateral[valid][-1])
        state = "VIOLATION" if abs(current) > self.sl_limit else "NORMAL"
        self.ax_sl.set_title(f"SL — {state} ({current:.2f} m)")
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

        self.ax_at.plot(
            t[valid], accel[valid], linewidth=2.0, label="Actual acceleration"
        )
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
