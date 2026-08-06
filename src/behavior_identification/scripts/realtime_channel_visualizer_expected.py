#!/usr/bin/env python3
"""Expected-channel view: CARLA scene + ST speed + SL lateral + AT acceleration."""

from __future__ import annotations

import atexit
import os
import signal
import time

import numpy as np
import rospy

from realtime_channel_visualizer_row import Monitor as BaseMonitor


class ExpectedChannelMonitor(BaseMonitor):
    """Use the existing robust data/capture pipeline with true expected bands."""

    def __init__(self) -> None:
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
            "SL — lateral expected channel",
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

    def draw(self) -> None:
        data = self.snapshot()
        if data is None:
            self.pump_gui()
            return

        try:
            self.draw_scene()
            self.draw_st_speed(data["t"], data["speed"])
            self.draw_sl(data["t"], data["lateral"])
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
            label=f"Lower bound {self.min_speed:.1f}",
        )
        self.ax_st.axhline(
            self.max_speed,
            linestyle="--",
            linewidth=1.1,
            label=f"Upper bound {self.max_speed:.1f}",
        )

        valid = np.isfinite(speed)
        if not np.any(valid):
            self.ax_st.set_title("ST — waiting for speed")
            self.ax_st.text(
                0.5,
                0.5,
                "No valid speed samples",
                ha="center",
                va="center",
                transform=self.ax_st.transAxes,
            )
            return

        self.ax_st.plot(t[valid], speed[valid], linewidth=2.0, label="Actual speed")
        low = valid & (speed < self.min_speed)
        high = valid & (speed > self.max_speed)
        if np.any(low):
            self.ax_st.scatter(t[low], speed[low], marker="v", s=24, label="Too slow")
        if np.any(high):
            self.ax_st.scatter(t[high], speed[high], marker="^", s=24, label="Too fast")

        current = float(speed[valid][-1])
        if current < self.min_speed:
            state = "TOO SLOW"
        elif current > self.max_speed:
            state = "TOO FAST"
        else:
            state = "NORMAL"
        self.ax_st.set_title(f"ST — {state} ({current:.2f} m/s)")
        self.ax_st.legend(loc="best", fontsize=7)

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
            self.ax_at.text(
                0.5,
                0.5,
                "No valid acceleration samples",
                ha="center",
                va="center",
                transform=self.ax_at.transAxes,
            )
            return

        self.ax_at.plot(t[valid], accel[valid], linewidth=2.0, label="Actual acceleration")
        abnormal = valid & (np.abs(accel) > self.max_accel)
        if np.any(abnormal):
            self.ax_at.scatter(
                t[abnormal], accel[abnormal], marker="x", s=28, label="Outside channel"
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
