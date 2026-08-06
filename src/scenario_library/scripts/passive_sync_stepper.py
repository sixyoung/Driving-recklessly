#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
passive_sync_stepper.py
在 ros_bridge 被动模式下，以固定频率（墙钟）驱动 CARLA 世界 tick：
- 将世界设置为 synchronous_mode=True, fixed_delta_seconds=~fixed_dt
- 以墙钟 ~rate Hz 调用 world.tick()（不依赖 /clock）
- 可选将 Traffic Manager 切到同步
- 退出时（非 ~no_restore）恢复原设置
"""

import time
import threading
import traceback

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

import carla


class PassiveSyncStepper(CompatibleNode):
    def __init__(self):
        super(PassiveSyncStepper, self).__init__("passive_sync_stepper")

        # --------- 读取参数 ---------
        self.host       = self.get_param("host", "localhost")
        self.port       = int(self.get_param("port", 2000))
        self.rate_hz    = float(self.get_param("rate", 20.0))
        self.fixed_dt   = float(self.get_param("fixed_dt", 0.05))
        self.timeout    = float(self.get_param("timeout", 5.0))
        self.tm_port    = int(self.get_param("tm_port", 8000))
        self.tm_sync    = bool(self.get_param("tm_sync", True))
        self.no_restore = bool(self.get_param("no_restore", False))
        self.passive    = bool(self.get_param("passive", False))

        # 启动横幅 + 参数总览
        sim_scale = self.rate_hz * self.fixed_dt  # 仿真时间倍率（s_sim / s_wall）
        self.loginfo(
            "\n================= Passive Sync Stepper =================\n"
            f"  host:port      = {self.host}:{self.port}\n"
            f"  rate (Hz)      = {self.rate_hz}\n"
            f"  fixed_dt (s)   = {self.fixed_dt}\n"
            f"  tm_port        = {self.tm_port}   tm_sync={self.tm_sync}\n"
            f"  timeout (s)    = {self.timeout}\n"
            f"  restore on exit= {not self.no_restore}\n"
            f"  passive        = {self.passive}\n"
            f"  sim speed scale= {sim_scale:.3f}x (rate*fixed_dt)\n"
            "=======================================================\n"
        )

        if self.rate_hz <= 0:
            self.logfatal("❌ ~rate 必须 > 0")
            raise RuntimeError("invalid ~rate")
        if self.fixed_dt <= 0:
            self.logwarn(f"⚠️  ~fixed_dt={self.fixed_dt:.6f}（可变步长更抖动）。建议与 ~rate 对齐，如 20Hz→0.05s")
        if abs(sim_scale - 1.0) > 1e-3:
            self.logwarn(f"⚠️  仿真时间将以 {sim_scale:.3f}x 速度相对墙钟推进 (rate * fixed_dt ≠ 1.0)")
        else:
            self.loginfo("✅ 仿真时间≈墙钟 (rate * fixed_dt ≈ 1.0)")

        # 如果桥不是被动模式，提示可能冲突
        try:
            if not bool(self.get_param("/passive", True)):
                self.logwarn("⚠️  检测到 /passive=false：ros_bridge 可能也在 tick。请确保**只有一个**tick 驱动者！")
        except Exception:
            pass

        # --------- 连接 CARLA ---------
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        try:
            self.world = self.client.get_world()
        except Exception as e:
            self.logfatal(f"❌ 连接 CARLA 失败：{e}")
            raise

        try:
            cv = self.client.get_client_version()
            sv = self.client.get_server_version()
            self.loginfo(f"CARLA 版本：client={cv}  server={sv}")
            if cv != sv:
                self.logwarn("⚠️  Client/Server 版本不一致，可能导致 API 行为差异")
        except Exception:
            pass

        # 记录并应用同步设置
        self._original_settings = self.world.get_settings()
        try:
            s = self.world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = self.fixed_dt
            self.world.apply_settings(s)
            eff = self.world.get_settings()
            self.loginfo(f"已应用同步设置：sync={eff.synchronous_mode} fixed_dt={(eff.fixed_delta_seconds or 0.0):.6f}")
        except Exception as e:
            self.logfatal(f"❌ 设置世界同步失败：{e}")
            raise

        # （可选）同步 Traffic Manager
        self.tm = None
        if self.tm_sync:
            try:
                self.tm = self.client.get_trafficmanager(self.tm_port)
                self.tm.set_synchronous_mode(True)
                self.loginfo(f"Traffic Manager(port {self.tm_port}) sync=True")
            except Exception as e:
                self.logwarn(f"⚠️  Traffic Manager 同步失败（可忽略）：{e}")

        # 期望设置（用于守护纠偏）
        self._desired_sync = True
        self._desired_fixed = self.fixed_dt
        self._last_check = time.perf_counter()

        # --------- 启动墙钟 tick 线程 ---------
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._tick_loop, name="passive_stepper", daemon=True)
        self._thread.start()

    # 墙钟驱动的 tick 循环
    def _tick_loop(self):
        dt_wall = 1.0 / self.rate_hz
        next_t = time.perf_counter()
        frames = 0
        t0 = time.perf_counter()
        self.loginfo(f"▶️  开始以 ~{self.rate_hz:.2f} Hz 驱动 world.tick()（墙钟）")

        while not self._stop.is_set() and roscomp.ok():
            loop_t0 = time.perf_counter()
            try:
                frame = self.world.tick()   # 推进一帧
                frames += 1

                # 每 ~5s 打印一次状态
                if frames % max(1, int(self.rate_hz * 5)) == 0:
                    ts = self.world.get_snapshot().timestamp
                    dt_eff = ts.delta_seconds
                    self.loginfo(f"⏱️  frame={frame} sim_t={ts.elapsed_seconds:.3f}s dt={dt_eff:.3f}s")
                    if abs(dt_eff - self.fixed_dt) > 1e-6:
                        self.logwarn(f"⚠️  观察到 delta_seconds={dt_eff:.6f} 与期望 fixed_dt={self.fixed_dt:.6f} 不一致")

            except Exception as e:
                # 避免刷屏
                self.logerr_throttle(5.0, f"world.tick() 异常：{e}\n{traceback.format_exc()}")
                time.sleep(0.1)

            # 每 1s 守护一次设置，防止 load_world() 重置
            now = time.perf_counter()
            if now - self._last_check > 1.0:
                self._last_check = now
                try:
                    s = self.world.get_settings()
                    if (s.synchronous_mode != self._desired_sync) or \
                       (abs((s.fixed_delta_seconds or 0.0) - self._desired_fixed) > 1e-9):
                        self.logwarn(
                            f"⚠️  发现世界设置被修改（sync={s.synchronous_mode} fixed_dt={s.fixed_delta_seconds}），"
                            f"正在纠正为 (True, {self._desired_fixed:.6f})"
                        )
                        s.synchronous_mode = self._desired_sync
                        s.fixed_delta_seconds = self._desired_fixed
                        self.world.apply_settings(s)
                        self.loginfo(f"✅ 已重新应用设置：sync=True fixed_dt={self._desired_fixed:.6f}")
                except Exception as e:
                    self.logwarn(f"检查/纠正设置失败（忽略，下次再试）：{e}")

            # 精准节拍（累加法避免漂移）
            next_t += dt_wall
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # 落后较多时，重置基准
                next_t = time.perf_counter()

            # 若一帧阻塞异常长，给出告警（例如仿真卡顿）
            spent = time.perf_counter() - loop_t0
            if spent > max(0.5, 3 * dt_wall):
                self.logwarn(f"⚠️  world.tick() 耗时 {spent:.3f}s，明显高于期望步长 {dt_wall:.3f}s（可能计算负载过高）")

        # 退出统计
        t1 = time.perf_counter()
        if t1 > t0 and frames > 0:
            self.loginfo(f"⏹️  停止。共推进 {frames} 帧，用时 {t1 - t0:.1f}s，平均墙钟频率 ~{frames / (t1 - t0):.1f} Hz")

    def destroy(self):
        # 停线程
        try:
            if hasattr(self, "_stop"):
                self._stop.set()
            if hasattr(self, "_thread") and self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass

        # 关闭 TM 同步
        if self.tm and self.tm_sync:
            try:
                self.tm.set_synchronous_mode(False)
                self.loginfo("Traffic Manager sync disabled.")
            except Exception:
                pass

        # 恢复原设置
        if not self.no_restore:
            try:
                self.world.apply_settings(self._original_settings)
                self.loginfo("🔁 已恢复启动前的世界设置。")
            except Exception:
                pass

        super(PassiveSyncStepper, self).destroy()


def main(args=None):
    roscomp.init("passive_sync_stepper", args=args)
    try:
        node = PassiveSyncStepper()
        roscomp.on_shutdown(node.destroy)
        node.spin()
    except Exception as e:
        roscomp.logerr(f"passive_sync_stepper 初始化失败：{e}")
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
