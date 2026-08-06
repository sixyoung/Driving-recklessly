#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synchronous-mode: spawn one vehicle every INTERVAL seconds (SIMULATION time) and set it to autopilot,
until N vehicles are spawned. Prints per-tick wall-clock duration to detect stalls.

Example:
  1) Start CARLA server (FPS suggestion 20):
       ./CarlaUE4.sh -quality-level=Low -fps=20
  2) Run this script:
       python3 carla_spawn_sync_every_5s.py --host 127.0.0.1 --port 2000 --tm-port 8000 \
         --delta 0.05 --interval 5.0 --count 20 --run-for 30 --warn-wall 0.20
"""

import argparse
import random
import sys
import time
from contextlib import contextmanager

try:
    import carla
except ImportError:
    raise RuntimeError("CARLA Python API not found. Ensure the 'carla' Python package matches the server version.")

@contextmanager
def sync_mode(client, world, tm_port, delta_seconds):
    """Enter synchronous mode for both world and Traffic Manager; restore on exit."""
    original_settings = world.get_settings()
    tm = client.get_trafficmanager(tm_port)
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(delta_seconds)
        world.apply_settings(settings)

        tm.set_synchronous_mode(True)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.global_percentage_speed_difference(0.0)
        tm.set_random_device_seed(42)

        yield world, tm
    finally:
        try:
            tm.set_synchronous_mode(False)
        except Exception:
            pass
        world.apply_settings(original_settings)


def pick_vehicle_blueprint(blueprints):
    """Pick a reasonable vehicle blueprint and randomize color when available."""
    candidates = [bp for bp in blueprints if bp.has_attribute('number_of_wheels')]
    bp = random.choice(candidates or list(blueprints))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'autopilot')
    return bp


def tick_and_check(world, warn_wall):
    t0 = time.perf_counter()
    frame = world.tick()                 # 0.9.13: returns int
    dt_wall = time.perf_counter() - t0
    if warn_wall is not None and dt_wall > warn_wall:
        print(f"[Tick] ⚠ slow wall-clock tick: {dt_wall:.3f}s (warn>{warn_wall:.2f}s)")
    snapshot = world.get_snapshot()      # 取 Snapshot
    if hasattr(snapshot, "frame") and snapshot.frame != frame:
        print(f"[Tick] Note: snapshot.frame={snapshot.frame} != tick_frame={frame}")
    return snapshot, dt_wall



def main():
    parser = argparse.ArgumentParser(description="Sync-mode spawn: every INTERVAL sim-seconds, create a vehicle and enable autopilot.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--delta", type=float, default=0.05, help="fixed_delta_seconds for synchronous mode (e.g., 0.05 for 20 FPS)")
    parser.add_argument("--interval", type=float, default=5.0, help="Seconds (SIMULATION time) between spawns")
    parser.add_argument("--count", type=int, default=20, help="Total vehicles to spawn")
    parser.add_argument("--run-for", type=float, default=30.0, help="SIMULATION seconds to keep running after all spawns")
    parser.add_argument("--max-retries", type=int, default=50, help="Max spawn attempts (advances sim to make space)")
    parser.add_argument("--warn-wall", type=float, default=0.20, help="Warn if a tick takes longer than this wall time (seconds)")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
    except RuntimeError as e:
        print(f"[Error] Could not connect to CARLA at {args.host}:{args.port}. Is the server running?\n{e}")
        sys.exit(1)

    blueprints = world.get_blueprint_library().filter("vehicle.*")
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("[Error] Current map has no spawn points.")
        sys.exit(1)

    random.seed(42)
    random.shuffle(spawn_points)

    vehicles = []
    sp_index = 0

    ticks_per_spawn = max(1, int(round(args.interval / args.delta)))
    print(f"[Info] Entering synchronous mode @ delta={args.delta:.3f}s; spacing {args.interval:.2f}s (~{ticks_per_spawn} ticks) between spawns.")

    try:
        with sync_mode(client, world, args.tm_port, args.delta) as (world, tm):
            # Align snapshots
            tick_and_check(world, args.warn_wall)

            for i in range(args.count):
                # Spawn with retries (advance sim time to free spawn points)
                actor = None
                attempt = 0
                while actor is None and attempt < args.max_retries:
                    bp = pick_vehicle_blueprint(blueprints)
                    transform = spawn_points[sp_index % len(spawn_points)]
                    sp_index += 1

                    actor = world.try_spawn_actor(bp, transform)
                    if actor is None:
                        attempt += 1
                        # Let traffic move for ~0.5s to free space
                        for _ in range(max(1, int(round(0.5 / args.delta)))):
                            tick_and_check(world, args.warn_wall)

                if actor is None:
                    print(f"[Warn] Could not spawn vehicle #{i+1} after {args.max_retries} attempts. Skipping.")
                    continue

                actor.set_autopilot(True, tm.get_port())
                vehicles.append(actor)

                # Commit spawn into the next snapshot
                snap, wall_dt = tick_and_check(world, args.warn_wall)
                sim_t = snap.timestamp.elapsed_seconds
                loc = actor.get_location()
                print(f"[Spawned {len(vehicles)}/{args.count}] id={actor.id} sim_t={sim_t:.2f} "
                      f"spawn=({transform.location.x:.2f},{transform.location.y:.2f},{transform.location.z:.2f}) "
                      f"loc=({loc.x:.2f},{loc.y:.2f},{loc.z:.2f}) tick_wall={wall_dt*1000:.1f}ms")

                # Wait remaining ticks to reach the target interval between spawns
                for _ in range(ticks_per_spawn - 1):
                    tick_and_check(world, args.warn_wall)

            # Keep running for extra sim seconds
            extra_ticks = int(round(args.run_for / args.delta))
            print(f"[Info] All spawns done. Running for {args.run_for:.1f}s of sim-time (~{extra_ticks} ticks).")
            for _ in range(extra_ticks):
                tick_and_check(world, args.warn_wall)

    except KeyboardInterrupt:
        print("\n[Info] Interrupted by user.")
    except Exception as e:
        print(f"[Error] {e}")
    finally:
        if vehicles:
            print(f"[Cleanup] Destroying {len(vehicles)} vehicles...")
            try:
                client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
            except Exception:
                for v in vehicles:
                    try:
                        v.destroy()
                    except Exception:
                        pass
        print("[Done] World settings restored. Bye.")


if __name__ == "__main__":
    main()