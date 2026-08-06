#!/usr/bin/env python3

import carla
import time
import math
import threading
import matplotlib.pyplot as plt
import argparse


def get_speed_mps(v):
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def bind_spectator(world, vehicle, stop_event):
    spectator = world.get_spectator()
    while not stop_event.is_set():
        transform = vehicle.get_transform()
        spectator_transform = carla.Transform(
            transform.transform(carla.Location(x=-6, y=0, z=3)),
            carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
        )
        spectator.set_transform(spectator_transform)
        time.sleep(0.03)


def get_user_defined_spawn_point():
    return carla.Transform(carla.Location(x=-300.0, y=-2.0, z=0.5), carla.Rotation(yaw=0.0))


def run_throttle_brake_cycle(vehicle, world, brake_value):
    records = []
    start_time = time.time()
    control = carla.VehicleControl(throttle=0.7, brake=0.0)
    phase = "accelerating"
    vehicle.apply_control(control)

    prev_time = time.time()
    prev_acc_carla = 0.0
    prev_speed = 0.0
    prev_distance = 0.0
    brake_start_distance = None
    brake_end_distance = None

    while True:
        now_time = time.time()
        now = now_time - start_time
        dt = now_time - prev_time

        velocity = vehicle.get_velocity()
        speed = get_speed_mps(velocity)

        acc_vector = vehicle.get_acceleration()
        yaw_rad = math.radians(vehicle.get_transform().rotation.yaw)
        acc_carla = acc_vector.x * math.cos(yaw_rad)
        est_acc = (speed - prev_speed) / dt if dt > 1e-4 else 0.0
        jerk_carla = (acc_carla - prev_acc_carla) / dt if dt > 1e-4 else 0.0

        control = vehicle.get_control()

        prev_distance += speed * dt
        records.append((now, control.throttle, control.brake, est_acc, jerk_carla, speed, prev_distance))

        # ✅ 检测进入刹车阶段
        if phase == "accelerating" and speed >= 8.0:
            print(f"🟡 达到 {speed:.2f} m/s，brake={brake_value}")
            control = carla.VehicleControl(throttle=0.0, brake=brake_value)
            vehicle.apply_control(control)
            phase = "braking"
            brake_start_distance = prev_distance  # ✅ 记录刹车起点

        if phase == "braking" and speed <= 0.5:
            brake_end_distance = prev_distance  # ✅ 记录停止时的距离
            print(f"🛑 停止，brake={brake_value} 测试完成")
            break

        prev_time = now_time
        prev_acc_carla = acc_carla
        prev_speed = speed
        time.sleep(0.05)

    if brake_start_distance is not None and brake_end_distance is not None:
        brake_distance = brake_end_distance - brake_start_distance
        print(f"📏 brake={brake_value} 的刹车距离为：{brake_distance:.2f} 米")
    else:
        print(f"⚠️ brake={brake_value} 未能成功记录刹车距离！")

    return records



def plot_multiple_tests(all_data, brake_values):
    colors = ['blue', 'orange', 'green', 'red', 'purple']
    plt.figure(figsize=(14, 15))

    for i, data in enumerate(all_data):
        times = [r[0] for r in data]
        throttle = [r[1] for r in data]
        brake = [r[2] for r in data]
        acc_carla = [r[3] for r in data]
        jerk_carla = [r[4] for r in data]
        speed = [r[5] for r in data]
        distance = [r[6] for r in data]

        plt.subplot(5, 1, 1)
        plt.plot(times, throttle, label=f'brake={brake_values[i]}', color=colors[i % len(colors)])
        plt.ylabel('Throttle')
        plt.grid(True)

        plt.subplot(5, 1, 2)
        plt.plot(times, brake, label=f'brake={brake_values[i]}', color=colors[i % len(colors)])
        plt.ylabel('Brake')
        plt.grid(True)

        plt.subplot(5, 1, 3)
        plt.plot(times, acc_carla, '--', label=f'CARLA Acc brake={brake_values[i]}', color=colors[i % len(colors)])
        plt.ylabel('Acceleration (m/s²)')
        plt.grid(True)

        plt.subplot(5, 1, 4)
        plt.plot(times, speed, label=f'Speed brake={brake_values[i]}', color=colors[i % len(colors)])
        plt.ylabel('Speed (m/s)')
        plt.grid(True)

        plt.subplot(5, 1, 5)
        plt.plot(distance, speed, label=f'Speed vs Distance brake={brake_values[i]}', color=colors[i % len(colors)])
        plt.xlabel('Distance (m)')
        plt.ylabel('Speed (m/s)')
        plt.grid(True)

    for i in range(5):
        plt.subplot(5, 1, i + 1)
        plt.legend()

    plt.suptitle("Throttle / Brake / Acceleration / Speed / Speed-Distance vs Time")
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--brakes", nargs="+", type=float, default=[0.0, 0.5, 1.0, 2.0],
                        help="List of brake values to test (e.g., --brakes 0.2 0.6 1.0)")
    args = parser.parse_args()

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]

    spawn_point = get_user_defined_spawn_point()
    vehicle = world.spawn_actor(blueprint, spawn_point)

    stop_event = threading.Event()
    t = threading.Thread(target=bind_spectator, args=(world, vehicle, stop_event))
    t.start()

    try:
        all_records = []
        for i, brake in enumerate(args.brakes):
            print(f"\n🚀 Round {i + 1}: brake={brake}")
            vehicle.set_transform(spawn_point)
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
            time.sleep(3.0)

            records = run_throttle_brake_cycle(vehicle, world, brake)
            all_records.append(records)
        plot_multiple_tests(all_records, args.brakes)

    finally:
        stop_event.set()
        t.join()
        vehicle.destroy()
        print("✅ 测试完成并清理资源")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("❌ 手动中断，退出")
