#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import carla
import pygame
import numpy as np
import time

# =============== 配置项 ===============
ROLE_NAME = "hero0"
CAMERA_HEIGHT = 20
WINDOW_SIZE = (702, 574)
CAMERA_FOV = 90
FPS = 60

FOLLOW_YAW = True   # True = 跟随车辆转向；False = 固定方向
# =====================================


def is_vehicle_valid(world, vehicle):
    if vehicle is None:
        return False
    if world.get_actor(vehicle.id) is None:
        return False
    try:
        vehicle.get_transform()
    except RuntimeError:
        return False
    return True


def find_vehicle_by_role_name(world, role_name):
    actors = world.get_actors().filter("vehicle.*")
    for v in actors:
        if v.attributes.get("role_name", "") == role_name:
            return v
    return None


def wait_for_vehicle(world, role_name):
    print(f"[INFO] Waiting for vehicle '{role_name}' ...")
    while True:
        try:
            vehicle = find_vehicle_by_role_name(world, role_name)
            if is_vehicle_valid(world, vehicle):
                print(f"[INFO] Vehicle '{role_name}' found (id={vehicle.id}).")
                return vehicle
        except:
            pass
        time.sleep(0.1)


def create_camera(world):
    bp_lib = world.get_blueprint_library()
    cam_bp = bp_lib.find("sensor.camera.rgb")

    cam_bp.set_attribute("image_size_x", str(WINDOW_SIZE[0]))
    cam_bp.set_attribute("image_size_y", str(WINDOW_SIZE[1]))
    cam_bp.set_attribute("fov", str(CAMERA_FOV))

    init_tf = carla.Transform(carla.Location(0, 0, CAMERA_HEIGHT),
                              carla.Rotation(-90, 0, 0))
    cam = world.spawn_actor(cam_bp, init_tf)
    print(f"[INFO] Camera created (id={cam.id}).")
    return cam


def camera_callback(image, surface):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1]
    pygame.surfarray.blit_array(surface, array.swapaxes(0, 1))


def main():
    pygame.init()
    display = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("CARLA Camera Follow")

    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    vehicle = wait_for_vehicle(world, ROLE_NAME)
    camera = create_camera(world)

    camera_surface = pygame.Surface(WINDOW_SIZE)
    camera.listen(lambda img: camera_callback(img, camera_surface))

    clock = pygame.time.Clock()

    print("[INFO] Camera started. Press ESC to quit.")

    try:
        while True:
            clock.tick(FPS)

            if not is_vehicle_valid(world, vehicle):
                print("[WARN] Vehicle lost. Waiting new hero0...")
                vehicle = wait_for_vehicle(world, ROLE_NAME)
                continue

            v_tf = vehicle.get_transform()
            v_loc = v_tf.location
            v_yaw = v_tf.rotation.yaw

            v_tf = vehicle.get_transform()
            v_loc = v_tf.location
            v_yaw = v_tf.rotation.yaw

            # 沿车辆朝向反向 6 米
            offset = 0
            yaw_rad = np.radians(v_yaw)

            dx = -offset * np.cos(yaw_rad)
            dy = -offset * np.sin(yaw_rad)

            cam_loc = carla.Location(
                v_loc.x + dx,
                v_loc.y + dy,
                v_loc.z + CAMERA_HEIGHT
            )

            if FOLLOW_YAW:
                cam_rot = carla.Rotation(pitch=-90, yaw=v_yaw, roll=0)
            else:
                cam_rot = carla.Rotation(pitch=0, yaw=0, roll=0)

            camera.set_transform(carla.Transform(cam_loc, cam_rot))

            for evt in pygame.event.get():
                if evt.type == pygame.QUIT or (
                    evt.type == pygame.KEYDOWN and evt.key == pygame.K_ESCAPE):
                    raise KeyboardInterrupt

            display.blit(camera_surface, (0, 0))
            pygame.display.flip()

    except KeyboardInterrupt:
        print("[INFO] Exiting...")

    finally:
        try:
            camera.stop()
            camera.destroy()
        except:
            pass
        pygame.quit()


if __name__ == "__main__":
    main()
