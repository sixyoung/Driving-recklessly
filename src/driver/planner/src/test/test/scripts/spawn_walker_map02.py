import carla
import random
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # 加载地图
    world = client.get_world()
    print(f"Using current map: {world.get_map().name}")
    print("Loaded map: Map02")

    blueprint_library = world.get_blueprint_library()

    # 1. 选择 walker 蓝图
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    if walker_bp.has_attribute('is_invincible'):
        walker_bp.set_attribute('is_invincible', 'false')

    spawn_point = carla.Transform(carla.Location(x=-12.0, y=2.0, z=1.0))

    # 2. 选择 controller 蓝图
    walker_controller_bp = blueprint_library.find('controller.ai.walker')

    # 3. 用 batch 同时生成 walker 和 controller
    batch = [
        carla.command.SpawnActor(walker_bp, spawn_point),
        # 用 FutureActor 让 controller 依附在上一步生成的 walker
        carla.command.SpawnActor(walker_controller_bp, carla.Transform(), carla.command.FutureActor)
    ]

    results = client.apply_batch_sync(batch, True)

    walker_id = results[0].actor_id
    controller_id = results[1].actor_id

    if walker_id == 0 or controller_id == 0:
        print("Failed to spawn walker or controller.")
        return

    walker = world.get_actor(walker_id)
    walker_controller = world.get_actor(controller_id)

    print(f"Spawned walker {walker.id} and controller {walker_controller.id}")

    # 4. 设置目标点
    target_location = carla.Location(x=20.0, y=10.0, z=0.0)

    walker_controller.start()
    walker_controller.go_to_location(target_location)
    walker_controller.set_max_speed(1.5)

    print(f"Walker {walker.id} walking towards {target_location}")

    time.sleep(20)

    # 5. 清理
    walker_controller.stop()
    walker_controller.destroy()
    walker.destroy()
    print("Cleaned up walker and controller.")

if __name__ == "__main__":
    main()
