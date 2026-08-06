import carla
import math
import sys

def get_unique_color(index):
    color_list = [
        carla.Color(255, 0, 0), carla.Color(0, 255, 0), carla.Color(0, 0, 255),
        carla.Color(255, 255, 0), carla.Color(255, 0, 255), carla.Color(0, 255, 255),
        carla.Color(255, 165, 0), carla.Color(128, 0, 128), carla.Color(0, 128, 128),
        carla.Color(128, 128, 0)
    ]
    return color_list[index % len(color_list)]

def draw_path(world, path, color):
    for wp in path:
        loc = wp.transform.location
        world.debug.draw_point(loc, size=0.1, color=color, life_time=60.0)

def generate_all_paths(map, start_wp, steps, step_dist, max_branches=100):
    all_paths = []

    def dfs(current_wp, path, remaining_steps):
        if len(all_paths) >= max_branches:
            return
        if remaining_steps == 0:
            all_paths.append(path)
            return

        next_wps = current_wp.next(step_dist)
        if not next_wps:
            all_paths.append(path)
            return

        for wp in next_wps:
            dfs(wp, path + [wp], remaining_steps - 1)

    dfs(start_wp, [start_wp], steps)
    return all_paths

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()

    # ✅ 起点坐标（可根据地图调整）
    x, y, z = -40, 2.0, 0.0
    start_location = carla.Location(x=x, y=y, z=z)
    start_wp = map.get_waypoint(start_location)

    if not start_wp:
        print("❌ 无法获取该位置的有效Waypoint，请确认坐标是否在道路上")
        return

    print(f"✅ 起点: {start_wp.transform.location}  road_id={start_wp.road_id}, lane_id={start_wp.lane_id}")

    # 参数
    step_dist = 0.5
    steps = 120
    max_branches = 50  # 防止爆炸性递归

    # ✅ 生成所有路径分支
    all_paths = generate_all_paths(map, start_wp, steps, step_dist, max_branches=max_branches)

    # ✅ 绘制所有路径
    for idx, path in enumerate(all_paths):
        color = get_unique_color(idx)
        draw_path(world, path, color)

    print(f"🎯 共生成路径分支：{len(all_paths)}")

if __name__ == "__main__":
    main()
