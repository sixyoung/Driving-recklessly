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

# def draw_path(world, path, color):
    # for wp in path:
    #     start = wp.transform.location
    #     forward = wp.transform.get_forward_vector()
    #     end = start + forward * 1.5
    #     world.debug.draw_arrow(start, end, thickness=0.1, arrow_size=0.3, color=color, life_time=30.0)
    #     def draw_path(world, path, color):

def draw_path(world, path, color):
    for wp in path:
        loc = wp.transform.location
        world.debug.draw_point(loc, size=0.1, color=color, life_time=30.0)


def generate_path_with_intent(map, start_wp, steps, step_dist, desired_direction="any"):
    path = [start_wp]
    current_wp = start_wp
    COS_THRESHOLD = math.cos(math.radians(60))  # ≈ 0.5

    for _ in range(steps):
        next_wps = current_wp.next(step_dist)
        if not next_wps:
            break

        # ✅ 只有一个候选点，直接使用
        if len(next_wps) == 1:
            chosen_wp = next_wps[0]
            path.append(chosen_wp)
            current_wp = chosen_wp
            continue

        # ✅ 多个候选点，先过滤方向差异过大的
        curr_dir = current_wp.transform.get_forward_vector()
        filtered_wps = []
        for wp in next_wps:
            next_dir = wp.transform.get_forward_vector()
            dot = curr_dir.x * next_dir.x + curr_dir.y * next_dir.y
            if dot >= COS_THRESHOLD:
                filtered_wps.append((wp, dot))  # 保存 dot 用于后续比较

        if not filtered_wps:
            print(f"🧭 全部分支方向都不合理")
            break  # 全部分支方向都不合理

        # ✅ 根据意图，从候选中选择最符合者
        chosen_wp = None
        if desired_direction == "left":
            min_cross = float('inf')
            for wp, _ in filtered_wps:
                next_wps = wp.next(6)[0]
                next_dir = next_wps.transform.get_forward_vector()
                cross_z = curr_dir.x * next_dir.y - curr_dir.y * next_dir.x
                if cross_z < min_cross:
                    min_cross = cross_z
                    chosen_wp = wp

        elif desired_direction == "right":
            max_cross = -float('inf')
            for wp, _ in filtered_wps:
                next_wps = wp.next(6)[0]
                next_dir = next_wps.transform.get_forward_vector()
                cross_z = curr_dir.x * next_dir.y - curr_dir.y * next_dir.x
                if cross_z > max_cross:
                    max_cross = cross_z
                    chosen_wp = wp

        elif desired_direction == "straight":
            max_dot = -float('inf')
            for wp, _ in filtered_wps:
                next_wps = wp.next(6)[0]
                next_dir = next_wps.transform.get_forward_vector()
                dot = curr_dir.x * next_dir.x + curr_dir.y * next_dir.y
                if dot > max_dot:
                    max_dot = dot
                    chosen_wp = wp

        else:
            # "any" 模式：直接选第一个
            chosen_wp = filtered_wps[0][0]

        if not chosen_wp:
            break

        path.append(chosen_wp)
        current_wp = chosen_wp

    return path

def main():
    # 获取转向意图参数
    direction_arg = "any"
    if len(sys.argv) > 1:
        direction_arg = sys.argv[1].lower()
        print(f"🧭 转向意图: {direction_arg}")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()

    # ✅ 起点坐标（你可以修改）
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

    path = generate_path_with_intent(map, start_wp, steps, step_dist, desired_direction=direction_arg)
    draw_path(world, path, get_unique_color(0))

if __name__ == "__main__":
    main()
