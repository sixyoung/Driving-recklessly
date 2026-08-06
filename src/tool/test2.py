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

# def draw_path(world, path, default_color):
#     entered_junction = False  # 标记是否已经进入过路口

#     for i in range(len(path)):
#         wp = path[i]
#         loc = wp.transform.location

#         # 默认颜色（灰或红）
#         if wp.is_junction:
#             color = carla.Color(255, 0, 0)  # 红色：处于路口
#         else:
#             color = carla.Color(100, 100, 100)  # 灰色：非路口

#         # 检查“从非路口进入路口”的转折点
#         if (not entered_junction and 
#             i + 1 < len(path) and 
#             not path[i].is_junction and 
#             path[i + 1].is_junction):
#             entered_junction = True  # 防止多次打印
#             entry_loc = path[i].transform.location
#             print(f"🔷 Detected junction entry point at: x={entry_loc.x:.2f}, y={entry_loc.y:.2f}, z={entry_loc.z:.2f}, road_id={path[i].road_id}, lane_id={path[i].lane_id} ")
#             color = carla.Color(0, 0, 255)  # 蓝色：进入路口的点

#         world.debug.draw_point(loc, size=0.1, color=color, life_time=30.0)

def draw_path(world, path, default_color):
    entered_junction = False
    id_to_color = {}  # {(road_id, lane_id): carla.Color}
    color_pool = [
        carla.Color(255, 0, 0), carla.Color(0, 255, 0), carla.Color(0, 0, 255),
        carla.Color(255, 255, 0), carla.Color(255, 0, 255), carla.Color(0, 255, 255),
        carla.Color(255, 165, 0), carla.Color(128, 0, 128), carla.Color(0, 128, 128),
        carla.Color(128, 128, 0)
    ]
    color_index = 0

    for i in range(len(path)):
        wp = path[i]
        loc = wp.transform.location
        key = (wp.road_id, wp.lane_id)

        # 分配颜色
        if key not in id_to_color:
            id_to_color[key] = color_pool[color_index % len(color_pool)]
            color_index += 1
        color = id_to_color[key]

        # 检查是否进入路口
        if (not entered_junction and 
            i + 1 < len(path) and 
            not path[i].is_junction and 
            path[i + 1].is_junction):
            
            entered_junction = True
            entry_loc = wp.transform.location
            print(f"🔷 Detected junction entry point at: "
                  f"{entry_loc.x:.2f}f, {entry_loc.y:.2f}f, {entry_loc.z:.2f}f, "
                  f"{wp.road_id}, {wp.lane_id}")
            print(f"🔷🔷 Detected junction entry point at: "
                  f"{path[i + 1].road_id}, {path[i + 1].lane_id}")
            
            print(f"{{{{{wp.road_id}, {wp.lane_id}}}, carla::geom::Location({entry_loc.x:.2f}f, {entry_loc.y:.2f}f, {entry_loc.z:.2f}f)}}")
            print(f"{{{{{path[i + 1].road_id}, {path[i + 1].lane_id}}}, carla::geom::Location({entry_loc.x:.2f}f, {entry_loc.y:.2f}f, {entry_loc.z:.2f}f)}}")

            color = carla.Color(0, 0, 255)  # 蓝色标记该点
            world.debug.draw_point(loc, size=0.15, color=color, life_time=30.0)

            # 打印并绘制 path[i + 5]
            # if i + 5 < len(path):
            #     wp_ahead = path[i + 5]
            #     loc_ahead = wp_ahead.transform.location
            #     print(f"🟡 Future point at i+5: "
            #           f"x={loc_ahead.x:.2f}, y={loc_ahead.y:.2f}, z={loc_ahead.z:.2f}, "
            #           f"road_id={wp_ahead.road_id}, lane_id={wp_ahead.lane_id}")
                
            #     world.debug.draw_point(
            #         carla.Location(loc_ahead.x, loc_ahead.y, loc_ahead.z + 0.5),
            #         size=0.15,
            #         color=carla.Color(255, 0, 0),
            #         life_time=30.0
            #     )
        # ✅ 检查出口点
        elif (entered_junction and 
              i + 1 < len(path) and 
              path[i].is_junction and 
              not path[i + 1].is_junction):
            
            exit_loc = path[i].transform.location
            print(f"🔶 Detected junction exit point at: "
                  f"{exit_loc.x:.2f}f, {exit_loc.y:.2f}f, {exit_loc.z:.2f}f, "
                  f"{path[i].road_id}, {path[i].lane_id}")
            print(f"🔶🔶 Detected junction exit point at: "
                  f"{path[i + 1].road_id}, {path[i + 1].lane_id}")
              
            print(f"{{{{{path[i].road_id}, {path[i].lane_id}}}, carla::geom::Location({exit_loc.x:.2f}f, {exit_loc.y:.2f}f, {exit_loc.z:.2f}f)}}")
            print(f"{{{{{path[i + 1].road_id}, {path[i + 1].lane_id}}}, carla::geom::Location({exit_loc.x:.2f}f, {exit_loc.y:.2f}f, {exit_loc.z:.2f}f)}}") 

            world.debug.draw_point(
                carla.Location(exit_loc.x, exit_loc.y, exit_loc.z + 0.2),
                size=0.15,
                color=carla.Color(255, 255, 0),  # 黄色
                life_time=30.0
            )
        else:
            # 普通点颜色绘制
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
    x, y, z = -2, -40, 0.0
    start_location = carla.Location(x=x, y=y, z=z)
    start_wp = map.get_waypoint(start_location)

    if not start_wp:
        print("❌ 无法获取该位置的有效Waypoint，请确认坐标是否在道路上")
        return

    print(f"✅ 起点: {start_wp.transform.location}  road_id={start_wp.road_id}, lane_id={start_wp.lane_id}")

    # 参数
    step_dist = 1.0
    steps = 120

    path = generate_path_with_intent(map, start_wp, steps, step_dist, desired_direction=direction_arg)
    draw_path(world, path, get_unique_color(0))

if __name__ == "__main__":
    main()
