
# import carla
# import time

# def get_active_light(group):
#     """返回当前 group 中处于激活状态的灯（即 elapsed > 0)"""
#     for light in group:
#         if light.get_elapsed_time() > 0:
#             return light
#     return None

# def get_red_light_remaining(group_lights, current_light):
#     """计算当前灯的红灯预计剩余时间"""
#     if not group_lights or current_light is None:
#         return None

#     active_light = get_active_light(group_lights)
#     if not active_light:
#         return None

#     # 通过 ID 找索引，避免对象地址不一致的问题
#     active_index = next((i for i, l in enumerate(group_lights) if l.id == active_light.id), None)
#     current_index = next((i for i, l in enumerate(group_lights) if l.id == current_light.id), None)

#     if active_index is None or current_index is None:
#         return None

#     elapsed = active_light.get_elapsed_time()

#     # 计算从 active 到 current 之间的时间总和（green + yellow）
#     total_time = 0.0
#     index = (active_index + 1) % len(group_lights)
#     while index != current_index:
#         light = group_lights[index]
#         total_time += light.get_green_time() + light.get_yellow_time() +light.get_red_time()
#         index = (index + 1) % len(group_lights)

#     # 加上当前 active 的剩余时间
#     state = active_light.get_state()
#     if state == carla.TrafficLightState.Green:
#         remaining = active_light.get_yellow_time() + active_light.get_red_time() + active_light.get_green_time() - elapsed
#     elif state == carla.TrafficLightState.Yellow:
#         remaining = active_light.get_red_time() + active_light.get_yellow_time() - elapsed
#     elif state == carla.TrafficLightState.Red:
#         remaining = active_light.get_red_time() - elapsed
#     else:
#         remaining = 0.0  # 正常情况下不会出现 active 是红灯

#     total_time += max(0.0, remaining)
#     return total_time

# def main():
#     client = carla.Client('localhost', 2000)
#     client.set_timeout(5.0)
#     world = client.get_world()
#     debug = world.debug

#     # 获取所有交通灯
#     traffic_lights = world.get_actors().filter('traffic.traffic_light')
#     print(f"共找到 {len(traffic_lights)} 个交通灯，正在设置每个灯的相位信息...\n")

#     # ✅ 自定义红绿灯相位时长（单位：秒）
#     RED_DURATION = 0.0
#     YELLOW_DURATION = 1.0
#     GREEN_DURATION = 6.0

#     for light in traffic_lights:
#         # 设置每个车灯的相位信息
#         light.set_green_time(GREEN_DURATION)
#         light.set_yellow_time(YELLOW_DURATION)
#         light.set_red_time(RED_DURATION)
#         light.set_state(carla.TrafficLightState.Red)
#         print(f"设置灯 ID={light.id} | GREEN={GREEN_DURATION}s | YELLOW={YELLOW_DURATION}s | RED={RED_DURATION}s")

#     while True:
#         for light in traffic_lights:
#             group_lights = light.get_group_traffic_lights()
#             if not group_lights:
#                 continue
#             active_light = get_active_light(group_lights)

#             if active_light and light.id == active_light.id:
#                 # 当前是激活灯，正常计时
#                 elapsed = light.get_elapsed_time()
#                 state = light.get_state()

#                 if state == carla.TrafficLightState.Red:
#                     duration = light.get_red_time()
#                     color = carla.Color(255, 0, 0)
#                 elif state == carla.TrafficLightState.Yellow:
#                     duration = light.get_yellow_time()
#                     color = carla.Color(255, 255, 0)
#                 elif state == carla.TrafficLightState.Green:
#                     duration = light.get_green_time()
#                     color = carla.Color(0, 255, 0)
#                 else:
#                     continue
#                 if state == carla.TrafficLightState.Red:
#                     print(f"设置灯red")
#                     # 定位当前灯的顺序
#                     current_index = next((i for i, l in enumerate(group_lights) if l.id == light.id), None)
#                     # 遍历 group 中“下一个灯”到“最后一个灯”的所有状态时长（红+绿+黄）
#                     wait_time = 0.0
#                     if current_index is not None:
#                         index = (current_index + 1) % len(group_lights)
#                         while index != current_index:
#                             l = group_lights[index]
#                             wait_time += l.get_red_time() + l.get_green_time() + l.get_yellow_time()
#                             index = (index + 1) % len(group_lights)

#                     remaining = max(0.0, duration - elapsed + wait_time)
#                 else:   
#                     remaining = max(0.0, duration - elapsed)
#             else:
#                 # 非激活灯，预测红灯剩余时间
#                 remaining = get_red_light_remaining(group_lights, light)
#                 if remaining is None:
#                     continue
#                 color = carla.Color(255, 0, 0)

#             loc = light.get_transform().location + carla.Location(z=8)
#             debug.draw_string(
#                 loc,
#                 text=f"ID={light.id}  S: {remaining:.1f}s",
#                 color=color,
#                 life_time=0.12,
#                 persistent_lines=False
#             )

#         time.sleep(0.05)

# if __name__ == '__main__':
#     main()

import carla
import time

def get_active_light(group):
    """返回当前 group 中处于激活状态的灯（即 elapsed > 0)"""
    for light in group:
        if light.get_elapsed_time() > 0:
            return light
    return None

def get_red_light_remaining(group_lights, current_light):
    """计算当前灯的红灯预计剩余时间"""
    if not group_lights or current_light is None:
        return None

    active_light = get_active_light(group_lights)
    if not active_light:
        return None

    # 通过 ID 找索引，避免对象地址不一致的问题
    active_index = next((i for i, l in enumerate(group_lights) if l.id == active_light.id), None)
    current_index = next((i for i, l in enumerate(group_lights) if l.id == current_light.id), None)

    if active_index is None or current_index is None:
        return None

    elapsed = active_light.get_elapsed_time()

    # 计算从 active 到 current 之间的时间总和（green + yellow）
    total_time = 0.0
    index = (active_index + 1) % len(group_lights)
    while index != current_index:
        light = group_lights[index]
        total_time += light.get_green_time() + light.get_yellow_time() +light.get_red_time()
        index = (index + 1) % len(group_lights)

    # 加上当前 active 的剩余时间
    state = active_light.get_state()
    if state == carla.TrafficLightState.Green:
        remaining = active_light.get_yellow_time() + active_light.get_red_time() + active_light.get_green_time() - elapsed
    elif state == carla.TrafficLightState.Yellow:
        remaining = active_light.get_red_time() + active_light.get_yellow_time() - elapsed
    elif state == carla.TrafficLightState.Red:
        remaining = active_light.get_red_time() - elapsed
    else:
        remaining = 0.0  # 正常情况下不会出现 active 是红灯

    total_time += max(0.0, remaining)
    return total_time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    debug = world.debug

    # 获取所有交通灯
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    print(f"共找到 {len(traffic_lights)} 个交通灯，正在设置每个灯的相位信息...\n")

    # ✅ 自定义红绿灯相位时长（单位：秒）
    RED_DURATION = 0.0
    YELLOW_DURATION = 3.0
    GREEN_DURATION = 30.0

    for light in traffic_lights:
        # 设置每个车灯的相位信息
        light.set_green_time(GREEN_DURATION)
        light.set_yellow_time(YELLOW_DURATION)
        light.set_red_time(RED_DURATION)
        light.set_state(carla.TrafficLightState.Red)
        print(f"设置灯 ID={light.id} | GREEN={GREEN_DURATION}s | YELLOW={YELLOW_DURATION}s | RED={RED_DURATION}s")

    while True:
        for light in traffic_lights:
            group_lights = light.get_group_traffic_lights()
            if not group_lights:
                continue
            active_light = get_active_light(group_lights)

            if active_light and light.id == active_light.id:
                # 当前是激活灯，正常计时
                elapsed = light.get_elapsed_time()
                state = light.get_state()

                if state == carla.TrafficLightState.Red:
                    duration = light.get_red_time()
                    color = carla.Color(255, 0, 0)
                elif state == carla.TrafficLightState.Yellow:
                    duration = light.get_yellow_time()
                    color = carla.Color(255, 255, 0)
                elif state == carla.TrafficLightState.Green:
                    duration = light.get_green_time()
                    color = carla.Color(0, 255, 0)
                else:
                    continue
                if state == carla.TrafficLightState.Red:
                    print(f"设置灯red")
                    # 定位当前灯的顺序
                    current_index = next((i for i, l in enumerate(group_lights) if l.id == light.id), None)
                    # 遍历 group 中“下一个灯”到“最后一个灯”的所有状态时长（红+绿+黄）
                    wait_time = 0.0
                    if current_index is not None:
                        index = (current_index + 1) % len(group_lights)
                        while index != current_index:
                            l = group_lights[index]
                            wait_time += l.get_red_time() + l.get_green_time() + l.get_yellow_time()
                            index = (index + 1) % len(group_lights)

                    remaining = max(0.0, duration - elapsed + wait_time)
                else:   
                    remaining = max(0.0, duration - elapsed)
            else:
                # 非激活灯，预测红灯剩余时间
                remaining = get_red_light_remaining(group_lights, light)
                if remaining is None:
                    continue
                color = carla.Color(255, 0, 0)

            loc = light.get_transform().location + carla.Location(z=8)
            debug.draw_string(
                loc,
                text=f"ID={light.id}  S: {remaining:.1f}s",
                color=color,
                life_time=0.12,
                persistent_lines=False
            )

        time.sleep(0.05)

if __name__ == '__main__':
    main()
