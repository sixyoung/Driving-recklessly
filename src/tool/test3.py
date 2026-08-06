# -*- coding: utf-8 -*-
import carla
import random

def get_unique_color(index):
    color_list = [
        carla.Color(255, 0, 0), carla.Color(0, 255, 0), carla.Color(0, 0, 255),
        carla.Color(255, 255, 0), carla.Color(255, 0, 255), carla.Color(0, 255, 255),
        carla.Color(255, 165, 0), carla.Color(128, 0, 128), carla.Color(0, 128, 128),
        carla.Color(128, 128, 0)
    ]
    return color_list[index % len(color_list)]

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # world = client.load_world("Town10HD")
    debug = world.debug

    traffic_lights = world.get_actors().filter("traffic.traffic_light")
    print(f"共找到 {len(traffic_lights)} 个交通灯")

    visited_groups = set()
    color_index = 0

    for light in traffic_lights:
        group = light.get_group_traffic_lights()
        group_ids = tuple(sorted([g.id for g in group]))

        # 去重：同一组只画一次
        if group_ids in visited_groups:
            continue
        visited_groups.add(group_ids)

        color = get_unique_color(color_index)
        color_index += 1

        for l in group:
            wp_list = l.get_stop_waypoints()
            id = l.get_opendrive_id()
            print(f"🚦 Traffic Light ID={l.id} 影响 {len(wp_list)} 个航点 opid={id}")

            for wp in wp_list:
                loc = wp.transform.location + carla.Location(z=0.3)
                debug.draw_point(loc, size=0.1, color=color, life_time=30.0)

                road_id = wp.road_id
                lane_id = wp.lane_id
                print(f"    📍 当前点: (x={loc.x:.2f}, y={loc.y:.2f}) | road_id={road_id}, lane_id={lane_id}")

                # ✅ 沿相反方向往回查找 3 米处的路点
                backward_wp = wp.previous(3.0)
                if backward_wp:
                    bw_wp = backward_wp[0]
                    bw_loc = bw_wp.transform.location
                    print(f"    🔙 往回3米点: (x={bw_loc.x:.2f}, y={bw_loc.y:.2f}) | road_id={bw_wp.road_id}, lane_id={bw_wp.lane_id}")
                    debug.draw_point(bw_loc + carla.Location(z=0.3), size=0.1, color=carla.Color(255, 255, 0), life_time=30.0)
                else:
                    print("    ⚠️ 无法获取回溯3m处的Waypoint")



if __name__ == '__main__':
    main()
