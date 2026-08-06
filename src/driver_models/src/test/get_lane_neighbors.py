#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：
  获取指定坐标点对应的 Carla Waypoint，
  打印中、左、右车道的详细信息（含是否在路口），
  并在 Carla 世界中用箭头可视化三个车道方向。
"""

import carla
import math

# ================= 工具函数 =================

def draw_arrow(world, wp, color, tag, life_time=15.0):
    """在CARLA中绘制代表车道方向的箭头"""
    if wp is None:
        return
    base = wp.transform.location
    yaw = math.radians(wp.transform.rotation.yaw)
    # 箭头终点（向前2米）
    end = carla.Location(
        x=base.x + 2.0 * math.cos(yaw),
        y=base.y + 2.0 * math.sin(yaw),
        z=base.z + 0.2,
    )

    world.debug.draw_arrow(
        base,
        end,
        thickness=0.1,
        arrow_size=0.3,
        color=color,
        life_time=life_time,
    )
    world.debug.draw_string(base, f"{tag}", draw_shadow=False,
                            color=color, life_time=life_time)


def print_waypoint_info(tag, wp):
    """打印 waypoint 的详细信息"""
    if wp is None:
        print(f"[{tag}] ❌ 无对应车道")
        return

    print(f"\n[{tag}] Waypoint 信息：")
    print(f"  RoadID: {wp.road_id}")
    print(f"  SectionID: {wp.section_id}")
    print(f"  LaneID: {wp.lane_id}")
    print(f"  LaneType: {wp.lane_type}")
    print(f"  LaneChange: {wp.lane_change}")
    print(f"  Driving方向: {'正向' if wp.lane_id > 0 else '反向'}")

    loc = wp.transform.location
    rot = wp.transform.rotation
    print(f"  坐标: x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}")
    print(f"  航向角(yaw): {rot.yaw:.2f}°")

    # 新增：是否处于路口
    print(f"  是否处于路口: {'✅ 是' if wp.is_junction else '❌ 否'}")


# ================= 主程序 =================

def main():
    # 1️⃣ 连接CARLA服务器
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    map_ = world.get_map()

    # 2️⃣ 指定位置
    x, y, z = 6.0, -80.0, 0.5   # 可以自行修改
    location = carla.Location(x=x, y=y, z=z)

    # 3️⃣ 获取Waypoint
    wp = map_.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        print("❌ 未找到对应车道！")
        return

    # 4️⃣ 获取左右车道
    left_wp = wp.get_left_lane()
    right_wp = wp.get_right_lane()

    # 5️⃣ 打印详细信息
    print_waypoint_info("中心车道", wp)
    print_waypoint_info("左侧车道", left_wp)
    print_waypoint_info("右侧车道", right_wp)

    # 6️⃣ 绘制箭头
    print("\n🎯 在CARLA中绘制车道方向箭头...")
    draw_arrow(world, wp, carla.Color(0, 255, 0), "中心车道")
    draw_arrow(world, left_wp, carla.Color(0, 0, 255), "左侧车道")
    draw_arrow(world, right_wp, carla.Color(255, 0, 0), "右侧车道")

    print("✅ 可视化完成！（绿色=中心，蓝色=左侧，红色=右侧）")


if __name__ == "__main__":
    main()
