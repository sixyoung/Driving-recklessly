import carla
import time

client = carla.Client('localhost', 2000)
client.set_timeout(20.0)

# 获取 CARLA 中所有可用的地图
available_maps = client.get_available_maps()
print("🚗 可用地图列表：")
for map_name in available_maps:
    print(map_name)


def show_point(world, point_location, point_type):
    color = carla.Color(r=255, g=0, b=0) if point_type == 'Star' else carla.Color(r=0, g=0, b=255)
    world.debug.draw_string(point_location, point_type, draw_shadow=False,
                            color=color, life_time=100, persistent_lines=True)


def show_axis_point(world, label, point_location):
    world.debug.draw_string(point_location, label, draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)


def show_orientation(world, location, yaw_angle_deg):
    """ 在指定位置显示朝向箭头和文字标签 """
    rotation = carla.Rotation(yaw=yaw_angle_deg)
    arrow_end = location + rotation.get_forward_vector() * 5  # 箭头长度
    world.debug.draw_arrow(location, arrow_end, thickness=0.2, arrow_size=0.5,
                           color=carla.Color(r=255, g=255, b=0), life_time=100, persistent_lines=True)
    world.debug.draw_string(location + carla.Location(z=1.5),
                            f"{yaw_angle_deg}°", draw_shadow=False,
                            color=carla.Color(r=255, g=255, b=255), life_time=100, persistent_lines=True)


def main():
    """ 🚗 加载地图并可视化标记点与方向 """
    map_name = "Town04"  # 替换为你需要的地图
    world = client.get_world()

    if world.get_map().name.split("/")[-1] != map_name:
        print(f"🔄 正在加载地图: {map_name} ...")
        world = client.load_world(map_name)
        world.tick()
        time.sleep(1.0)

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=40), carla.Rotation(yaw=0, pitch=-90)))

    start_point = carla.Location(-120, -20, 0)
    end_point = carla.Location(10, 37, 12)
    origin_point = carla.Location(0, 0, 0)
    x_axis = carla.Location(10,0, 0)
    y_axis = carla.Location(0,10, 0)

    start_wp = world.get_map().get_waypoint(start_point, project_to_road=True, lane_type=carla.LaneType.Driving)
    print("\n📍 起点 Waypoint:")
    print(f"  位置: {start_wp.transform.location}")
    print(f"  车道ID: {start_wp.lane_id}, 车道类型: {start_wp.lane_type}")
    print(f"  朝向: {start_wp.transform.rotation}")

    # 标记起点终点及坐标轴原点
    show_point(world, start_point, 'Star')
    show_point(world, end_point, 'End')
    show_axis_point(world, 'O', origin_point)
    show_axis_point(world, 'X', x_axis)
    show_axis_point(world, 'Y', y_axis)

    # 可视化方向箭头：0°、90°、180°
    show_orientation(world, carla.Location(-83, 22, 20), 0)
    show_orientation(world, carla.Location(-83, 22, 20), 90)

    world.wait_for_tick()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')


# import carla
# import time

# client = carla.Client('localhost', 2000)
# client.set_timeout(20.0)

# def main():
#     """ 🚗 **加载地图并依次卸载所有地图图层** """
#     map_name = "Town10HD_Opt"  # 选择地图
#     print(f"🔄 正在加载地图: {map_name} ...")
    
#     # 加载地图
#     world = client.load_world(map_name)
#     print("✅ 初始地图加载完成！")

#     # 所有 CARLA 地图图层
#     map_layers = [
#         carla.MapLayer.Buildings,
#         carla.MapLayer.Decals,
#         carla.MapLayer.Foliage,
#         carla.MapLayer.Ground,
#         carla.MapLayer.ParkedVehicles,
#         carla.MapLayer.Props,
#         carla.MapLayer.StreetLights,
#         carla.MapLayer.Walls
#     ]

#     # 依次移除所有图层
#     for layer in map_layers:
#         world.unload_map_layer(layer)
#         print(f"🚧 已移除图层: {layer}")
#         time.sleep(2)  # 等待 1 秒，确保图层卸载生效

#     print("❌ 所有地图图层已移除！")

#     world.wait_for_tick()

# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         print(' - Exited by user.')
