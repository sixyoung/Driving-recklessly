import carla
import time

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 清除已有绘制
    # world.debug.clear()

    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    print(f"Found {len(traffic_lights)} traffic lights.")

    for light in traffic_lights:
        stop_wps = light.get_stop_waypoints()
        if not stop_wps:
            print(f"TrafficLight id={light.id} has no stop waypoints.")
            continue

        for wp in stop_wps:
            # 起点
            start = wp.transform.location + carla.Location(z=0.5)
            # 朝向方向
            forward = wp.transform.get_forward_vector()
            # 终点：朝车辆行驶方向前方 2 米
            end = start + forward * 2.0

            # 绘制红色箭头（2米长，持续10秒）
            world.debug.draw_arrow(
                start,
                end,
                thickness=0.1,
                arrow_size=0.3,
                color=carla.Color(255, 0, 0),
                life_time=100.0
            )
            print(f"Drawn arrow at stop wp: RoadId={wp.road_id}, LaneId={wp.lane_id}")

    print("✅ Done drawing all stop waypoints.")
    time.sleep(10)  # 保证绘制可见

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
