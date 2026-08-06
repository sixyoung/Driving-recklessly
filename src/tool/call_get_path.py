#!/usr/bin/env python3
import rospy
import json
from geometry_msgs.msg import Pose
from driver_models_types.srv import PathWithOptionsService, PathWithOptionsServiceRequest
from carla_common.transforms import carla_transform_to_ros_pose, ros_pose_to_carla_transform
import carla
import random


def call_get_path(role_name, start_pose, goal_pose):
    """调用 /carla_waypoint_publisher/get_path 服务，获取从 start 到 goal 的路径"""
    rospy.wait_for_service('/carla_waypoint_publisher/get_path')
    try:
        get_path_client = rospy.ServiceProxy('/carla_waypoint_publisher/get_path', PathWithOptionsService)
        req = PathWithOptionsServiceRequest()
        req.role_name = role_name
        req.start = start_pose
        req.goal = goal_pose

        rospy.loginfo(f"[{role_name}] Calling service '/carla_waypoint_publisher/get_path' ...")
        resp = get_path_client(req)

        if resp.success:
            rospy.loginfo(f"[{role_name}] ✅ Success: {resp.message}")
            rospy.loginfo(f"Returned path has {len(resp.path.waypoints)} points.")
            return resp.path.waypoints
        else:
            rospy.logerr(f"[{role_name}] ❌ Failed: {resp.message}")
            return None

    except rospy.ServiceException as e:
        rospy.logerr(f"[{role_name}] 🚨 Service call failed: {e}")
        return None


def draw_path_in_carla(world, waypoints, color=(0.0, 0.0, 1.0), lifetime=30.0):
    """在 CARLA 世界中绘制路径"""
    if waypoints is None or len(waypoints) == 0:
        rospy.logwarn("⚠️ No path points to draw.")
        return

    rospy.loginfo(f"🎨 Drawing {len(waypoints)} path points in CARLA world ...")

    color_r = max(color[0], 0.3)  # 保证最低亮度
    color_g = max(color[1], 0.3)
    color_b = max(color[2], 0.3)

    draw_color = carla.Color(
        r=int(min(255, color_r * 255)),
        g=int(min(255, color_g * 255)),
        b=int(min(255, color_b * 255))
    )

    debug = world.debug

    # 连续绘制线段
    for i in range(len(waypoints) - 1):
        p1 = ros_pose_to_carla_transform(waypoints[i].pose).location
        loc1 = carla.Location(x=p1.x, y=p1.y, z=p1.z + 0.3)
        debug.draw_point(loc1, size=0.2, color=draw_color, life_time=lifetime)

    # 每隔 10 个点显示编号
    for i, wp in enumerate(waypoints[::10]):
        p2 = ros_pose_to_carla_transform(wp.pose).location
        loc2 = carla.Location(x=p2.x, y=p2.y, z=p2.z + 0.3)
        debug.draw_string(loc2, f"{i*10}", False, draw_color, lifetime, False)

    # 起点绿色 / 终点红色球体
    start_location = ros_pose_to_carla_transform(waypoints[0].pose).location
    end_location = ros_pose_to_carla_transform(waypoints[-1].pose).location
    debug.draw_point(carla.Location(x=start_location.x, y=start_location.y, z=start_location.z + 0.3),
                     size=0.2, color=carla.Color(0, 255, 0), life_time=lifetime)
    debug.draw_point(carla.Location(x=end_location.x, y=end_location.y, z=end_location.z + 0.3),
                     size=0.2, color=carla.Color(255, 0, 0), life_time=lifetime)

    rospy.loginfo("✅ Path drawn in CARLA (visible for %.1f seconds)." % lifetime)


def save_carla_transforms_to_file(transforms, save_path_txt="main_path_transforms.txt", save_path_json="main_path_transforms.json"):
    """
    按“路点0001:{"x": xx, ...}”格式保存 carla.Transform 列表
    :param transforms: carla.Transform 列表（主路径路点）
    :param save_path_txt: TXT 文件保存路径（默认当前目录）
    :param save_path_json: JSON 文件保存路径（默认当前目录）
    """
    if not transforms or len(transforms) == 0:
        rospy.logwarn("⚠️ No carla.Transform data to save.")
        return

    # 1. 保存为 TXT 格式（严格按“路点0256:{"x": -16, "y": 10, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": xx}”格式）
    try:
        with open(save_path_txt, "w") as f:
            f.write("主路径路点数据（格式：路点编号:{'x': 位置x, 'y': 位置y, 'z': 位置z, 'roll': 滚转, 'pitch': 俯仰, 'yaw': 偏航}\n")
            f.write("="*120 + "\n")
            for idx, trans in enumerate(transforms):
                # 格式化路点编号（4位补零，如0001、0256）
                wp_num = f"{idx+1:04d}"  # 路点从1开始编号，补零至4位
                loc = trans.location
                rot = trans.rotation
                # 按要求格式拼接字符串（注意yaw值保留原精度，与其他参数一致）
                wp_str = f'路点{wp_num}:"x": {loc.x:.1f}, "y": {loc.y:.1f}, "z": {loc.z:.1f}, "roll": {rot.roll:.1f}, "pitch": {rot.pitch:.1f}, "yaw": {rot.yaw:.1f}'
                f.write(wp_str + "\n")
        rospy.loginfo(f"✅ 主路径 TXT 文件已保存至: {save_path_txt}")
    except Exception as e:
        rospy.logerr(f"❌ TXT 文件保存失败: {e}")

    # 2. 保存为 JSON 格式（同步按“路点0001”为键的格式）
    try:
        transform_dict = {}
        for idx, trans in enumerate(transforms):
            wp_num = f"路点{idx+1:04d}"  # 键名格式：路点0001
            loc = trans.location
            rot = trans.rotation
            # 值为包含所有参数的字典
            transform_dict[wp_num] = {
                "x": round(loc.x, 1),
                "y": round(loc.y, 1),
                "z": round(loc.z, 1),
                "roll": round(rot.roll, 1),
                "pitch": round(rot.pitch, 1),
                "yaw": round(rot.yaw, 1)
            }
        
        with open(save_path_json, "w") as f:
            json.dump(transform_dict, f, indent=2, ensure_ascii=False)
        rospy.loginfo(f"✅ 主路径 JSON 文件已保存至: {save_path_json}")
    except Exception as e:
        rospy.logerr(f"❌ JSON 文件保存失败: {e}")


if __name__ == "__main__":
    rospy.init_node("get_path_client_draw")

    role_name = "hero0"

    # === 1. 连接 CARLA ===
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    # === 2. 定义主路径的起点/终点 ===
    start_location = carla.Location(x=-50.4, y=142.5, z=2)
    start_rotation = carla.Rotation(pitch=0, yaw=0.0, roll=0.0)
    goal_location = carla.Location(x=-10, y=37.6, z=11)
    goal_rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)

    start_pose = carla_transform_to_ros_pose(carla.Transform(start_location, start_rotation))
    goal_pose = carla_transform_to_ros_pose(carla.Transform(goal_location, goal_rotation))

    # === 3. 主路径（新增：获取并保存 carla.Transform） ===
    main_waypoints = call_get_path(role_name, start_pose, goal_pose)
    if main_waypoints:
        draw_path_in_carla(world, main_waypoints, color=(0.0, 0.4, 1.0))
        # 将 ros waypoints 转换为 carla.Transform 列表
        main_carla_transforms = [ros_pose_to_carla_transform(wp.pose) for wp in main_waypoints]
        # 保存到文件（可修改保存路径，默认保存在当前运行目录）
        save_carla_transforms_to_file(main_carla_transforms, "/home/bob/文档/备份/demo05/src/tool/main_path_transforms.json")

    # === 4. 获取起点右侧车道的路径 ===
    end_wp = carla_map.get_waypoint(goal_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    right_wp = end_wp.get_right_lane()

    # if right_wp is None:
    #     rospy.logwarn("❌ 当前车道右侧没有可行驶车道，无法生成右侧路径。")
    # else:
    #     rospy.loginfo(f"右侧车道: road_id={right_wp.road_id}, lane_id={right_wp.lane_id}")
    #     right_end_pose = carla_transform_to_ros_pose(right_wp.transform)

    #     # 调用服务生成右侧路径
    #     right_waypoints = call_get_path("hero0", start_pose, right_end_pose)

    #     if right_waypoints:
    #         draw_path_in_carla(world, right_waypoints, color=(1.0, 0.2, 0.2))  # 红色右侧路径
    #     else:
    #         rospy.logwarn("⚠️ 无法生成右侧路径。")

    rospy.loginfo("✅ 所有路径绘制与保存完成。")