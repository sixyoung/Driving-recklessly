import ros_compatibility as roscomp
import ros_compatibility.qos
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode

from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
import geometry_msgs
from carla_common.transforms import carla_transform_to_ros_pose, carla_transform_to_ros_transform

from carla_msgs.srv import SpawnObject, DestroyObject
from carla_msgs.msg import CarlaActorList
from driver_models_types.msg import VehicleConfig, VehicleGoalStatus, RandomBehavior, LaneInfo, VehicleConfigAck

import random
import json
import os
import copy
import carla
import time
import math

class ScenarioExecutor(CompatibleNode):
    def __init__(self, ):
        super(ScenarioExecutor, self).__init__('scenario_executor')

        self.objects_definition_file = self.get_param('scenario_path', '')
        self.spawn_sensors_only = self.get_param('spawn_sensors_only', False)
        self.dynamic_spawn_flag = self.get_param('dynamic_spawn_flag', False)

        self.found_sensor_actor_list = False
        self.global_sensors_spawned = False
        self.vehicles_spawned = False

        self.vehicle_spawn_and_target_points_left = [] # 左
        self.vehicle_spawn_and_target_points_right = [] # 右
        self.vehicle_spawn_and_target_points_up = [] # 上
        self.vehicle_spawn_and_target_points_down = [] # 下


        self.vehicle_types = [] # 车辆模型库
        self.cruise_speeds = [] # 车辆巡航速度库
        self.driver_models = [] # 驾驶员模型库
        self.global_sensors = [] # 全局传感器库
        self.vehicle_sensors = [] # 车辆传感器库
        self.vehicles = [] # 固定车辆配置库
        self.ego_vehicles = [] # 固定车辆配置库

        self.parse_object_definitions() # 解析配置文件
        self.set_map_and_observer_view()

        # 车辆生成频率
        self.spawn_interval_left = 5.0
        self.spawn_interval_right = 2.0
        self.spawn_interval_up = 3.0
        self.spawn_interval_down = 2.0
        # 车辆计数和配置管理
        self.vehicle_count_left = 0
        self.vehicle_count_right = 0
        self.vehicle_count_up = 0
        self.vehicle_count_down = 0

        self.max_vehicles = self.get_param('max_vehicles',30)      # 最大车辆数
        self.vehicle_configs = {}      # key: role_name → msg
        self.unacked_roles = set()     # 当前还没收到 ACK 的车辆
        self.timer_started = False
        self.timer = None
        # 创建生成对象和销毁对象客户端
        self.spawn_object_client = self.new_client(SpawnObject, "/carla/spawn_object")
        self.destroy_object_client = self.new_client(DestroyObject, "/carla/destroy_object")
        self.reach_goal_subscriber = self.new_subscription( VehicleGoalStatus, "/carla/goal_status", self.check_vehicle_status, qos_profile=10) 
        qos_profile = ros_compatibility.qos.QoSProfile(depth=10, durability=ros_compatibility.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.vehicle_config_publisher = self.new_publisher(VehicleConfig, '/carla/vehicle_config', qos_profile)
        self.ack_sub = self.new_subscription(VehicleConfigAck,"/driver_model_manager/ack",self._on_ack,qos_profile =10)

    def _on_ack(self, msg):
        role = msg.role_name
        if role in self.unacked_roles:
            self.unacked_roles.remove(role)
            print(f"[ScenarioExecutor] ACK received for {role}, remaining={len(self.unacked_roles)}")

    def parse_object_definitions(self):
        """
        Parses the object definitions from the JSON file.

        :param definition_file: Path to the JSON file containing object definitions
        :return: Parsed data as a dictionary
        """
        if not self.objects_definition_file or not os.path.exists(self.objects_definition_file):
            raise RuntimeError(f"Could not read object definitions from {self.objects_definition_file}")
        
        with open(self.objects_definition_file, 'r') as handle:
            json_actors = json.load(handle)
        # ✅ 添加基本配置的解析
        self.map_name = json_actors.get("map_name", "Town01")
        self.scene_type = json_actors.get("scene_type", None) 
        if not self.scene_type:
            raise RuntimeError("Error: 'scene_type' is not defined in the JSON configuration file or is empty.")
        self.observer_view = json_actors.get("observer_view", {
            "x": 0.0, "y": 0.0, "z": 50.0,
            "roll": 0.0, "pitch": -90.0, "yaw": 0.0
        })
        for object in json_actors["objects"]:
            if object["name"] == "global_sensors":
                for sensor in object["data"]:
                    if sensor["type"] == "sensor.pseudo.actor_list" and self.spawn_sensors_only:
                        self.found_sensor_actor_list = True
                    self.global_sensors.append(sensor)
            elif object["name"] == "cruise_speeds":
                self.cruise_speeds.extend(object["data"])
            elif object["name"] == "driver_model":
                self.driver_models.extend(object["data"])
            elif object["name"] == "vehicle_spawn_and_target_points_left":
                for entry in object["data"]:
                    spawn_point = entry["spawn_point"]
                    goal_point = entry["goal_point"]
                    self.vehicle_spawn_and_target_points_left.append((spawn_point, goal_point))
            elif object["name"] == "vehicle_spawn_and_target_points_right":
                for entry in object["data"]:
                    spawn_point = entry["spawn_point"]
                    goal_point = entry["goal_point"]
                    self.vehicle_spawn_and_target_points_right.append((spawn_point, goal_point))
            elif object["name"] == "vehicle_spawn_and_target_points_up":
                for entry in object["data"]:
                    spawn_point = entry["spawn_point"]
                    goal_point = entry["goal_point"]
                    self.vehicle_spawn_and_target_points_up.append((spawn_point, goal_point))
            elif object["name"] == "vehicle_spawn_and_target_points_down":
                for entry in object["data"]:
                    spawn_point = entry["spawn_point"]
                    goal_point = entry["goal_point"]
                    self.vehicle_spawn_and_target_points_down.append((spawn_point, goal_point))
            elif object["name"] == "vehicle_types":
                self.vehicle_types.extend(object["data"])
            elif object["name"] == "vehicle_sensors":
                self.vehicle_sensors.extend(object["data"])
            elif object["name"] == "vehicles":
                self.vehicles.extend(object["data"])
            elif object["name"] == "ego_vehicles":
                self.ego_vehicles.extend(object["data"])
            else:
                self.logwarn(
                    f"Object with type {object['name']} is not global_sensors, vehicle_spawn_and_target_points, vehicle_types, vehicle_sensors or vehicles, ignoring."
                )

        if self.spawn_sensors_only is True and self.found_sensor_actor_list is False:
            raise RuntimeError("Parameter 'spawn_sensors_only' enabled, " +
                               "but 'sensor.pseudo.actor_list' is not instantiated, add it to your config file.")
    def set_map_and_observer_view(self):
        self.loginfo(f"Setting map: {self.map_name} and observer view.")
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        
        # 切换地图
        if client.get_world().get_map().name.split("/")[-1] != self.map_name.split("/")[-1]:
            self.loginfo(f"Loading map: {self.map_name}")  
            client.load_world(self.map_name)
            client.get_world().tick()
            time.sleep(0.1)
        else:
            self.loginfo(f"Map {self.map_name} already loaded")

        # 设置观察视角
        self.world = client.get_world()
        spectator = self.world.get_spectator()
        location = carla.Location(
            x=self.observer_view["x"],
            y=self.observer_view["y"],
            z=self.observer_view["z"])
        rotation = carla.Rotation(
            roll=self.observer_view["roll"],
            pitch=self.observer_view["pitch"],
            yaw=self.observer_view["yaw"])
        spectator.set_transform(carla.Transform(location, rotation))
        self.loginfo(f"Map spectator already loaded")


    def spawn_vehicles_left(self, event=None):
        self.spawn_vehicles_from_list(self.vehicle_spawn_and_target_points_left, self.vehicle_count_left, 'L')

    def spawn_vehicles_right(self, event=None):
        self.spawn_vehicles_from_list(self.vehicle_spawn_and_target_points_right, self.vehicle_count_right, 'R')

    def spawn_vehicles_up(self, event=None):
        self.spawn_vehicles_from_list(self.vehicle_spawn_and_target_points_up, self.vehicle_count_up, 'U')

    def spawn_vehicles_down(self, event=None):
        self.spawn_vehicles_from_list(self.vehicle_spawn_and_target_points_down, self.vehicle_count_down, 'D')

    def spawn_vehicles_from_list(self, route_list, vehicle_count, direction):
        """ 从指定的路线库中生成车辆 """
        if direction == 'L':
            if self.vehicle_count_left >= self.max_vehicles :
                self.loginfo(f"左侧车辆数已达上限，不再生成")
                return
            vehicle_role_name = f'heroL{self.vehicle_count_left}'
        elif direction == 'R':
            if self.vehicle_count_right >= self.max_vehicles:
                self.loginfo(f"右侧车辆数已达上限，不再生成")
                return
            vehicle_role_name = f'heroR{self.vehicle_count_right}'
        elif direction == 'U':
            if self.vehicle_count_up >= self.max_vehicles:
                self.loginfo(f"上方车辆数已达上限，不再生成")
                return
            vehicle_role_name = f'heroU{self.vehicle_count_up}'
        elif direction == 'D':
            if self.vehicle_count_down >= self.max_vehicles:
                self.loginfo(f"下方车辆数已达上限，不再生成")
                return
            vehicle_role_name = f'heroD{self.vehicle_count_down}'
        else:
            self.logwarn("未知的车辆生成方向")
            return
        if not route_list:
            self.logwarn("指定的路线库为空，无法生成车辆")
            return
        driver_model = random.choice(self.driver_models)
        vehicle_type = random.choice(self.vehicle_types)
        spawn_and_target_point = random.choice(route_list)
        spawn_point = copy.deepcopy(spawn_and_target_point[0])
        goal_point = copy.deepcopy(spawn_and_target_point[1])

        carla_spawn_location = carla.Location(spawn_point["x"], spawn_point["y"], spawn_point["z"])
        carla_spawn_rotation = carla.Rotation(spawn_point["pitch"], spawn_point["yaw"], spawn_point["roll"])
        # 基于原始位置和角度，向前偏移 2 米生成新位置
        yaw_rad = math.radians(carla_spawn_rotation.yaw)
        offset_x = 2.0 * math.cos(yaw_rad)
        offset_y = 2.0 * math.sin(yaw_rad)
        offset_location = carla.Location(
            x=carla_spawn_location.x + offset_x,
            y=carla_spawn_location.y + offset_y,
            z=carla_spawn_location.z
        )
        carla_spawn_transform = carla.Transform(offset_location, carla_spawn_rotation)

        if self.is_spawn_box_occupied(carla_spawn_transform, box_length=8.0, box_width=3.0):
            self.loginfo(f"跳过车辆 {vehicle_role_name}：生成区域已被占用")
            return

        vehicle_config = VehicleConfig()
        vehicle_config.total_vehicles = -1
        vehicle_config.role_name = vehicle_role_name
        vehicle_config.scene_type = self.scene_type
        vehicle_config.driver_model = driver_model
        vehicle_config.cruise_speed.data = random.choice(self.cruise_speeds)

        vehicle_config.spawn_point.header.frame_id = "map"
        ros_spawn_transform = carla_transform_to_ros_transform(carla.Transform(carla_spawn_location, carla_spawn_rotation))
        vehicle_config.spawn_point.pose.position.x = ros_spawn_transform.translation.x
        vehicle_config.spawn_point.pose.position.y = ros_spawn_transform.translation.y
        vehicle_config.spawn_point.pose.position.z = ros_spawn_transform.translation.z
        vehicle_config.spawn_point.pose.orientation.x = ros_spawn_transform.rotation.x
        vehicle_config.spawn_point.pose.orientation.y = ros_spawn_transform.rotation.y
        vehicle_config.spawn_point.pose.orientation.z = ros_spawn_transform.rotation.z
        vehicle_config.spawn_point.pose.orientation.w = ros_spawn_transform.rotation.w

        vehicle_config.goal_point.header.frame_id = "map"
        carla_goal_location = carla.Location(goal_point["x"], goal_point["y"], goal_point["z"])
        carla_goal_rotation = carla.Rotation(goal_point["pitch"], goal_point["yaw"], goal_point["roll"])
        ros_goal_transform = carla_transform_to_ros_transform(carla.Transform(carla_goal_location, carla_goal_rotation))
        vehicle_config.goal_point.pose.position.x = ros_goal_transform.translation.x
        vehicle_config.goal_point.pose.position.y = ros_goal_transform.translation.y
        vehicle_config.goal_point.pose.position.z = ros_goal_transform.translation.z
        vehicle_config.goal_point.pose.orientation.x = ros_goal_transform.rotation.x
        vehicle_config.goal_point.pose.orientation.y = ros_goal_transform.rotation.y
        vehicle_config.goal_point.pose.orientation.z = ros_goal_transform.rotation.z
        vehicle_config.goal_point.pose.orientation.w = ros_goal_transform.rotation.w

        request = roscomp.get_service_request(SpawnObject)
        request.type = vehicle_type
        request.id = vehicle_role_name
        request.attach_to = 0
        request.random_pose = False
        request.transform = self.create_spawn_point(
            spawn_point.get("x"),
            spawn_point.get("y"),
            spawn_point.get("z"),
            spawn_point.get("roll", 0.0),
            spawn_point.get("pitch", 0.0),
            spawn_point.get("yaw", 0.0))
        if driver_model == "manual_driver":
            request.attributes.append(
            KeyValue(key=str("color"), value=str("0,0,0")))
        elif driver_model == "auto_driver":
            colors = ["255,0,0",   # 红色
                    "0,0,0",     # 黑色
                    "0,0,255"]   # 蓝色
            color = random.choice(colors)

            request.attributes.append(
                KeyValue(key="color", value=color)
            )
        response_id = self.spawn_object(request)
        if response_id != -1:
            vehicle_config.carla_id = response_id
            try:
                self.spawn_sensors(self.vehicle_sensors, response_id)
            except KeyError:
                self.logwarn(f"车辆 {vehicle_role_name} 没有传感器配置")
            self.vehicle_config_publisher.publish(vehicle_config)
            self.loginfo(f"{vehicle_config.role_name}:ID({vehicle_config.carla_id})车辆CARLA生成成功,发布车辆配置信息.")
            if direction == 'L':
                self.vehicle_count_left += 1
            elif direction == 'R':
                self.vehicle_count_right += 1
            elif direction == 'U':
                self.vehicle_count_up += 1
            elif direction == 'D':
                self.vehicle_count_down += 1
        else:
            self.logwarn(f"{vehicle_role_name}:ID({vehicle_config.carla_id})车辆生成失败！")

    def spawn_vehicles_from_vehicles(self):
        if not self.vehicles_spawned:
            for vehicle in self.vehicles:
                if vehicle.get("is_random_behavior_vehicle", False):
                    override_role = self.get_param("random_behavior_role_name", "")
                    if override_role:
                        vehicle["role_name"] = override_role
                    else:
                        self.logwarn(f"未提供 random_behavior_role_name")
                spawn_object_request = roscomp.get_service_request(SpawnObject)
                spawn_object_request.type = vehicle["type"]
                spawn_object_request.id = vehicle["role_name"]
                spawn_object_request.attach_to = 0
                spawn_object_request.random_pose = False
                spawn_point = None

                # check if there's a spawn_point corresponding to this vehicle
                spawn_point_param = self.get_param("spawn_point_" + vehicle["role_name"], None)
                spawn_param_used = False
                if (spawn_point_param is not None):
                    # try to use spawn_point from parameters
                    spawn_point = self.check_spawn_point_param(spawn_point_param)
                    if spawn_point is None:
                        self.logwarn("{}: Could not use spawn point from parameters, ".format(vehicle["role_name"]) +
                                    "the spawn point from config file will be used.")
                    else:
                        self.loginfo("Spawn point from ros parameters")
                        spawn_param_used = True

                if "spawn_point" in vehicle and spawn_param_used is False:
                    try:
                        sp = vehicle["spawn_point"]

                        # ===============================
                        # 情况 1：单一 spawn_point（旧格式）
                        # ===============================
                        if isinstance(sp, dict) and all(k in sp for k in ["x", "y", "z", "roll", "pitch", "yaw"]):
                            p = sp
                            name = "single"

                        # ===============================
                        # 情况 2：多个 spawn_point（新格式）
                        # ===============================
                        elif isinstance(sp, dict) and len(sp) > 0:
                            name = random.choice(list(sp.keys()))
                            p = sp[name]

                            if not all(k in p for k in ["x", "y", "z", "roll", "pitch", "yaw"]):
                                raise ValueError(f"spawn_point {name} missing required fields")

                        else:
                            raise ValueError("invalid spawn_point format")

                        # ⭐ 统一保存“最终使用的点”
                        vehicle["_selected_spawn_point"] = p
                        vehicle["_selected_spawn_point_name"] = name

                        spawn_point = self.create_spawn_point(
                            p["x"], p["y"], p["z"],
                            p["roll"], p["pitch"], p["yaw"]
                        )

                        self.loginfo(
                            f"[{vehicle['role_name']}] 使用 spawn_point: {name} "
                            f"({p['x']:.1f}, {p['y']:.1f}, yaw={p['yaw']:.1f})"
                        )

                    except Exception as e:
                        self.logerr(
                            f"[{vehicle['role_name']}] spawn_point 解析失败: {e}，使用随机出生点"
                        )

                if spawn_point is None:
                    # pose not specified, ask for a random one in the service call
                    self.loginfo("Spawn point selected at random")
                    spawn_point = Pose()  # empty pose
                    spawn_object_request.random_pose = True

                spawn_object_request.transform = spawn_point
                if vehicle["driver_model"] == "manual_driver":
                    spawn_object_request.attributes.append(
                    KeyValue(key=str("color"), value=str("0,0,0")))
                elif vehicle["driver_model"] == "auto_driver":
                    spawn_object_request.attributes.append(
                    KeyValue(key=str("color"), value=str("255,0,0")))
                # 应用车辆属性 (颜色等)
                if "attributes" in vehicle:
                    for key, value in vehicle["attributes"].items():
                        spawn_object_request.attributes.append(
                        KeyValue(key=str(key), value=str(value)))
                response_id = self.spawn_object(spawn_object_request )
                if response_id != -1:
                    try:
                        self.spawn_sensors(vehicle["sensors"], response_id)#用的是车辆传感器库
                    except (KeyError):
                        self.logwarn(
                            "Object (type='{}', id='{}') has no 'vehicle_sensors' file, none will be spawned."
                            .format(spawn_object_request.type, spawn_object_request.id))
                    #####################################################################################################
                    spawn_point = copy.deepcopy(vehicle["_selected_spawn_point"])

                    spawn_key = vehicle["_selected_spawn_point_name"]
                    gp = vehicle["goal_point"]

                    # ===============================
                    # 情况 1：single goal_point（旧格式）
                    # ===============================
                    if isinstance(gp, dict) and all(k in gp for k in ["x", "y", "z", "roll", "pitch", "yaw"]):
                        goal_point = copy.deepcopy(gp)
                        goal_key = spawn_key  # 逻辑上绑定，但不从 dict 取

                    # ===============================
                    # 情况 2：multi goal_point（新格式）
                    # ===============================
                    elif isinstance(gp, dict) and spawn_key in gp:
                        goal_point = copy.deepcopy(gp[spawn_key])
                        goal_key = spawn_key

                    else:
                        raise ValueError(
                            f"[{vehicle['role_name']}] goal_point 格式错误，"
                            f"spawn_key={spawn_key}，goal_point keys={list(gp.keys())}"
                        )

                    vehicle_config = VehicleConfig()
                    vehicle_config.role_name = vehicle["role_name"]
                    if vehicle.get("is_random_behavior_vehicle", False):
                        vehicle_config.scene_class = vehicle["scene_class"]
                    else:
                        vehicle_config.scene_class = "none"
                    vehicle_config.is_random_behavior_vehicle = vehicle.get("is_random_behavior_vehicle", False)    
                    vehicle_config.scene_type = self.scene_type
                    vehicle_config.driver_model = vehicle["driver_model"]
                    vehicle_config.cruise_speed.data = vehicle["cruise_speed"]
                    vehicle_config.carla_id = response_id
                    vehicle_config.spawn_point.header.frame_id = "map"
                    vehicle_config.spawn_key = vehicle["_selected_spawn_point_name"]
                    carla_spawn_location = carla.Location(spawn_point["x"], spawn_point["y"], spawn_point["z"])
                    carla_spawn_rotation = carla.Rotation(spawn_point["pitch"], spawn_point["yaw"], spawn_point["roll"])
                    ros_spawn_transform = carla_transform_to_ros_transform(carla.Transform(carla_spawn_location, carla_spawn_rotation))
                    vehicle_config.spawn_point.pose.position.x = ros_spawn_transform.translation.x
                    vehicle_config.spawn_point.pose.position.y = ros_spawn_transform.translation.y
                    vehicle_config.spawn_point.pose.position.z = ros_spawn_transform.translation.z
                    vehicle_config.spawn_point.pose.orientation.x = ros_spawn_transform.rotation.x
                    vehicle_config.spawn_point.pose.orientation.y = ros_spawn_transform.rotation.y
                    vehicle_config.spawn_point.pose.orientation.z = ros_spawn_transform.rotation.z
                    vehicle_config.spawn_point.pose.orientation.w = ros_spawn_transform.rotation.w

                    vehicle_config.goal_point.header.frame_id = "map"
                    carla_goal_location = carla.Location(goal_point["x"], goal_point["y"], goal_point["z"])
                    carla_goal_rotation = carla.Rotation(goal_point["pitch"], goal_point["yaw"], goal_point["roll"])
                    ros_goal_transform = carla_transform_to_ros_transform(carla.Transform(carla_goal_location, carla_goal_rotation))
                    vehicle_config.goal_point.pose.position.x = ros_goal_transform.translation.x
                    vehicle_config.goal_point.pose.position.y = ros_goal_transform.translation.y
                    vehicle_config.goal_point.pose.position.z = ros_goal_transform.translation.z
                    vehicle_config.goal_point.pose.orientation.x = ros_goal_transform.rotation.x
                    vehicle_config.goal_point.pose.orientation.y = ros_goal_transform.rotation.y
                    vehicle_config.goal_point.pose.orientation.z = ros_goal_transform.rotation.z
                    vehicle_config.goal_point.pose.orientation.w = ros_goal_transform.rotation.w

                    # ================== 新增字段解析 ==================
                    # 1️⃣ path_id
                    if "path_id" in vehicle:
                        vehicle_config.path_id = vehicle["path_id"]
                        self.loginfo(f"[{vehicle['role_name']}] path_id: {vehicle_config.path_id}")
                    else:
                        vehicle_config.path_id = -1
                        self.logwarn(f"[{vehicle['role_name']}] 未设置 path_id，使用默认值 -1")
                    if "road_option" in vehicle:
                        vehicle_config.road_option = vehicle["road_option"]
                        self.loginfo(f"[{vehicle['role_name']}] road_option: {vehicle_config.road_option}")
                    else:
                        vehicle_config.road_option = -1
                        self.logwarn(f"[{vehicle['role_name']}] 未设置 road_option -1")
                    # 2️⃣ stop_line_point
                    if "stop_line_point" in vehicle:
                        p = vehicle["stop_line_point"]
                        carla_loc = carla.Location(p["x"], p["y"], p["z"])
                        carla_rot = carla.Rotation(p["pitch"], p["yaw"], p["roll"])
                        tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))

                        vehicle_config.stop_line_pose.position.x = tf.translation.x
                        vehicle_config.stop_line_pose.position.y = tf.translation.y
                        vehicle_config.stop_line_pose.position.z = tf.translation.z
                        vehicle_config.stop_line_pose.orientation.x = tf.rotation.x
                        vehicle_config.stop_line_pose.orientation.y = tf.rotation.y
                        vehicle_config.stop_line_pose.orientation.z = tf.rotation.z
                        vehicle_config.stop_line_pose.orientation.w = tf.rotation.w

                        self.loginfo(f"[{vehicle['role_name']}] stop_line_point: ({p['x']}, {p['y']}, {p['z']})")
                    else:
                        self.logwarn(f"[{vehicle['role_name']}] 未设置 stop_line_point")

                    # 3️⃣ exit_intersection_point
                    if "exit_intersection_point" in vehicle:
                        p = vehicle["exit_intersection_point"]
                        carla_loc = carla.Location(p["x"], p["y"], p["z"])
                        carla_rot = carla.Rotation(p["pitch"], p["yaw"], p["roll"])
                        tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))

                        vehicle_config.exit_intersection_pose.position.x = tf.translation.x
                        vehicle_config.exit_intersection_pose.position.y = tf.translation.y
                        vehicle_config.exit_intersection_pose.position.z = tf.translation.z
                        vehicle_config.exit_intersection_pose.orientation.x = tf.rotation.x
                        vehicle_config.exit_intersection_pose.orientation.y = tf.rotation.y
                        vehicle_config.exit_intersection_pose.orientation.z = tf.rotation.z
                        vehicle_config.exit_intersection_pose.orientation.w = tf.rotation.w

                        self.loginfo(f"[{vehicle['role_name']}] exit_intersection_point: ({p['x']}, {p['y']}, {p['z']})")
                    else:
                        self.logwarn(f"[{vehicle['role_name']}] 未设置 exit_intersection_point")

                    # ================== 新增：随意驾驶行为配置 ==================
                    if "random_behavior" in vehicle:
                        rb_info = vehicle["random_behavior"]
                        rb = RandomBehavior()
                        rb.type = rb_info.get("type", "")

                        params = rb_info.get("parameters", {})

                        # ✅ 通用参数（标量类型）
                        scalar_params = {k: v for k, v in params.items() if isinstance(v, (int, float, bool))}
                        rb.keys = list(scalar_params.keys())
                        rb.values = [float(v) for v in scalar_params.values()]
                        if"scene_type"in params:
                            rb.scene_type = params["scene_type"]
                        if "lane_change_point" in params:
                            lc_points = params["lane_change_point"]

                            if not isinstance(lc_points, dict):
                                self.logwarn(f"[{vehicle['role_name']}] lane_change_point 格式错误，必须是 { '{pointX: {...}}' } 结构")
                            else:
                                for name, p in lc_points.items():
                                    try:
                                        carla_loc = carla.Location(p["x"], p["y"], p["z"])
                                        carla_rot = carla.Rotation(p["pitch"], p["yaw"], p["roll"])
                                        tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))

                                        pose = geometry_msgs.msg.Pose()
                                        pose.position.x = tf.translation.x
                                        pose.position.y = tf.translation.y
                                        pose.position.z = tf.translation.z
                                        pose.orientation.x = tf.rotation.x
                                        pose.orientation.y = tf.rotation.y
                                        pose.orientation.z = tf.rotation.z
                                        pose.orientation.w = tf.rotation.w

                                        rb.lane_change_point.append(pose)
                                        self.loginfo(f"[{vehicle['role_name']}] lane_change_point {name}: ({p['x']:.1f}, {p['y']:.1f})")
                                    except KeyError as e:
                                        self.logwarn(f"[{vehicle['role_name']}] lane_change_point {name} 缺少字段 {e}")
                        else:
                            self.logwarn(f"[{vehicle['role_name']}] 未设置 lane_change_point")
                        # ✅ 新增：change_speed_point
                        if "change_speed_point" in params:
                            cs_points = params["change_speed_point"]

                            if not isinstance(cs_points, dict):
                                self.logwarn(f"[{vehicle['role_name']}] change_speed_point 格式错误，必须是 {{pointX: {{...}}}} 结构")
                            else:
                                for name, p in cs_points.items():
                                    try:
                                        carla_loc = carla.Location(p["x"], p["y"], p["z"])
                                        carla_rot = carla.Rotation(p["pitch"], p["yaw"], p["roll"])
                                        tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))

                                        pose = geometry_msgs.msg.Pose()
                                        pose.position.x = tf.translation.x
                                        pose.position.y = tf.translation.y
                                        pose.position.z = tf.translation.z
                                        pose.orientation.x = tf.rotation.x
                                        pose.orientation.y = tf.rotation.y
                                        pose.orientation.z = tf.rotation.z
                                        pose.orientation.w = tf.rotation.w

                                        rb.change_speed_point.append(pose)
                                        self.loginfo(f"[{vehicle['role_name']}] change_speed_point {name}: ({p['x']:.1f}, {p['y']:.1f})")
                                    except KeyError as e:
                                        self.logwarn(f"[{vehicle['role_name']}] change_speed_point {name} 缺少字段 {e}")
                        else:
                            self.logwarn(f"[{vehicle['role_name']}] 未设置 change_speed_point")  
                        # ✅ 新增：recovery_speed_point
                        if "recovery_speed_point" in params:
                            rs_points = params["recovery_speed_point"]

                            if not isinstance(rs_points, dict):
                                self.logwarn(f"[{vehicle['role_name']}] recovery_speed_point 格式错误，必须是 {{pointX: {{...}}}} 结构")
                            else:
                                for name, p in rs_points.items():
                                    try:
                                        carla_loc = carla.Location(p["x"], p["y"], p["z"])
                                        carla_rot = carla.Rotation(p["pitch"], p["yaw"], p["roll"])
                                        tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))

                                        pose = geometry_msgs.msg.Pose()
                                        pose.position.x = tf.translation.x
                                        pose.position.y = tf.translation.y
                                        pose.position.z = tf.translation.z
                                        pose.orientation.x = tf.rotation.x
                                        pose.orientation.y = tf.rotation.y
                                        pose.orientation.z = tf.rotation.z
                                        pose.orientation.w = tf.rotation.w

                                        rb.recovery_speed_point.append(pose)
                                        self.loginfo(f"[{vehicle['role_name']}] recovery_speed_point {name}: ({p['x']:.1f}, {p['y']:.1f})")
                                    except KeyError as e:
                                        self.logwarn(f"[{vehicle['role_name']}] recovery_speed_point {name} 缺少字段 {e}")
                        else:
                            self.logwarn(f"[{vehicle['role_name']}] 未设置 recovery_speed_point")

                        # ✅ 特殊参数：other_lane
                        if "other_lane" in params:
                            other_lane = params["other_lane"]
                            for lane_name, lane_info in other_lane.items():
                                try:
                                    start = lane_info["start_point"]
                                    end = lane_info["end_point"]

                                    # 创建 ROS Pose
                                    start_pose = geometry_msgs.msg.Pose()
                                    end_pose = geometry_msgs.msg.Pose()

                                    # ---- start_point ----
                                    carla_loc = carla.Location(start["x"], start["y"], start["z"])
                                    carla_rot = carla.Rotation(start["pitch"], start["yaw"], start["roll"])
                                    tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))
                                    start_pose.position.x = tf.translation.x
                                    start_pose.position.y = tf.translation.y
                                    start_pose.position.z = tf.translation.z
                                    start_pose.orientation.x = tf.rotation.x
                                    start_pose.orientation.y = tf.rotation.y
                                    start_pose.orientation.z = tf.rotation.z
                                    start_pose.orientation.w = tf.rotation.w

                                    # ---- end_point ----
                                    carla_loc = carla.Location(end["x"], end["y"], end["z"])
                                    carla_rot = carla.Rotation(end["pitch"], end["yaw"], end["roll"])
                                    tf = carla_transform_to_ros_transform(carla.Transform(carla_loc, carla_rot))
                                    end_pose.position.x = tf.translation.x
                                    end_pose.position.y = tf.translation.y
                                    end_pose.position.z = tf.translation.z
                                    end_pose.orientation.x = tf.rotation.x
                                    end_pose.orientation.y = tf.rotation.y
                                    end_pose.orientation.z = tf.rotation.z
                                    end_pose.orientation.w = tf.rotation.w

                                    # ✅ 保存到 vehicle_config.random_behavior 的附加字段
                                    lane = LaneInfo()
                                    lane.name = lane_name
                                    lane.start_point = start_pose
                                    lane.end_point = end_pose
                                    rb.other_lanes.append(lane)

                                    self.loginfo(
                                        f"[{vehicle['role_name']}] 解析 other_lane: {lane_name} "
                                        f"start=({start['x']:.1f},{start['y']:.1f}) "
                                        f"→ end=({end['x']:.1f},{end['y']:.1f})"
                                    )
                                except KeyError as e:
                                    self.logwarn(f"[{vehicle['role_name']}] 解析 other_lane {lane_name} 出错: 缺少字段 {e}")
                        # ================== 新增：ignored_vehicle_roles ==================
                        if "ignored_vehicle_roles" in params:
                            roles = params["ignored_vehicle_roles"]

                            if isinstance(roles, dict):
                                for role_name in roles.keys():
                                    if isinstance(role_name, str):
                                        rb.ignored_vehicle_roles.append(role_name)
                                    else:
                                        self.logwarn(
                                            f"[{vehicle['role_name']}] ignored_vehicle_roles 中存在非字符串 key: {role_name}"
                                        )

                                # 打印最终解析结果
                                if rb.ignored_vehicle_roles:
                                    self.loginfo(
                                        f"[{vehicle['role_name']}] ignored_vehicle_roles 解析完成: "
                                        f"{rb.ignored_vehicle_roles}"
                                    )
                                else:
                                    self.logwarn(
                                        f"[{vehicle['role_name']}] ignored_vehicle_roles dict 为空"
                                    )
                            else:
                                self.logwarn(
                                    f"[{vehicle['role_name']}] ignored_vehicle_roles 格式错误："
                                    f"必须是 dict，例如 {{'hero2': 'hero2'}}"
                                )

                        # ================== 打印 random_behavior 解析结果 ==================
                        self.loginfo(f"[{vehicle['role_name']}] 随意驾驶行为: {rb.type}, 标量参数: {scalar_params}")
                        self.loginfo(f"[{vehicle['role_name']}] ===== RandomBehavior Dump =====")
                        self.loginfo(f"type: {rb.type}")
                        self.loginfo(f"keys: {rb.keys}")
                        self.loginfo(f"values: {rb.values}")

                        self.loginfo(f"lane_change_point size: {len(rb.lane_change_point)}")
                        self.loginfo(f"change_speed_point size: {len(rb.change_speed_point)}")
                        self.loginfo(f"recovery_speed_point size: {len(rb.recovery_speed_point)}")
                        self.loginfo(f"other_lanes size: {len(rb.other_lanes)}")
                        self.loginfo(f"ignored_vehicle_roles: {rb.ignored_vehicle_roles}")

                        # 可选：打印点的坐标（只打印前几个）
                        for i, p in enumerate(rb.change_speed_point):
                            self.loginfo(
                                f"  change_speed_point[{i}]: "
                                f"({p.position.x:.2f}, {p.position.y:.2f}, {p.position.z:.2f})"
                            )

                        for lane in rb.other_lanes:
                            self.loginfo(
                                f"  other_lane: {lane.name}, "
                                f"start=({lane.start_point.position.x:.1f},{lane.start_point.position.y:.1f}), "
                                f"end=({lane.end_point.position.x:.1f},{lane.end_point.position.y:.1f})"
                            )

                        self.loginfo(f"[{vehicle['role_name']}] ===== End RandomBehavior Dump =====")
                    else:
                        rb = RandomBehavior()
                        rb.type = "none"
                        rb.keys = []
                        rb.values = []
                    vehicle_config.random_behavior = rb
                    role = vehicle_config.role_name
                    self.vehicle_configs[role] = vehicle_config
                    self.unacked_roles.add(role)
                    #####################################################################################################
                    self.loginfo(f"{vehicle_config.role_name}：发布车辆配置信息:{vehicle_config.carla_id}")
                else:
                    self.logwarn(f"车辆 {vehicle['role_name']} 生成失败")
            self.vehicles_spawned = True  
            # ⭐ 设置 total_vehicles（所有车辆数量）
            total = len(self.vehicle_configs)
            for role, cfg in self.vehicle_configs.items():  
                cfg.total_vehicles = total
                self.vehicle_config_publisher.publish(cfg)
                self.loginfo(f"[Init] 发送带 total_vehicles={total} 的配置: {role}")
            # ⭐ Timer 只启动一次
            if not self.timer_started:
                self.timer = rospy.Timer(rospy.Duration(0.2), self.resend_unacked)
                self.timer_started = True
                self.loginfo("🔁 Timer started: resending unacked VehicleConfigs every 0.2s")

    def resend_unacked(self, event):
        for role in list(self.unacked_roles):
            msg = self.vehicle_configs[role]
            self.vehicle_config_publisher.publish(msg)
            rospy.loginfo(f"[ScenarioExecutor] 111111111111111111111111111111111111111111111111111111111111111111111111111 {role}")
            
    def spawn_global_sensors(self, event=None):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """          
        if not self.global_sensors_spawned:
            self.spawn_sensors(self.global_sensors)
            self.global_sensors_spawned = True
        self.loginfo("All global_sensors spawned!")

    def spawn_sensors(self, sensors, attached_vehicle_role_name=None):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_role_name: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        # rospy.loginfo(f"sensors: {sensors}")
        # rospy.loginfo(f"Number of sensor tuples: {len(sensors)}")

        sensor_names = []
        for sensor_spec in sensors:
            if not roscomp.ok():
                break
            try:
                # rospy.loginfo(f"Processing sensor spec: {sensor_spec}")
                sensor_spec_copy = copy.deepcopy(sensor_spec)  # 创建深拷贝
                # self.loginfo(f"Copied sensor spec: {sensor_spec_copy}")

                sensor_type = str(sensor_spec_copy.pop("type"))
                sensor_id = str(sensor_spec_copy.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError
                sensor_names.append(sensor_name)

                if attached_vehicle_role_name is None and "pseudo" not in sensor_type:
                    spawn_point = sensor_spec_copy.pop("spawn_point")
                    sensor_transform = self.create_spawn_point(
                        spawn_point.pop("x"),
                        spawn_point.pop("y"),
                        spawn_point.pop("z"),
                        spawn_point.pop("roll", 0.0),
                        spawn_point.pop("pitch", 0.0),
                        spawn_point.pop("yaw", 0.0))
                else:
                    # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                    spawn_point = sensor_spec_copy.pop("spawn_point", 0)
                    if spawn_point == 0:
                        sensor_transform = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    else:
                        sensor_transform = self.create_spawn_point(
                            spawn_point.pop("x", 0.0),
                            spawn_point.pop("y", 0.0),
                            spawn_point.pop("z", 0.0),
                            spawn_point.pop("roll", 0.0),
                            spawn_point.pop("pitch", 0.0),
                            spawn_point.pop("yaw", 0.0))

                spawn_object_request = roscomp.get_service_request(SpawnObject)
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = attached_vehicle_role_name if attached_vehicle_role_name is not None else 0
                spawn_object_request.transform = sensor_transform
                spawn_object_request.random_pose = False  # never set a random pose for a sensor

                attached_objects = []
                for attribute, value in sensor_spec_copy.items():
                    if attribute == "attached_objects":
                        for attached_object in sensor_spec_copy["attached_objects"]:
                            attached_objects.append(attached_object)
                        continue
                    spawn_object_request.attributes.append(
                        KeyValue(key=str(attribute), value=str(value)))

                response_id = self.spawn_object(spawn_object_request)

                # if response_id == -1:
                #     raise RuntimeError(response.error_string)

                if attached_objects:
                    # spawn the attached objects
                    self.spawn_sensors(attached_objects, response_id)

            except KeyError as e:
                self.logerr(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
                continue

            except RuntimeError as e:
                self.logerr(
                    "Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            except NameError:
                self.logerr("Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                    sensor_id))
                continue


    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        # 将输入参数转换为 CARLA 的 Transform
        location = carla.Location(x=x, y=y, z=z)
        rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
        transform = carla.Transform(location, rotation)
        spawn_point = carla_transform_to_ros_pose(transform)
        return spawn_point

    def check_vehicle_status(self, msg):
        """ 检查车辆是否到达目标点，若到达则销毁"""
        if msg.reached_goal:  # 车辆接近目标点，触发销毁
            self.loginfo(f"销毁车辆: {msg.carla_id}")
            self.destroy_object(msg.carla_id)   

    def spawn_object(self, spawn_object_request):
        response_id = -1
        response = self.call_service(self.spawn_object_client, spawn_object_request, spin_until_response_received=True)
        response_id = response.id
        if response_id != -1:
            self.loginfo("Object (type='{}', id='{}') spawned successfully as {}.".format(
                spawn_object_request.type, spawn_object_request.id, response_id))
        else:
            self.logwarn("Error while spawning object (type='{}', id='{}').".format(
                spawn_object_request.type, spawn_object_request.id))
            # raise RuntimeError(response.error_string)
        return response_id
    
    def destroy_object(self, carla_id):
        request = roscomp.get_service_request(DestroyObject)
        request.id = carla_id
        self.call_service(self.destroy_object_client, request, spin_until_response_received=True)  

    def is_point_in_rotated_box(self, point, box_center, length, width, yaw_deg):
        """
        判断一个点是否在旋转矩形框内
        :param point: carla.Location，要判断的点
        :param box_center: carla.Location，矩形中心
        :param length: 矩形长（沿车辆前向）
        :param width: 矩形宽（车辆横向）
        :param yaw_deg: 车头方向角度（CARLA中为角度制）
        :return: bool
        """
        dx = point.x - box_center.x
        dy = point.y - box_center.y
        yaw_rad = math.radians(yaw_deg)

        # 将点旋转到矩形的局部坐标系下
        local_x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        local_y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        return (-length/2 <= local_x <= length/2) and (-width/2 <= local_y <= width/2)        

    def is_spawn_box_occupied(self, spawn_transform, box_length=8.0, box_width=3.0):
        """
        判断以 spawn_transform 为中心的旋转矩形范围内是否有车
        :param world: carla.World
        :param spawn_transform: carla.Transform，车辆将要生成的位置
        :param box_length: 检测框长度（前后方向）
        :param box_width: 检测框宽度（左右方向）
        :return: True 如果区域内已有车
        """
        center = spawn_transform.location
        yaw = spawn_transform.rotation.yaw

        vehicle_list = self.world.get_actors().filter('vehicle.*')
        for vehicle in vehicle_list:
            if not vehicle.is_alive:
                continue
            loc = vehicle.get_location()
            if self.is_point_in_rotated_box(loc, center, box_length, box_width, yaw):
                return True
        return False
    
    def destroy(self):
        """
        释放 DynamicCarSpawner 相关的资源。
        """
        self.loginfo("Releasing ROS resources for DynamicCarSpawner...")

        try:
            # 停止 ROS 定时器，防止继续生成车辆
            if self.spawn_timer:
                self.loginfo("Stopping ROS timer")
                self.spawn_timer.shutdown()
                self.spawn_timer = None  # 释放定时器资源

            # 关闭 ROS 话题发布者
            if hasattr(self, "vehicle_config_publisher"):
                self.loginfo("Shutting down ROS publisher /carla/vehicle_config")
                self.destroy_publisher(self.vehicle_config_publisher)
            # 关闭 ROS 订阅者
            if hasattr(self, "reach_goal_subscriber"):
                self.loginfo("Shutting down ROS subscriber /carla/goal_status")
                self.destroy_subscription(self.reach_goal_subscriber)

            # 关闭 ROS 服务客户端
            if hasattr(self, "spawn_object_client"):
                self.loginfo("Shutting down ROS service client /carla/spawn_object")
                self.spawn_object_client.close()

            if hasattr(self, "destroy_object_client"):
                self.loginfo("Shutting down ROS service client /carla/destroy_object")
                self.spawn_object_client.close()

            # 清空存储的车辆相关数据
            self.loginfo("Successfully released all ROS resources for ScenarioExecutor.")
        except Exception as e:
            self.logwarn(f"Exception occurred while releasing resources: {e}")

import time

def wait_for_driver_manager(timeout_sec=10.0):
    pub = rospy.Publisher('/carla/vehicle_config', VehicleConfig, queue_size=1)
    rospy.loginfo("🔁 Waiting for DriverModelManager to subscribe to /carla/vehicle_config...")
    
    start_time = time.time()
    rate = rospy.Rate(10)
    while (time.time() - start_time) < timeout_sec:
        if pub.get_num_connections() > 0:
            rospy.loginfo("✅ DriverModelManager is ready.")
            return True
        rospy.loginfo_throttle(1.0, "⏳ Still waiting for /carla/vehicle_config subscriber...")
        rate.sleep()
    
    rospy.logwarn("⚠️ Timeout: DriverModelManager not ready.")
    return False

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):
    roscomp.init("spawn_objects", args=args)

    # 确保 driver_model_manager 准备就绪
    if not wait_for_driver_manager(timeout_sec=10.0):
        roscomp.logerr("❌ No subscriber on /carla/vehicle_config. Exiting.")
        roscomp.shutdown()
        return
    
    try:
        node = ScenarioExecutor()
        node.logerr("ScenarioExecutor initialized successfully!")
        # 生成全局传感器
        node.spawn_global_sensors()
        # 动态生成时确认driver_manager是否接收到车辆信息
        if node.dynamic_spawn_flag:
            # 启动定时器
            # if node.vehicle_spawn_and_target_points_left:
            #     node.spawn_timer_left = node.new_timer(node.spawn_interval_left, node.spawn_vehicles_left)
            #     node.loginfo(f"左侧车辆生成定时器已启动")
            # if node.vehicle_spawn_and_target_points_up:
            #     node.spawn_timer_up = node.new_timer(node.spawn_interval_up, node.spawn_vehicles_up)
            #     node.loginfo(f"上方车辆生成定时器已启动")
            if node.vehicle_spawn_and_target_points_right:
                node.spawn_timer_right = node.new_timer(node.spawn_interval_right, node.spawn_vehicles_right)
                node.loginfo(f"右侧车辆生成定时器已启动")
            if node.vehicle_spawn_and_target_points_down:
                node.spawn_timer_down = node.new_timer(node.spawn_interval_down, node.spawn_vehicles_down)
                node.loginfo(f"下方车辆生成定时器已启动")
        else: 
            node.loginfo("Dynamic vehicle spawning mode is disabled, all vehicles will be spawned at once.")
            node.spawn_vehicles_from_vehicles()

        roscomp.on_shutdown(node.destroy)
        node.spin()
    except Exception as e:
        roscomp.logerr(f"Unexpected error: {e}")
    finally:
        roscomp.shutdown()

if __name__ == '__main__':
    main()

