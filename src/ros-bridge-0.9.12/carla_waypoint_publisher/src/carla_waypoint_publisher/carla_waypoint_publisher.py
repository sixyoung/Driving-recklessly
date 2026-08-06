import math
import sys
import threading
import os
from datetime import datetime

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetWaypoint, GetActorWaypoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from driver_models_types.msg import WaypointWithOption
from driver_models_types.srv import PathWithOptionsService

class CarlaToRosWaypointConverter(CompatibleNode):
    """
    This class generates a plan of waypoints to follow for each vehicle.
    """
    def __init__(self):
        """
        Constructor for CarlaToRosWaypointConverter

        :param role_name: The role name of the vehicle
        :param world: The CARLA world
        """
        super(CarlaToRosWaypointConverter, self).__init__(f'carla_waypoint_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
   
        self.get_waypoint_service = self.new_service(
            GetWaypoint,
            '/carla_waypoint_publisher/get_waypoint',
            self.get_waypoint)

        self.get_actor_waypoint_service = self.new_service(
            GetActorWaypoint,
            '/carla_waypoint_publisher/get_actor_waypoint',
            self.get_actor_waypoint)

        self.get_path_service = self.new_service(
            PathWithOptionsService,
            '/carla_waypoint_publisher/get_path',
            self.get_path)

    def destroy(self):
        """
        Destructor for cleanup
        """
        self.ego_vehicle = None
        self.destroy_service(self.get_waypoint_service)
        self.destroy_service(self.get_actor_waypoint_service)
        self.destroy_service(self.get_path_service)

    def get_waypoint(self, req, response=None):
        """
        Get the waypoint for a location
        """
        carla_position = carla.Location(req.location.x, -req.location.y, req.location.z)
        carla_waypoint = self.map.get_waypoint(carla_position)

        response = roscomp.get_service_response(GetWaypoint)
        response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        return response

    def get_actor_waypoint(self, req, response=None):
        """
        Convenience method to get the waypoint for an actor
        """
        actor = self.world.get_actors().find(req.id)
        response = roscomp.get_service_response(GetActorWaypoint)
        if actor:
            carla_waypoint = self.map.get_waypoint(actor.get_location())
            response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
            response.waypoint.is_junction = carla_waypoint.is_junction
            response.waypoint.road_id = carla_waypoint.road_id
            response.waypoint.section_id = carla_waypoint.section_id
            response.waypoint.lane_id = carla_waypoint.lane_id
        else:
            self.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response
    
    def get_path(self, req):
        # 解析请求中的目标点
        response = roscomp.get_service_response(PathWithOptionsService)
        role_name =req.role_name
        start_pose = req.start
        goal_pose = req.goal
        carla_start = trans.ros_pose_to_carla_transform(start_pose)
        carla_goal = trans.ros_pose_to_carla_transform(goal_pose)
        hero = None
        route = self.calculate_route(carla_start, carla_goal, hero)
        if route != None:
            # 创建响应
            response.path.header.frame_id = "map"
            response.path.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
            response.success = True
            response.message = f"成功生成从车辆位置到目标点 ({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f}) 的路径，共 {len(route)} 个路径点"
            for wp in route:
                waypoint = WaypointWithOption()
                waypoint.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
                waypoint.road_option = wp[1].value
                response.path.waypoints.append(waypoint)
        else:
            response.success = False
            response.message = f"无法计算从车辆位置到目标点 ({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f}) 的路径"
        return response


    def save_waypoints_to_file(self, route, folder_path="waypoints", file_name="waypoints.txt"):
        """
        Save waypoints information to a file in a specific folder.
        The folder is created if it doesn't exist. If the file exists, it will append.
        """
        # Create folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Combine folder path and file name to get the full file path
        file_path = os.path.join(folder_path, file_name)
        
        # Check if the file exists
        file_exists = os.path.exists(file_path)

        # Open the file (if exists, append; otherwise, create a new file)
        mode = 'a' if file_exists else 'w'
        with open(file_path, mode) as file:
            # If the file doesn't exist, write a header
            if not file_exists:
                file.write("Waypoint Information\n")
                file.write("=" * 50 + "\n")

            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            file.write(f"\nData saved at {current_time}\n")
            file.write("=" * 100 + "\n")
            
            # Iterate over the waypoints in the route
            for i, waypoint in enumerate(route):
                # Assuming waypoint[0] is the carla.Waypoint
                is_junction = waypoint[0].is_junction
                location = waypoint[0].transform.location
                rotation = waypoint[0].transform.rotation
                lane_id = waypoint[0].lane_id
                road_id = waypoint[0].road_id
                section_id = waypoint[0].section_id
                road_option = waypoint[1]

                # Write waypoint details to the file
                file.write(f"Waypoint {i + 1}:\n")
                file.write(f"  Is junction: {is_junction}\n")
                file.write(f"  Rotation: ({rotation.roll}, {rotation.pitch}, {rotation.yaw})\n")
                file.write(f"  Location: ({location.x}, {location.y}, {location.z})\n")
                file.write(f"  Lane ID: {lane_id}\n")
                file.write(f"  Road ID: {road_id}\n")
                file.write(f"  Section ID: {section_id}\n")
                file.write(f"  Road Option: {road_option}\n")
                file.write("-" * 50 + "\n")


    def calculate_route(self, start, goal, hero):
        """
        Calculate a route from the current location to 'goal'
        """
        sampling_resolution = 1.0  # 单位：米，每隔1.0m一个路径点
        grp = GlobalRoutePlanner(self.world.get_map(), sampling_resolution=sampling_resolution)
        route = grp.trace_route(carla.Location(start.location.x,
                                               start.location.y,
                                               start.location.z),
                                carla.Location(goal.location.x,
                                               goal.location.y,
                                               goal.location.z))
        # self.save_waypoints_to_file(route, "/home/hzq/waypoint", "waypoints.txt")
        return route
    
    def connect_to_carla(self):
        """
        Connect to CARLA world
        """
        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr(f"Error while waiting for world info: {e}")
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo(f"CARLA world available. Trying to connect to {host}:{port}")

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            # Try to get the world from CARLA
            self.world = carla_client.get_world()           
            self.loginfo(f"Connected to CARLA world at {host}:{port}")

        except RuntimeError as e:
            self.logerr(f"Error while connecting to CARLA: {e}")
            raise e
             
def main(args=None):
    """
    Main function to initialize the multi-vehicle manager and run the logic.
    """
    roscomp.init('carla_waypoint_publisher', args)

    waypoint_converter = None
    try:
        waypoint_converter = CarlaToRosWaypointConverter()
        waypoint_converter.logerr("CarlaWaypointPublisher initialized successfully!")
        waypoint_converter.spin()
        
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shutdown.")
    finally:
        roscomp.loginfo("Shutting down.")
        if waypoint_converter:
            waypoint_converter.destroy()
        roscomp.shutdown()

if __name__ == "__main__":
    main()
