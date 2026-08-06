#ifndef DRIVER_MODEL_DATA_POOL_H
#define DRIVER_MODEL_DATA_POOL_H

#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Location.h>
#include <carla/client/Waypoint.h>
#include <carla/client/TrafficLight.h>
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>

struct FollowableVehicle {
    int32_t id;
    carla::geom::Transform transform; 
    carla::SharedPtr<carla::client::Waypoint> waypoint;  
    std::vector<carla::geom::Location> corners; 
    double vel_;  
    double t_ini; 
    double t_end; 
    double s_ini; 
    double s_end; 
    double width_; 
    double length_;
    FollowableVehicle() = default;
};

struct InterferingVehicle {
    int id;
    carla::geom::Transform transform;
    carla::SharedPtr<carla::client::Waypoint> waypoint;
    std::vector<carla::geom::Location> corners;
    double ego_entry_index;
    double ego_front_nearest_wp_idx;         // 目标车从当前位置到重叠区域的路径距离
    double tar_entry_index;         // 自车从当前位置到重叠区域的路径距离
    double tar_front_nearest_wp_idx;         // 目标车从当前位置到重叠区域的路径距离
    double vel_;  
    double t_ini; 
    double t_end; 
    double s_ini; 
    double s_end; 
    double width_; 
    double length_;
    InterferingVehicle() = default;
};

struct TFObstacle{
    int32_t id;
    carla::geom::Transform transform; 
    double vel_;  
    double t_ini; 
    double t_end; 
    double s_ini; 
    double s_end; 
    double width_; 
    double length_;
};

enum class SceneType {
  NORMAL,
  INTERSECTION,
  CROSSWALK,
  MERGE
};

enum class DriverState {
    INIT = 0,                      ///< 初始化状态
    CRUISE = 1,                   ///< 自由巡航状态
    RUNING_IN_CONTROL_AREA  = 2,  ///< 在控制区域内行驶
    RUNING_IN_DECISION_AREA = 3,  ///< 在决策区域内行驶
    WAITING_IN_DECISION_AREA = 4, ///< 在决策区域内等待
    CROSSING_INTERSECTION = 5,    ///< 在路口穿行
    EMERGENCY_STOP = 6,           ///< 紧急停车
    DESTROYED = 7                 ///< 已销毁
};

struct DataPool {
  // 自车状态
  double cruise_speed = 0.0;
  double ego_speed;
  geometry_msgs::Pose ego_pose;
  carla::geom::Transform ego_transform;
  std::shared_ptr<carla::client::Waypoint> ego_waypoint;

  // 感知结果
  std::optional<FollowableVehicle> follow_target_;
  std::vector<InterferingVehicle> interfering_vehicles_; 

  // 路口/信号灯状态
  bool is_right_side_vehicle = false;
  bool is_crosswalk_present = false;
  bool is_traffic_blocked = false;
  bool is_following_vehicle = false;
  bool is_front_vehicle = false;
  bool is_left_turn_safe = true;
  bool is_traffic_light_affecting = false;

  // 距离与目标信息
  double distance_to_target = std::numeric_limits<double>::infinity();
  double target_speed = std::numeric_limits<double>::infinity();
  int target_id = -1;

  // 当前场景类型
  SceneType scene_type = SceneType::NORMAL;

  // 路径规划
  std::vector<carla::geom::Transform> global_path;
  std::vector<carla::geom::Transform> local_path;

  // 时间戳（用于同步）
  ros::Time last_updated;

  // 线程安全访问（可选）
  std::mutex mutex;
};


#endif //JERK_SPEED_PLANNING_DATA_POOL_H
