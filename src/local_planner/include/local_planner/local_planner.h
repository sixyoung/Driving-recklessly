#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <deque>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <local_planner/pid_controller.h>

#include <visualization_msgs/Marker.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <driver_models_types/VehicleConfig.h>
#include <driver_models_types/PathWithOptions.h>
#include <driver_models_types/VehicleGoalStatus.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <publishable_trajectory.h>

namespace local_planner {
enum class RoadOption {
    VOID = -1,             // 无效选项
    LEFT = 1,              // 左转
    RIGHT = 2,             // 右转
    STRAIGHT = 3,          // 直行
    LANEFOLLOW = 4,        // 跟随车道
    CHANGELANELEFT = 5,    // 左换道
    CHANGELANERIGHT = 6    // 右换道
};

class LocalPlanner {
public:
    LocalPlanner(ros::NodeHandle& nh, const driver_models_types::VehicleConfig &vehicle_config);
    ~LocalPlanner() = default; 
    carla_msgs::CarlaEgoVehicleControl run_step(bool compute_control = true);
    std::shared_ptr<std::deque<std::pair<geometry_msgs::Pose, RoadOption>>> get_plan() const;
    std::shared_ptr<std::pair<geometry_msgs::Pose, RoadOption>> get_incoming_waypoint_and_direction(int steps);

    geometry_msgs::Pose get_current_pose_() const;
    nav_msgs::Odometry get_odom_() const;

    double get_current_speed_() const;
    bool get_reached_goal_flag() const;
    bool get_odometry_received_flag() const;

    void set_target_speed(const double& speed);
    void set_target_pose(const geometry_msgs::Pose& pose);
    void set_reached_goal(const bool& reached_goal);
    void set_global_plan(const std::shared_ptr<driver_models_types::PathWithOptions>& path);
    void set_goal_point(const geometry_msgs::PoseStamped& goal_point);

public:
    void set_local_trajectory(const planner::PublishableTrajectory& traj);
    void set_reference_line(const std::pair<std::vector<double>, std::vector<double>>& ref_line);
private:
    planner::PublishableTrajectory pb_planned_trajectory_;

private:
    // Callbacks
    void odometry_call_back(const nav_msgs::Odometry::ConstPtr& msg);
    bool is_reached_goal(const geometry_msgs::Pose& target_pose, double min_distance);
    bool is_waypoint_queue_empty();
    void match_to_global_path();
    // Utility methods
    void emergency_stop();
    visualization_msgs::Marker pose_to_markerMsg(const geometry_msgs::Pose& pose);
    
    // Parameters
    mutable std::mutex data_mutex_;
    std::string role_name_;
    u_int32_t carla_id_;
    double target_speed_;
    double min_distance_percentage_;
    double min_distance_reach_goal;
    bool goal_reached_flag_;
    bool odom_received_flag_;           // 标志位，检查是否已接收Odometry消息
    // State variables
    double current_speed_;
    geometry_msgs::Pose current_pose_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Pose target_pose_;
    std::deque<std::pair<geometry_msgs::Pose, RoadOption>> waypoints_queue_;
    std::deque<std::pair<geometry_msgs::Pose, RoadOption>> waypoint_buffer_;
    std::deque<std::pair<geometry_msgs::Pose, RoadOption>> processed_waypoints_;
    std::deque<std::pair<geometry_msgs::Pose, RoadOption>> last_deleted_waypoints_; // 存储最近删除的5个点

    driver_models_types::VehicleConfig vehicle_config_;
    size_t buffer_size_;
    size_t max_queue_size_;

    // Controller
    double lon_KP_, lon_KI_, lon_KD_, lat_KP_, lat_KI_, lat_KD_;
    PIDController controller_;


    // 消息过滤器和同步器
    ros::Subscriber odometry_sub_;

    ros::Time last_time_;
    ros::Publisher target_pose_pub_;
    ros::Publisher goal_status_pub_;
    ros::Publisher control_cmd_pub_;

    std::shared_ptr<std::ofstream> log_file_;
    ros::Time start_time_;
    
private:
    ros::Time last_goal_time_;
    bool goal_reached_once_ = false;
    std::shared_ptr<driver_models_types::PathWithOptions> global_path_;
    std::pair<std::vector<double>, std::vector<double>> reference_line_;
};
}

#endif 