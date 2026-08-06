
#include "local_planner/local_planner.h"
#include <filesystem>  // C++17
#include <tf/tf.h>
using namespace common;

namespace local_planner {

     LocalPlanner::LocalPlanner(ros::NodeHandle& nh, const driver_models_types::VehicleConfig &vehicle_config)
        :target_speed_(0.0), 
         min_distance_percentage_(0.9),
         min_distance_reach_goal(5.0), 
         buffer_size_(5),
         max_queue_size_(10000), 
         role_name_(vehicle_config.role_name),
         carla_id_(vehicle_config.carla_id), 
         goal_reached_flag_(false), 
         odom_received_flag_(false),
         vehicle_config_(vehicle_config){

        nh.param("/driver_model_manager_node/Kp_longitudinal", lon_KP_, 0.206);
        nh.param("/driver_model_manager_node/Ki_longitudinal", lon_KI_, 0.0206);
        nh.param("/driver_model_manager_node/Kd_longitudinal", lon_KD_, 0.515);
        nh.param("/driver_model_manager_node/Kp_lateral", lat_KP_, 0.9);
        nh.param("/driver_model_manager_node/Ki_lateral", lat_KI_, 0.0);
        nh.param("/driver_model_manager_node/Kd_lateral", lat_KD_, 0.0);

        controller_ = PIDController(lon_KP_, lon_KI_, lon_KD_, lat_KP_, lat_KI_, lat_KD_);
        odometry_sub_ = nh.subscribe(
            "/carla/" + role_name_ + "/odometry", 1, &LocalPlanner::odometry_call_back, this);
        goal_status_pub_ = nh.advertise<driver_models_types::VehicleGoalStatus>("/carla/goal_status", 10, false);
        control_cmd_pub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name_ + "/vehicle_control_cmd", 10);

    }

    void LocalPlanner::odometry_call_back(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_ = msg->pose.pose;
        current_speed_ = std::sqrt(
            std::pow(msg->twist.twist.linear.x, 2) +
            std::pow(msg->twist.twist.linear.y, 2) +
            std::pow(msg->twist.twist.linear.z, 2)); // Convert to km/h
        odom_ = *msg;
        odom_received_flag_ = true;
    }

    void LocalPlanner::set_global_plan(const std::shared_ptr<driver_models_types::PathWithOptions>& path) {
        if (path){
            std::lock_guard<std::mutex> lock(data_mutex_);
            waypoints_queue_.clear();
            waypoint_buffer_.clear();
            processed_waypoints_.clear();
            for (const auto& waypoint : path->waypoints) {
                RoadOption road_option = static_cast<RoadOption>(waypoint.road_option);
                waypoints_queue_.emplace_back(waypoint.pose, road_option);
            }
        } else {
            ROS_ERROR("Received null path.");
        }
    }

    void LocalPlanner::set_target_speed(const double& speed){
       
        target_speed_ = speed;
    }

    void LocalPlanner::set_target_pose(const geometry_msgs::Pose& pose){
        target_pose_ = pose;
    }

    void LocalPlanner::set_reached_goal(const bool& reached_goal){
       
        goal_reached_flag_ = reached_goal;
    }
    void LocalPlanner::set_local_trajectory(const planner::PublishableTrajectory& traj) {
        pb_planned_trajectory_ = traj;
    }

    bool LocalPlanner::is_reached_goal(const geometry_msgs::Pose& goal_pose, double threshold) {
        double distance = distance_vehicle(goal_pose.position, current_pose_.position);
        bool reached = (distance <= threshold);

        if (reached && !goal_reached_once_) {
            goal_reached_once_ = true;
            last_goal_time_ = ros::Time::now();
            ROS_INFO("%s: First time reaching goal! Distance: %.2f, Threshold: %.2f", 
                    role_name_.c_str(), distance, threshold);
            return true;
        }

        if (goal_reached_once_ && (ros::Time::now() - last_goal_time_).toSec() > 1.0) {
            goal_reached_once_ = false;
        }

        return false;
    }

    bool LocalPlanner::is_waypoint_queue_empty(){

        return waypoints_queue_.empty() && waypoint_buffer_.empty();
    }

static void LogFrame(const geometry_msgs::Pose& current_pose,
                     const geometry_msgs::Pose& target_pose,
                     const TrajectoryPoint& target_point,
                     const PIDDebugInfoLO& lon_dbg,   // 纵向
                     const PIDDebugInfoLA& lat_dbg,   // 横向
                     const carla_msgs::CarlaEgoVehicleControl& control_msg,
                     const PublishableTrajectory& pb_planned_trajectory,
                     double current_speed,
                     double veh_rel_time,
                     const std::shared_ptr<std::deque<std::pair<geometry_msgs::Pose, RoadOption>>>& plan,
                     const std::pair<std::vector<double>, std::vector<double>>& reference_line,
                     bool split_files) {
    // === 静态变量：文件流、计数器、文件编号 ===
        static std::ofstream ofs;
        static size_t frame_count = 0;
        static size_t file_index = 0;

        // === 文件夹路径 ===
        const std::string log_dir = "/home/bob/文档/备份/demo05/src/test/txt/frame_log";

        // === 1. 检查目录是否存在，不存在则创建 ===
        try {
            if (!std::filesystem::exists(log_dir)) {
                std::filesystem::create_directories(log_dir);
                ROS_WARN("Created log directory: %s", log_dir.c_str());
            }
        } catch (const std::filesystem::filesystem_error& e) {
            ROS_ERROR("Failed to ensure log directory: %s", e.what());
            return;
        }

        // === 2. 若开启分文件模式 ===
        if (split_files && (!ofs.is_open() || frame_count >= 300)) {
            if (ofs.is_open()) ofs.close();

            std::ostringstream fname;
            fname << log_dir << "/frame_log_" << file_index++ << ".txt";

            ofs.open(fname.str(), std::ios::out | std::ios::trunc);
            if (!ofs.is_open()) {
                ROS_ERROR("Failed to open log file: %s", fname.str().c_str());
                return;
            }

            ofs << std::fixed << std::setprecision(6);
            frame_count = 0;
            ROS_INFO("Switch log file -> %s", fname.str().c_str());
        }

        // === 3. 若不分文件模式，打开单一日志文件 ===
        if (!split_files && !ofs.is_open()) {
            std::string file_path = log_dir + "/frame_log.txt";
            ofs.open(file_path, std::ios::out | std::ios::trunc);
            if (!ofs.is_open()) {
                ROS_ERROR("Failed to open log file: %s", file_path.c_str());
                return;
            }

            ofs << std::fixed << std::setprecision(6);
            frame_count = 0;
            ROS_INFO("Logging to single file -> %s", file_path.c_str());
        }

        // ===== 时间戳 =====
        double now_ms = ros::Time::now().toSec() * 1000.0;
        ofs << "time(ms): " << now_ms << "\n";

        // ===== Ego Pose =====
        double ego_yaw = tf::getYaw(current_pose.orientation);
        ofs << "# Ego Pose\n";
        ofs << "position " << current_pose.position.x << " "
            << current_pose.position.y << " "
            << current_pose.position.z << "\n";
        ofs << "orientation " << current_pose.orientation.x << " "
            << current_pose.orientation.y << " "
            << current_pose.orientation.z << " "
            << current_pose.orientation.w << "\n";
        ofs << "yaw " << ego_yaw << "\n";

        // ===== Target Pose =====
        double target_yaw = tf::getYaw(target_pose.orientation);
        ofs << "# Target Pose\n";
        ofs << "position " << target_pose.position.x << " "
            << target_pose.position.y << " "
            << target_pose.position.z << "\n";
        ofs << "orientation " << target_pose.orientation.x << " "
            << target_pose.orientation.y << " "
            << target_pose.orientation.z << " "
            << target_pose.orientation.w << "\n";
        ofs << "yaw " << target_yaw << "\n";
        ofs << "v " << target_point.v << "\n";
        ofs << "t " << veh_rel_time << "\n";

        // ===== 速度信息 =====
        ofs << "# Speed Info\n";
        ofs << "ego_speed "    << current_speed   << "\n";
        ofs << "target_speed " << target_point.v  << "\n";

        // ===== Longitudinal PID Debug Info =====
        ofs << "# PID Debug Info\n";
        ofs << "error "            << lon_dbg.error           << "\n";
        ofs << "error_integral "   << lon_dbg.error_integral  << "\n";
        ofs << "error_derivative " << lon_dbg.error_derivative<< "\n";
        ofs << "p_term "           << lon_dbg.p_term          << "\n";
        ofs << "i_term "           << lon_dbg.i_term          << "\n";
        ofs << "d_term "           << lon_dbg.d_term          << "\n";
        ofs << "pid_output "       << lon_dbg.output          << "\n";

        // ===== Lateral Debug Info =====
        ofs << "# Lateral Debug Info\n";
        ofs << "heading_error " << lat_dbg.heading_error << "\n";
        ofs << "e_y "           << lat_dbg.e_y           << "\n";
        ofs << "cross_term "    << lat_dbg.cross_term    << "\n";
        ofs << "stanley_term "  << lat_dbg.stanley_term  << "\n";
        ofs << "delta_raw "     << lat_dbg.delta_raw     << "\n";
        ofs << "lat_output "    << lat_dbg.output        << "\n";

        // ===== Control Command =====
        ofs << "# Control Command\n";
        ofs << "throttle " << control_msg.throttle << "\n";
        ofs << "brake "    << control_msg.brake    << "\n";
        ofs << "steer "    << control_msg.steer    << "\n";

        // ===== 当前规划轨迹 =====
        ofs << "# Planned Trajectory\n";
        for (size_t i = 0; i < pb_planned_trajectory.NumOfPoints(); ++i) {
            const auto &p = pb_planned_trajectory.TrajectoryPointAt(i);
            ofs << p.path_point().x << " "
                << p.path_point().y << " "
                << p.path_point().theta << " "
                << p.v << " "
                << p.a << " "
                << p.relative_time << " "
                << p.path_point().s << " "
                << p.d_d << " "
                << p.s_d << " "
                << "\n";
        }

        if (plan) {
            ofs << "# Global Plan (size=" << plan->size() << ")\n";
            for (const auto& wp : *plan) {
                const auto& pose = wp.first;
                int roadopt = int(wp.second);
                ofs << pose.position.x << " "
                    << pose.position.y << " "
                    << pose.position.z << " "
                    << roadopt << "\n";
            }
        } else {
            ofs << "# Global Plan: EMPTY\n";
        }

        ofs << "# Reference Line (size=" << reference_line.first.size() << ")\n";
        for (size_t i = 0; i < reference_line.first.size(); ++i) {
            double x = reference_line.first[i];
            double y = reference_line.second[i];
            ofs << x << " " << y << "\n";
        }

        ofs << "-----------------------------------\n";
        ofs.flush();

        // 递增帧计数
        frame_count++;
    }

    carla_msgs::CarlaEgoVehicleControl LocalPlanner::run_step(bool compute_control) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 默认控制指令（全零），用于提前返回
        carla_msgs::CarlaEgoVehicleControl control_msg;
        control_msg.throttle   = 0.0;
        control_msg.brake      = 1.0;
        control_msg.steer      = 0.0;
        control_msg.hand_brake = false;
        control_msg.reverse    = false;

        // 检查车辆是否达到终点
        if (goal_reached_flag_ == true) {
            return control_msg;
        }
        if (is_reached_goal(vehicle_config_.goal_point.pose, min_distance_reach_goal)) {
            goal_reached_flag_ = true;
            driver_models_types::VehicleGoalStatus msg;
            msg.vehicle_id   = role_name_;
            msg.carla_id     = vehicle_config_.carla_id;
            msg.reached_goal = true;
            goal_status_pub_.publish(msg);
            return control_msg;
        }
        if (odom_received_flag_ == false) {
            // ROS_WARN("%s: odom not received!", role_name_.c_str());
            emergency_stop();
            return control_msg;
        }

        if (is_waypoint_queue_empty()) {
            ROS_DEBUG("%s: The waypoints_queue_ is empty, Waiting for a route...", role_name_.c_str());
            emergency_stop();
            return control_msg;
        }

        // Buffer waypoints if needed
        while (waypoint_buffer_.size() < buffer_size_ && !waypoints_queue_.empty()) {
            waypoint_buffer_.emplace_back(waypoints_queue_.front());
            processed_waypoints_.emplace_back(waypoints_queue_.front());
            waypoints_queue_.pop_front();
        }

        if (!pb_planned_trajectory_.empty()) {
            double now = ros::Time::now().toSec();
            TrajectoryPoint target_point;

            const double veh_rel_time = now - pb_planned_trajectory_.header_time() + 0.22;
            // const double veh_rel_time = now - pb_planned_trajectory_.header_time() + 0.02;
            size_t time_matched_index =
                pb_planned_trajectory_.QueryLowerBoundPoint(veh_rel_time);

            planner::Vec2d current_xy(current_pose_.position.x, current_pose_.position.y);
            double yaw = tf::getYaw(current_pose_.orientation);
            size_t position_forward_index =
                pb_planned_trajectory_.QueryForwardNearestPoint(current_xy, yaw);

            // 时间匹配点
            TrajectoryPoint time_point =
                pb_planned_trajectory_.TrajectoryPointAt(
                    std::min(time_matched_index, pb_planned_trajectory_.NumOfPoints() - 1));
            // ========== 前方性判断 ==========
            size_t matched_index = time_matched_index;
            {
                // 车辆朝向向量
                planner::Vec2d heading_vec(std::cos(yaw), std::sin(yaw));
                planner::Vec2d time_point_xy(time_point.path_point().x,
                                            time_point.path_point().y);

                // 向量点积判断是否在前方
                planner::Vec2d vec_to_time_point = time_point_xy - current_xy;
                double dot = heading_vec.InnerProd(vec_to_time_point);

                if (dot < 0) {
                    // 时间匹配点在后方 → 改用位置匹配点
                    ROS_ERROR_THROTTLE(1.0, "[%s] Time-matched point at idx=%lu is behind vehicle, "
                            "switching to position-matched idx=%lu",
                            role_name_.c_str(),
                            time_matched_index,
                            position_forward_index);
                    matched_index = position_forward_index;
                }
            }
            // ========== 目标点 ==========
            target_point = pb_planned_trajectory_.TrajectoryPointAt(
                std::min(matched_index, pb_planned_trajectory_.NumOfPoints() - 1));
            geometry_msgs::Pose target_pose;
            target_pose.position.x = target_point.path_point().x;
            target_pose.position.y = target_point.path_point().y;
            target_pose.orientation = tf::createQuaternionMsgFromYaw(target_point.path_point().theta);

            target_speed_ = target_point.v;

            if (vehicle_config_.role_name == "hero0") {
                ROS_INFO_THROTTLE(
                    1.0,
                    "[%s] speed: current=%.2f m/s, target=%.2f m/s",
                    role_name_.c_str(),
                    current_speed_,
                    target_speed_
                );
            }

            control_msg = controller_.run_step(
                target_speed_, current_speed_, current_pose_, target_pose);

            if(role_name_ == "hero16" && 0){
                auto plan_ptr = get_plan();  // 加锁复制当前全局路径
                auto [lon_dbg, lat_dbg] = controller_.get_debug_info();
                LogFrame(current_pose_, target_pose, target_point, lon_dbg, lat_dbg,
                    control_msg, pb_planned_trajectory_, current_speed_,veh_rel_time,plan_ptr,reference_line_,true);
            }
        }else{
            if(compute_control){
                control_msg = controller_.run_step(target_speed_, current_speed_, current_pose_, target_pose_);
            }
        }

        // Purge obsolete waypoints
        int max_index = -1;
        double min_distance_reach_wp = 5.0;
        for (size_t i = 0; i < waypoint_buffer_.size(); ++i) {
            const auto& route_point = waypoint_buffer_[i].first;
            double distance = common::distance_vehicle(route_point.position, current_pose_.position);
            if (distance < min_distance_reach_wp) {
                max_index = static_cast<int>(i);
            }
        }
        if (max_index >= 0) {
            for (int i = 0; i <= max_index; ++i) {
                waypoint_buffer_.pop_front();
            }
        }

        int max_processed_index = -1;
        double min_distance_reach_wp_processed = 1.0;
        for (size_t i = 0; i < processed_waypoints_.size(); ++i) {
            const auto& route_point = processed_waypoints_[i].first;
            double distance = common::distance_vehicle(route_point.position, current_pose_.position);
            if (distance < min_distance_reach_wp_processed) {
                max_processed_index = static_cast<int>(i);
            }
        }
        if (max_processed_index >= 0) {
            for (int i = 0; i <= max_processed_index; ++i) {
                last_deleted_waypoints_.emplace_front(processed_waypoints_.front());
                if (last_deleted_waypoints_.size() > 10) {
                    last_deleted_waypoints_.pop_back();
                }
                processed_waypoints_.pop_front();
            }
        }

        // 6. 执行控制
        control_cmd_pub_.publish(control_msg);
       
        // 正常路径：返回控制器计算的控制指令
        return control_msg;
    }


    void LocalPlanner::emergency_stop() {
        carla_msgs::CarlaEgoVehicleControl control_msg;
        control_msg.steer = 0.0;
        control_msg.throttle = 0.0;
        control_msg.brake = 1.0;

    }

    visualization_msgs::Marker LocalPlanner::pose_to_markerMsg(const geometry_msgs::Pose& pose) {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.header.frame_id = "map";
        marker.pose = pose;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        return marker;
    }

    std::shared_ptr<std::deque<std::pair<geometry_msgs::Pose, RoadOption>>> LocalPlanner::get_plan() const {
        // 如果两个队列都为空，返回空指针
        if (processed_waypoints_.empty() && waypoints_queue_.empty()) {
            return nullptr;
        }

        // 创建一个新的队列存储拼接结果
        auto combined_plan = std::make_shared<std::deque<std::pair<geometry_msgs::Pose, RoadOption>>>();

        // 将 waypoint_buffer_ 内容插入
        combined_plan->insert(combined_plan->end(), processed_waypoints_.begin(), processed_waypoints_.end());

        // 将 waypoints_queue_ 内容插入
        combined_plan->insert(combined_plan->end(), waypoints_queue_.begin(), waypoints_queue_.end());

        return combined_plan;
    }

    std::shared_ptr<std::pair<geometry_msgs::Pose, RoadOption>> 
    LocalPlanner::get_incoming_waypoint_and_direction(int steps) {
        // 计算 processed 和 queue 的总可用步数
        int processed_size = processed_waypoints_.size();
        int queue_size = waypoints_queue_.size();
        // 1. 处理负索引：从 last_deleted_waypoints_ 取倒数第 `|steps|` 个点
        if (steps < 0) {
            int index = std::abs(1 + steps);  // 计算后方索引
            if (index >= 0 && index < last_deleted_waypoints_.size()) {
                return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(last_deleted_waypoints_[index]);
            } else if (!last_deleted_waypoints_.empty()) {
                return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(last_deleted_waypoints_.back());  // 返回最后一个点
            } else if (!processed_waypoints_.empty()) {
                return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(processed_waypoints_.front());
            } else {
                return nullptr;  // 没有任何点可返回
            }
        }
        // 1. 优先从 waypoint_buffer_ 获取
        if (steps < processed_size) {
            return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(processed_waypoints_[steps]);
        }

        // 2. 如果 processed_size 不足，再从 waypoints_queue_ 获取
        int remaining_steps = steps - processed_size;
        if (remaining_steps < queue_size) {
            return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(waypoints_queue_[remaining_steps]);
        }

        // 3. 如果 queue 也不够，返回最后一个元素
        if (!waypoints_queue_.empty()) {
            return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(waypoints_queue_.back());
        } else if (!processed_waypoints_.empty()) {
            return std::make_shared<std::pair<geometry_msgs::Pose, RoadOption>>(processed_waypoints_.back());
        }
        // 4. 如果两者都为空，返回空指针
        return nullptr;
    }

    geometry_msgs::Pose LocalPlanner::get_current_pose_() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return current_pose_; 
    }

    double LocalPlanner::get_current_speed_() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return current_speed_; }

    bool LocalPlanner::get_reached_goal_flag() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return goal_reached_flag_; }
        
    bool LocalPlanner::get_odometry_received_flag() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return odom_received_flag_; }

    nav_msgs::Odometry LocalPlanner::get_odom_() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return odom_; }

    void local_planner::LocalPlanner::set_goal_point(const geometry_msgs::PoseStamped& goal_point) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        vehicle_config_.goal_point = goal_point;
        ROS_INFO("[%s] 更新 vehicle_config_.goal_point -> (%.2f, %.2f, %.2f)",
                role_name_.c_str(),
                goal_point.pose.position.x,
                goal_point.pose.position.y,
                goal_point.pose.position.z);
    }
    void LocalPlanner::set_reference_line(const std::pair<std::vector<double>, std::vector<double>>& ref_line)
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        reference_line_ = ref_line;
    }
} // namespace carla_ad_agent