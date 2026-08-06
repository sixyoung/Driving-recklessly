#ifndef MANUAL_DRIVER_H
#define MANUAL_DRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <driver_models/base_driver.h>
#include <road_side_system/TrafficLightPhase.h>
#include <road_side_system/TrafficLightPhaseArray.h>

#include "driver_models/random_behavior_injector.h"
#include "driver_models/random_bt_yaml_loader.h"
#include <speed_planner/jeker_speed_planner.h>

#include <limits>

#include <fstream>
#include <chrono>

namespace driver_model {
class ManualDriver : public BaseDriver {
public:
    ManualDriver(ros::NodeHandle& nh, 
                const driver_models_types::VehicleConfig &vehicle_config, 
                const carla::client::World& world);
    ~ManualDriver();

    struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
    };

    struct obstacle {
        int id;     // 车辆ID
        carla::geom::Transform transform;  // 当前姿态
        std::vector<carla::geom::Location> corners;  // 边界点
        Polygon polygon;  // 多边形边界
        double speed;  // 速度（单位：m/s）
        carla::geom::Vector3D velocity_vec; // 三维速度向量
        double length;
        double width;
        std::vector<carla::geom::Transform> predicted_path;  // 原始预测路径
        std::vector<carla::geom::Transform> interp_path;     // 插值后路径
        carla::SharedPtr<carla::client::Waypoint> waypoint;  // 当前所处 Waypoint
        std::string turn_signal;  // “left” / “right” / “straight” / “unknown”
        std::vector<Point> overlap_points;  // 重叠区域点集
        std::vector<double> interp_s_list;
    };

    struct ObstacleIDM {
        double s_ini = 0.0;
        double s_end = 0.0;
        double t_ini = 0.0;
        double t_end = 0.0;
        double velocity = 0.0;
        std::string name;  // 🔹 添加名称字段

        ObstacleIDM() = default;

        ObstacleIDM(double t0, double t1,
                    double s0, double s1,
                    double vel,
                    const std::string& obs_name = "")
            : s_ini(s0), s_end(s1), t_ini(t0), t_end(t1),
            velocity(vel), name(obs_name) {}
    };

    struct ClosestObstacleInfo {
        double s_ini = 1000;  
        int type = -1;
        ObstacleIDM obs;  // 最近障碍物的IDM信息   
        void update(ObstacleIDM obs_new, int new_type) {
            if (obs_new.s_ini < s_ini) {
                s_ini = obs_new.s_ini;
                type = new_type;
                obs = obs_new;
            }
        }
    };
    /**
     * @brief 驾驶员状态枚举
     */
    enum class DriverState {
        INIT = 0,                      ///< 初始化状态
        CRUISE = 1,                   ///< 自由巡航状态
        RUNING_OUT_CONTROL_AREA = 2,  ///< 在控制区域外行驶
        RUNING_IN_CONTROL_AREA  = 3,  ///< 在控制区域内行驶
        RUNING_IN_DECISION_AREA = 4,  ///< 在决策区域内行驶
        WAITING_IN_DECISION_AREA = 5, ///< 在决策区域内等待
        CROSSING_INTERSECTION = 6,    ///< 在路口穿行
        EMERGENCY_STOP = 7,           ///< 紧急停车
        DESTROYED = 8                 ///< 已销毁
    };

    void Initialize() override;
    void PerceiveEnvironment() override;
    void MakeDecision() override;
    void destroy() override;
    void reset();
    std::shared_ptr<local_planner::LocalPlanner> get_local_planner() override;
    int get_vehicle_id() const override;
    /**
     * @brief 检查是否到达目标
     * @return 是否到达目标
     */
    bool has_reached_goal() const override {
        return local_planner_->get_reached_goal_flag();
    }

    /**
     * @brief 运行一个仿真步长
     */
    void run_step() override;

private:
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    // Jerk限速规划器
    speed_planner::DataPool data_pool_;
    speed_planner::JerkSpeedPlanner jerk_speed_planner_;

    // 场景相关
    std::string scene_type_;
    std::unordered_map<std::pair<int, int>, carla::geom::Transform, PairHash> road_lane_to_entry_stop_line_;
    std::unordered_map<std::pair<int, int>, carla::geom::Transform, PairHash> road_lane_to_exit_reference_;
    std::unordered_map<int, int> road_to_traffic_light_;

    // 车辆相关
    DriverState last_non_emergency_state_ = DriverState::INIT;
    DriverState driver_state_ = DriverState::INIT;

    geometry_msgs::Pose ego_pose;
    carla::geom::Transform ego_transform;
    carla::geom::Vector3D ego_forward;
    carla::SharedPtr<carla::client::Waypoint> ego_wpt;
    std::vector<carla::geom::Location> ego_corners;
    Polygon ego_polygon;
    std::vector<carla::geom::Transform> ego_path;

    std::vector<double> s_list;
    std::vector<double> v_limit_list;
    std::vector<carla::geom::Transform> interp_path;

    Polygon ego_path_poly;

    double current_speed_;
    double current_acc_;

    bool is_destroyed_;  

    // 感知信息
    std::mutex objects_mutex_;
    double detection_range;
    std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>> last_valid_ego_wpts_;  ///< 自车上次有效的目标路径点
    std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>> last_valid_tar_wpts_;  ///< 目标车辆上次有效的目标路径点
    std::vector<obstacle> inter_obs_; 
    std::vector<obstacle> emergency_obs_; 
    std::vector<obstacle> tar_obs_;  
    std::vector<ObstacleIDM> obstacles_;
    std::optional<ObstacleIDM> traffic_light_obs;
    ClosestObstacleInfo closest_obstacle_info_;

    // 交通灯信息
    std::mutex traffic_light_mutex_;
    std::unordered_map<int, road_side_system::TrafficLightPhase> traffic_light_map_;

    // 周围物体信息
    std::map<uint32_t, derived_object_msgs::Object> carla_objects_;  ///< 物体信息

    // ROS订阅者
    ros::Subscriber objects_sub;                                ///< 物体信息订阅
    ros::Subscriber traffic_lights_phases_sub;                  ///< 交通灯相位信息订阅

    // 场景状态标志
    std::unordered_map<int32_t, int> target_stationary_counter_;  ///< 目标静止计数器


    // 交通灯相关
    carla::SharedPtr<carla::client::TrafficLight> affecting_traffic_light;  ///< 影响自车的交通灯
    carla::rpc::TrafficLightState last_light_state;  ///< 上次交通灯状态

    bool lane_change_triggered_ = false;  // 是否已经执行过换道
    // 新增函数声明
    void update_infomation(carla::SharedPtr<carla::client::Vehicle> vehicle);
    void uodate_vehicle_info();
    void update_path_waypoint();
    void update_intention();
    void update_vehicle_light(bool is_active, carla::SharedPtr<carla::client::Vehicle> vehicle);
    std::shared_ptr<std::pair<geometry_msgs::Pose, local_planner::RoadOption>> get_road_option();

    // 状态处理与切换
    void state_transition_tl(); 
    void state_transition_untl(); 
    void handle_state();
    bool has_valid_position() const;
    double calcu_distance_to_entry_line(const carla::SharedPtr<carla::client::Waypoint>& wp,
                                const carla::geom::Location& ego_loc) const;
                                 
    bool yield_in_decision_area();
    bool has_emergency_conditon(const std::vector<obstacle>& inter_obstacles, double emergency_speed_threshold);
    void handle_emergency_stop_state(const std::vector<obstacle>& emergency_obs_); 
    bool is_crossed_entry_line( const carla::SharedPtr<carla::client::Waypoint>& wp, 
                            const carla::geom::Transform& transform,
                            const double extent_x,
                            const double extent_y,
                            double* exceeded_distance);
    bool is_crossed_exit_line(const carla::SharedPtr<carla::client::Waypoint>& wp, 
                        const carla::geom::Transform& transform,
                        const double extent_x,
                        const double extent_y,
                        double* exceeded_distance);
    
    // 场景处理
    void handle_intersection();  ///< 处理路口
    void handle_trafficlight();  ///< 处理交通灯
    void handle_merge();         ///< 处理并道

    // 回调函数
    void objects_call_back(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    void trafficlight_phase_call_back(const road_side_system::TrafficLightPhaseArray::ConstPtr& msg);

    //辅助函数
    std::tuple<bool, BaseSolver::OutputInfo> EstimateTravelTime(
        const std::vector<speed_planner::Obstacle>& obstacles,
        const std::vector<double>& s_list,
        const std::vector<double>& v_limit_list,
        bool should_output_results);
    /**
     * @brief 检查目标框是否与路径相交，并确定相交类型
     * @param path_poly 路径多边形
     * @param target_transform 目标框的变换矩阵
     * @param target_extent_x 目标框x方向尺寸
     * @param target_extent_y 目标框y方向尺寸
     */
    bool is_box_intersect_path(
        const Polygon& path_poly,
        const carla::geom::Transform& target_transform,
        const double target_extent_x,
        const double target_extent_y);


    /**
     * @brief 获取目标未来路径
     * @param tar_wp 目标waypoint
     * @param tar_id 目标ID
     * @param direction 方向
     * @param spacing 间距
     * @param num_points 点数
     * @return 未来路径
     */
    std::vector<carla::geom::Transform> get_target_path(
        const carla::SharedPtr<carla::client::Waypoint>& tar_wp,
        const int32_t& target_id,
        const std::string& direction,
        double spacing,
        int num_points); 

    /**
     * @brief 获取自车路径
     * @param num_points 点数
     * @return 路径
     */
    std::vector<carla::geom::Transform> get_ego_path(int num_points);

    void draw_ego_box();

    carla::SharedPtr<carla::client::Waypoint> find_match_waypoint(
        int id, const carla::geom::Transform& transform, double max_dot, double search_resolution,
        std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>>& last_valid_map);

    void get_forward_path(
        const carla::geom::Transform& transform,
        double extent_x,
        const std::vector<carla::geom::Transform>& path,
        std::vector<carla::geom::Transform>& trimmed_path,
        std::vector<double>& raw_s_list,
        const Polygon vehicle_polygon);

    void interpolate_path(
        const std::vector<carla::geom::Transform>& path,
        const std::vector<double>& s_list,
        double spacing,
        double v_limit,
        std::vector<carla::geom::Transform>& interp_path,
        std::vector<double>& s_interp,
        std::vector<double>& v_interp);

    std::optional<std::pair<double, size_t>> project_point_to_path(
        const carla::geom::Location& point,
        const std::vector<carla::geom::Transform>& interp_path,
        const std::vector<double>& s_list,
        double eps_dist_sq);

    std::string DriverStateToString(DriverState state);
    double PlanSpeed();
    void lane_change(const carla::geom::Location& trigger_loc);

};

} // namespace driver_models

#endif // MANUAL_DRIVER_H 