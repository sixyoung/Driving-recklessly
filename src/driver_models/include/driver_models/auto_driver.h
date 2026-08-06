#ifndef AUTO_DRIVER_H
#define AUTO_DRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <driver_models/base_driver.h>
#include <road_side_system/TrafficLightPhase.h>
#include <road_side_system/TrafficLightPhaseArray.h>
#include <road_side_system_type/PoseWithTimeWindowArray.h>
#include "em_planner/em_planner.h"
#include <fstream>

namespace driver_model {
class AutoDriver : public BaseDriver {
public:
    AutoDriver(ros::NodeHandle& nh,  const driver_models_types::VehicleConfig &vehicle_config, const carla::client::World& world);
    ~AutoDriver();

    struct PairHash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

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

    template <typename Func>
    long measure_ms(const std::string& tag, Func&& f, const std::string& role_name) {
        using Clock = std::chrono::steady_clock;
        auto start = Clock::now();

        f();  // 执行函数

        auto end = Clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        ROS_INFO("[%s] %s 耗时: %ld ms", role_name.c_str(), tag.c_str(), duration_ms);
        return duration_ms;
    }

public:
    void Initialize() override;
    void PerceiveEnvironment() override;
    void MakeDecision() override;
    void destroy() override;
    void reset();
    bool has_reached_goal() const override {
        return local_planner_->get_reached_goal_flag();
    }
    
    void run_step() override;
    std::shared_ptr<local_planner::LocalPlanner> get_local_planner() override;
    int get_vehicle_id() const override; 
private:
    struct ScheduleCommand {
        int id;
        geometry_msgs::Pose pose;
        std::vector<geometry_msgs::Pose>ConflictpPoint_corners;
        ros::Time time_start;   // 绝对时间窗口起点
        ros::Time time_end;     // 绝对时间窗口终点
    };
    std::vector<ScheduleCommand> schedule_cmds_;
    std::mutex scheduler_mutex_;                  // 保证回调线程安全
    bool dynamic_routing();
    void UpdateVehicleStateFromOdom();
private:

    // 场景相关
    std::string scene_type_;
    std::unordered_map<std::pair<int, int>, carla::geom::Transform, PairHash> road_lane_to_entry_stop_line_;
    std::unordered_map<std::pair<int, int>, carla::geom::Transform, PairHash> road_lane_to_exit_reference_;
    std::unordered_map<int, int> road_to_traffic_light_;

    // 车辆相关
    DriverState last_non_emergency_state_ = DriverState::INIT;
    DriverState driver_state_ = DriverState::INIT;

    geometry_msgs::Pose ego_pose;
    nav_msgs::Odometry ego_odmo;
    carla::geom::Transform ego_transform;
    carla::geom::Vector3D ego_forward;
    carla::SharedPtr<carla::client::Waypoint> ego_wpt;
    std::vector<carla::geom::Location> ego_corners;
    Polygon ego_polygon;
    std::vector<carla::geom::Transform> ego_path;

    std::vector<carla::geom::Transform> interp_path;

    Polygon ego_path_poly;

    double cruise_speed_kph;
    double current_speed_;

    // 感知信息
    std::mutex objects_mutex_;
    double detection_range;
    std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>> last_valid_ego_wpts_;  ///< 自车上次有效的目标路径点
    std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>> last_valid_tar_wpts_;  ///< 目标车辆上次有效的目标路径点

    // 交通灯信息
    std::mutex traffic_light_mutex_;
    std::unordered_map<int, road_side_system::TrafficLightPhase> traffic_light_map_;

    // 周围物体信息
    std::vector<derived_object_msgs::Object> carla_objects_;  ///< 物体信息

    // ROS订阅者
    ros::Subscriber objects_sub;                                ///< 物体信息订阅
    ros::Subscriber traffic_lights_phases_sub;                  ///< 交通灯相位信息订阅
    ros::Subscriber scheduler_sub_;
    ros::Publisher reckless_pub_;
    // 场景状态标志
    std::unordered_map<int32_t, int> target_stationary_counter_;  ///< 目标静止计数器


    // 交通灯相关
    carla::SharedPtr<carla::client::TrafficLight> affecting_traffic_light;  ///< 影响自车的交通灯
    carla::rpc::TrafficLightState last_light_state;  ///< 上次交通灯状态

    // 新增函数声明
    void update(carla::SharedPtr<carla::client::Vehicle> vehicle);
    void uodate_vehicle_info();
    void update_path_waypoint();
    void update_intention();
    void update_vehicle_light(bool is_active, carla::SharedPtr<carla::client::Vehicle> vehicle);
    std::shared_ptr<std::pair<carla::geom::Transform, local_planner::RoadOption>> get_road_option();

    // 状态处理与切换
    bool has_valid_position() const;
    double calcu_distance_to_entry_line(const carla::SharedPtr<carla::client::Waypoint>& wp,
                                const carla::geom::Location& ego_loc) const;
                                 
    bool yield_in_decision_area();
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
    
    // 回调函数
    void objects_call_back(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    void trafficlight_phase_call_back(const road_side_system::TrafficLightPhaseArray::ConstPtr& msg);
    void schedulerCallback(const road_side_system_type::PoseWithTimeWindowArray::ConstPtr& msg);
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
        const carla::geom::Transform& tar_transform,
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
        double spacing,
        std::vector<carla::geom::Transform>& interp_path);

    std::optional<std::pair<double, size_t>> project_point_to_path(
        const carla::geom::Location& point,
        const std::vector<carla::geom::Transform>& interp_path,
        const std::vector<double>& s_list,
        double eps_dist_sq);

    std::string DriverStateToString(DriverState state);
    void creat_obstacle(std::vector<planner::Obstacle>& AllObstacle,
                        const derived_object_msgs::Object& obj,
                        const geometry_msgs::PoseArray& tar_path);
    
    void create_traffic_light_obstacle(std::vector<planner::Obstacle>& AllObstacle);
    void create_schedule_obstacles(
                        std::vector<planner::Obstacle>& AllObstacle,
                        std::vector<ScheduleCommand>& cmds,
                        const ReferenceLine &reference_line);
    void generate_test_schedule_commands(std::vector<ScheduleCommand>& cmds);
    void InitializeRandomBehaviorOnce();

private:
    std::vector<ReferencePoint> reference_points;
    std::vector<double> accumulated_s;
    std::pair<std::vector<double>, std::vector<double>> reference_path;
    TrajectoryPoint planning_init_point;
    SpeedData speed_data;
    Obstacle_avoid oba;
    CubicSpline2D* csp = nullptr;
    DiscretizedTrajectory best_path; // 最佳路径
    PublishableTrajectory pb_planned_trajectory_;
    std::vector<planner::Obstacle> AllObstacle;

    bool has_last_trajectory_ = false;
    bool is_running_ = false;
    ros::Time planning_start_time_;
    VehicleStateProvider vehicle_state_provider_;
private:
    Param_Configs Config_;
    EMPlanner EM;
    bool global_path_updated_ = false;  // ✅ 新增：全局路径更新标志
    int last_matched_point_idx_ = -1;
    bool has_first_matched_point_ = false;
    int current_lane_change_index_ = 0;
    double trigger_time_first_ = -1.0;
    int lane_change_index_ = 0;
    int speed_change_index_ = 0;
    int speed = 0.0;
    int distance = 0.0;
    bool behavior_initialized_ = false;
    struct SpawnPosition {
        double x{0.0};
        double y{0.0};
        double z{0.0};
    };
    SpawnPosition spawn_position_;

};

} // namespace driver_models

#endif // MANUAL_DRIVER_H 