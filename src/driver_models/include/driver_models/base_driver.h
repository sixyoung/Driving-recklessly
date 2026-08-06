/**
 * ============================================================================
 * @file base_driver.h
 * @brief 驾驶员基类，定义了驾驶员模型的基本接口和通用功能
 * ============================================================================
 */

#ifndef DRIVER_MODEL_BASE_DRIVER_H
#define DRIVER_MODEL_BASE_DRIVER_H

#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <set>

// ROS消息类型
#include <derived_object_msgs/ObjectArray.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/DestroyObject.h>

// CARLA相关头文件
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/DebugHelper.h>
#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
// 路径规划相关
#include <carla_waypoint_types/GetWaypoint.h>
#include <carla_waypoint_types/CarlaWaypoint.h>

// 项目内部依赖
#include <local_planner/local_planner.h>
#include <common/common.h>
#include <driver_models_types/VehicleConfig.h>
#include <driver_models_types/PathWithOptions.h>
#include <driver_models_types/PathWithOptionsService.h>

#include <speed_planner/jeker_speed_planner.h>


namespace driver_model {

using namespace local_planner;
using namespace common;

/**
 * @brief 驾驶员状态枚举
 */
enum class DrivingIntention {
    NAVIGATING = 1,          ///< 导航中
    BLOCKED_BY_VEHICLE = 2,  ///< 被其他车辆阻挡
    BLOCKED_RED_LIGHT = 3,   ///< 被红灯阻挡
    TURNING_LEFT = 4,        ///< 左转中
    TURNING_RIGHT = 5,       ///< 右转中
    GOING_STRAIGHT = 6,      ///< 直行中
    CHANGELANELEFT = 7,      ///< 左变道中
    CHANGELANERIGHT = 8,     ///< 右变道中
    VOID = 9,                 ///< 无效状态

};

/**
 * @brief 驾驶员基类
 */
class BaseDriver {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param vehicle_config 车辆配置
     * @param world CARLA世界实例
     */
    BaseDriver(ros::NodeHandle& nh, 
              const driver_models_types::VehicleConfig& vehicle_config,
              const carla::client::World& world);

    virtual ~BaseDriver();

    // 必须实现：返回 Carla 车辆 ID
    virtual int get_vehicle_id() const = 0;
    virtual std::shared_ptr<local_planner::LocalPlanner> get_local_planner() { return nullptr; }
    virtual void mark_destroyed() {  is_destroyed_ = true; }

    // 核心功能接口
    virtual void Initialize() = 0;
    virtual void PerceiveEnvironment() = 0;          
    virtual void MakeDecision() = 0;               
    virtual std::tuple<bool, BaseSolver::OutputInfo> PlanSpeedProfile(
                    const std::vector<speed_planner::Obstacle>& obstacles,
                    const std::vector<double>& s_list,
                    const std::vector<double>& v_limit_list,
                    bool should_output_results);
    virtual void run_step() = 0;                     
    virtual bool has_reached_goal() const = 0;

    // 公共接口
    std::string GetRoleName() const {
        return role_name_;
    }
    virtual void destroy();

    virtual std::shared_ptr<driver_models_types::PathWithOptions> GetPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
    bool destroy_carla_object(int32_t carla_id);
    std::atomic<bool> in_flight{false};
protected:
    // 受保护的成员变量
    ros::NodeHandle& nh_;                            ///< ROS节点句柄
    mutable std::mutex data_mutex;                  ///< 数据互斥锁
    bool is_destroyed_;  

    // 车辆相关
    carla::client::World world_;                    ///< CARLA世界实例
    std::string role_name_;                         ///< 车辆角色名称
    uint32_t id_;                                   ///< 自车ID
    carla::SharedPtr<carla::client::Vehicle> vehicle_carla_actor_; ///< 自车实例
    driver_models_types::VehicleConfig vehicle_config_; ///< 车辆配置
    float ego_length;                             ///< 车辆前后尺寸
    float ego_width;                             ///< 车辆左右尺寸

    // 状态相关
    DrivingIntention driver_intention_ = DrivingIntention::VOID;             ///< 驾驶员意图
    bool avoid_vehicle_ = true;                            ///< 是否避让其他车辆
    bool avoid_traffic_light_ = true;                      ///< 是否避让交通信号灯

    // 控制相关
    carla_msgs::CarlaEgoVehicleControl control_command_; ///< 控制指令
    std::shared_ptr<driver_models_types::PathWithOptions> global_path_; ///< 全局路径
    std::vector<std::shared_ptr<driver_models_types::PathWithOptions>> global_paths_; ///< 全局路径库
    std::vector<std::pair<carla::geom::Transform, RoadOption>> path_points;
    std::shared_ptr<local_planner::LocalPlanner> local_planner_; ///< 局部规划器
    std::unique_ptr<carla::client::DebugHelper> debug_helper_; ///< 调试助手

    // ROS相关
    ros::ServiceClient get_path_client;             ///< 获取路径服务客户端
    ros::ServiceClient destroy_carla_object_client;             ///< 获取路径服务客户端
    ros::Publisher control_cmd_pub_;

    // 受保护的方法
    virtual void InitializeROSResources();          ///< 初始化ROS资源
    void SetAvoidTrafficLight(bool active);         ///< 设置是否避让交通信号灯
    void SetAvoidVehicle(bool active);             ///< 设置是否避让车辆
    
    // 常量定义
    const std::vector<int> OBJECT_VEHICLE_CLASSIFICATION = {
        derived_object_msgs::Object::CLASSIFICATION_CAR,
        derived_object_msgs::Object::CLASSIFICATION_BIKE,
        derived_object_msgs::Object::CLASSIFICATION_MOTORCYCLE,
        derived_object_msgs::Object::CLASSIFICATION_TRUCK,
        derived_object_msgs::Object::CLASSIFICATION_OTHER_VEHICLE
    };
};

} // namespace driver_model

#endif // DRIVER_MODEL_BASE_DRIVER_H 