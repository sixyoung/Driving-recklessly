#include "driver_models/base_driver.h"

namespace driver_model {

BaseDriver::BaseDriver(ros::NodeHandle& nh, 
                     const driver_models_types::VehicleConfig& vehicle_config,
                     const carla::client::World& world)
    : nh_(nh)
    , world_(world)
    , vehicle_config_(vehicle_config)
    , role_name_(vehicle_config.role_name)
    , id_(vehicle_config.carla_id)
    , ego_length(std::numeric_limits<double>::infinity())
    , ego_width(std::numeric_limits<double>::infinity())
    , driver_intention_(DrivingIntention::VOID)
    , avoid_vehicle_(true)
    , avoid_traffic_light_(true)
{
    local_planner_ = std::make_shared<local_planner::LocalPlanner>(nh_, vehicle_config_);
    debug_helper_ = std::make_unique<carla::client::DebugHelper>(world_.MakeDebugHelper());
    control_cmd_pub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name_ + "/vehicle_control_cmd", 10);

}

BaseDriver::~BaseDriver() {
    destroy();
}

void BaseDriver::InitializeROSResources() {

    // 初始化路径服务客户端
    get_path_client = nh_.serviceClient<driver_models_types::PathWithOptionsService>("/carla_waypoint_publisher/get_path");
    destroy_carla_object_client = nh_.serviceClient<carla_msgs::DestroyObject>("/carla/vehicle/destroy");
}


std::tuple<bool, BaseSolver::OutputInfo> BaseDriver::PlanSpeedProfile(
    const std::vector<speed_planner::Obstacle>& obstacles,
    const std::vector<double>& s_list,
    const std::vector<double>& v_limit_list,
    bool should_output_results) {

    ROS_WARN("[BaseDriver] ⚠️ PlanSpeedProfile() 默认实现被调用！你是否忘记在子类中重写该函数？");

    return std::make_tuple(false, BaseSolver::OutputInfo{});
}


std::shared_ptr<driver_models_types::PathWithOptions> 
BaseDriver::GetPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
    driver_models_types::PathWithOptionsService srv;
    srv.request.role_name = role_name_;
    srv.request.start = start;
    srv.request.goal = goal;
    if (get_path_client.call(srv)) {
        if(srv.response.success){
            ROS_INFO("%s: %s", role_name_.c_str(), srv.response.message.c_str());
            return std::make_shared<driver_models_types::PathWithOptions>(srv.response.path);
        }else{
            ROS_ERROR("%s: %s", role_name_.c_str(), srv.response.message.c_str());
            return nullptr;
        }
    } else {
        ROS_ERROR("%s: Service call 'get_path' failed at goal: (%.2f, %.2f, %.2f)", 
            role_name_.c_str(), goal.position.x, goal.position.y, goal.position.z);
        return nullptr;
    }
}

bool BaseDriver::destroy_carla_object(int32_t carla_id) {
    // 构造请求
    carla_msgs::DestroyObject srv;
    srv.request.id = carla_id;

    // 调用服务
    if (destroy_carla_object_client.call(srv)) {
        ROS_INFO("Successfully requested destruction of object ID %d", carla_id);
        return true;
    } else {
        ROS_ERROR("Failed to call DestroyObject service for ID %d", carla_id);
        return false;
    }
}

void BaseDriver::SetAvoidTrafficLight(bool active) {
    avoid_traffic_light_ = active;
}

void BaseDriver::SetAvoidVehicle(bool active) {
    avoid_vehicle_ = active;
}

void BaseDriver::destroy() {
    // 销毁车辆
    if (vehicle_carla_actor_) {
        vehicle_carla_actor_.reset();
    }
    // 销毁局部规划器
    if (local_planner_) {
        local_planner_.reset();
    }
    // 销毁调试助手
    if (debug_helper_) {
        ROS_INFO("%s: Destroying debug helper", role_name_.c_str());
        debug_helper_.reset();
    }
}

} // namespace driver_model 