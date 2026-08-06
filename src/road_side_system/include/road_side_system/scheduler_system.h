#ifndef ROAD_SIDE_SYSTEM_SCHEDULER_SYSTEM_H
#define ROAD_SIDE_SYSTEM_SCHEDULER_SYSTEM_H

#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <vector>

#include <derived_object_msgs/ObjectArray.h>
#include <road_side_system_type/VehicleInfo.h>
#include <road_side_system_type/VehicleInfoArray.h>
#include <road_side_system_type/PathInfo.h>
#include <driver_models_types/PathWithOptionsService.h>
#include <driver_models_types/VehicleConfig.h>
#include <driver_models_types/VehicleGoalStatus.h>
#include <road_side_system_type/RoadSideSysInit.h>
#include <road_side_system_type/VehicleInfoOutarray.h>
#include <road_side_system_type/PoseWithTimeWindow.h>
#include <road_side_system_type/PoseWithTimeWindowArray.h>
namespace road_side_system
{

class SchedulerSystem
{
public:
    explicit SchedulerSystem();
    ~SchedulerSystem();
    void PublishOnce();  // 对外接口：每帧发布一次
    // void SavePathInfoToJson(const road_side_system_type::PathInfo& path_info);
private:
    // ========== ROS 通信接口 ==========
    ros::NodeHandle nh_;
    ros::Subscriber objects_sub_;
    ros::Subscriber vehicle_config_sub_;
    ros::Subscriber goal_status_sub_;
    ros::ServiceServer scheduler_info_srv_;

    ros::Publisher vehicle_info_pub_;
    ros::Subscriber vehicle_info_sub_;
    std::unordered_map<int, ros::Publisher> vehicle_pubs_;

    // ========== 任务队列与线程池 ==========
    std::queue<driver_models_types::VehicleConfig> task_queue_;
    std::mutex queue_mutex_, config_mutex_, objects_mutex_;
    std::condition_variable queue_cv_;
    bool stop_threads_ = false;
    std::vector<std::thread> workers_;
    int worker_count_ = 1;  // ✅ 当前仅启用一个线程，但结构支持多线程

    // ========== 存储结构 ==========
    std::unordered_map<int, driver_models_types::VehicleConfig> vehicle_config_map_;
    std::unordered_map<int, road_side_system_type::PathInfo> vehicle_path_map_;
    std::vector<derived_object_msgs::Object> carla_objects_;


    void ObjectsCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    void VehicleConfigCallback(const driver_models_types::VehicleConfig::ConstPtr& msg);
    void GoalStatusCallback(const driver_models_types::VehicleGoalStatus::ConstPtr& msg);
    void BuildVehicleInfoArray(road_side_system_type::VehicleInfoArray& out_msg);
    void WorkerThread(int thread_id);
    void vehicleInfoArrayCallback(const road_side_system_type::VehicleInfoOutarray::ConstPtr& msg);
    void publishToVehicle(int vehicle_id, const geometry_msgs::Pose& pose);
    bool GetSchedulerInfo(
        road_side_system_type::RoadSideSysInit::Request &req,
        road_side_system_type::RoadSideSysInit::Response &res);
    std::shared_ptr<driver_models_types::PathWithOptions> GetPath(
        const std::string &role_name,
        const geometry_msgs::Pose &start,
        const geometry_msgs::Pose &goal,
        ros::ServiceClient &client);
    const std::vector<int> OBJECT_VEHICLE_CLASSIFICATION = {
        derived_object_msgs::Object::CLASSIFICATION_CAR,
        derived_object_msgs::Object::CLASSIFICATION_BIKE,
        derived_object_msgs::Object::CLASSIFICATION_MOTORCYCLE,
        derived_object_msgs::Object::CLASSIFICATION_TRUCK,
        derived_object_msgs::Object::CLASSIFICATION_OTHER_VEHICLE
    };
    // std::mutex file_mutex_;
    // std::string path_save_file_ = "/home/bob/文档/备份/demo05/src/road_side_system/data/all_paths.json";
};

} // namespace road_side_system

#endif
