#pragma once
#include <ros/ros.h>
#include <carla/client/World.h>
#include <driver_models_types/VehicleConfig.h>
#include <driver_models_types/VehicleConfigAck.h>
#include "task_scheduler.h"
#include "driver_models/base_driver.h"
#include "driver_models/auto_driver.h"
#include "driver_models/manual_driver.h"

class DriverModelManager {
public:
    DriverModelManager(ros::NodeHandle& nh, const carla::client::World& world, size_t pool_size = 8);
    ~DriverModelManager();

    void VehicleConfigCallback(const driver_models_types::VehicleConfig::ConstPtr& msg);

private:
    void builderLoop();
    void spawnDriver(const driver_models_types::VehicleConfig& cfg);

    ros::NodeHandle nh_;
    carla::client::World world_;
    ros::Subscriber vehicle_config_sub_;
    ros::Publisher ack_pub_;


    TaskScheduler scheduler_;  // 统一任务调度器

    std::thread builder_thread_;
    std::atomic<bool> stop_builder_{false};
    std::mutex cfg_mtx_;
    std::condition_variable cfg_cv_;
    std::unordered_map<std::string, driver_models_types::VehicleConfig> pending_cfgs_;
    size_t expected_total_vehicles_ = 0;   // 总车辆数
    std::atomic<int> created_vehicles_{0};
    bool total_vehicle_count_received_ = false;  // 是否已读取
    std::atomic<bool> scheduler_started_{false};
    std::unordered_set<std::string> received_roles_;

};
