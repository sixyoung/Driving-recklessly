#pragma once

#include <ros/ros.h>
#include <carla/client/World.h>
#include <driver_models_types/VehicleConfig.h>
#include <std_msgs/UInt32.h>
#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <shared_mutex>
#include "driver_models/manual_driver.h"
#include "driver_models/auto_driver.h"
#include "driver_models/base_driver.h"

#include <chrono>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <cstdio>   // std::snprintf

using Clock = std::chrono::steady_clock;
using namespace driver_model;

class DriverModelManager {
public:
    // 构造函数，传入 ROS NodeHandle 和 Carla World
    DriverModelManager(ros::NodeHandle& nh, const carla::client::World& world);
    ~DriverModelManager();

    // 每个时间步调用所有车辆的 run_step
    void run_all_steps();

    // 车辆配置信息的回调函数
    void VehicleConfigCallback(const driver_models_types::VehicleConfig::ConstPtr& msg);

private:
    void worker_thread();

    ros::NodeHandle& nh_;
    const carla::client::World& world_;
    ros::Subscriber vehicle_config_sub_;          // 订阅器
    ros::Publisher driver_model_created_pub_;

    // 双缓冲机制
    std::map<std::string, std::shared_ptr<BaseDriver>> drivers_read_;   // 读取缓冲区
    std::map<std::string, std::shared_ptr<BaseDriver>> drivers_write_;  // 写入缓冲区
    std::mutex read_mutex_;   // 保护读取缓冲区
    std::mutex write_mutex_;  // 保护写入缓冲区
    std::atomic<bool> buffer_swapped_{false};  // 标记是否需要交换缓冲区

    // 线程池相关
    std::vector<std::thread> thread_pool_;
    std::queue<std::function<void()>> task_queue_;
    std::mutex task_queue_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_thread_pool_{false};
    std::atomic<int> active_tasks_{0};

    int last_tick = -1;  // 记录上一帧的 tick 号

    std::mutex shared_data_mutex_;
    ros::Subscriber object_sub_;
    std::map<uint32_t, derived_object_msgs::Object> current_objects_;

    bool dynamic_spawn_flag;
    void ObjectsCallBack(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    std::map<uint32_t, derived_object_msgs::Object> getCurrentObjects();

    // 生成队列 & 专用构造线程
    std::mutex cfg_mtx_;
    std::condition_variable cfg_cv_;
    std::unordered_map<std::string, driver_models_types::VehicleConfig> pending_cfgs_; // 按 role_name 去重

    std::atomic<bool> stop_builder_{false};
    std::thread builder_thread_;

    void builderLoop();   // 新增：后台构造线程
};

