#include "driver_models/driver_model_manager.h"

DriverModelManager::DriverModelManager(ros::NodeHandle& nh, const carla::client::World& world, size_t pool_size)
: nh_(nh), world_(world), scheduler_(pool_size) {
    vehicle_config_sub_ = nh_.subscribe("/carla/vehicle_config", 30, &DriverModelManager::VehicleConfigCallback, this);
    ack_pub_ = nh_.advertise<driver_models_types::VehicleConfigAck>("/driver_model_manager/ack", 50);
    builder_thread_ = std::thread(&DriverModelManager::builderLoop, this);
}

DriverModelManager::~DriverModelManager() {
    stop_builder_.store(true);
    cfg_cv_.notify_all();
    if (builder_thread_.joinable()) builder_thread_.join();
    vehicle_config_sub_.shutdown();
    ROS_INFO("DriverModelManager shutdown complete.");
}

void DriverModelManager::VehicleConfigCallback(
    const driver_models_types::VehicleConfig::ConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        // ⭐ 读取总车辆数（只读取一次）
        if (!total_vehicle_count_received_) {
            expected_total_vehicles_ = msg->total_vehicles;
            total_vehicle_count_received_ = true;

            ROS_WARN("📌 Received total vehicle count = %zu", expected_total_vehicles_);
        }
        // 判断 driver 是否已经创建过
        if (received_roles_.count(msg->role_name)) {
            ROS_INFO("Duplicate config received for %s (still pending). Ignoring duplicate.",
                    msg->role_name.c_str());
            return;
        }

        received_roles_.insert(msg->role_name);
        // ⭐ 存入待创建列表
        pending_cfgs_[msg->role_name] = *msg;
    }

    cfg_cv_.notify_one();
}

void DriverModelManager::builderLoop() {
    while (ros::ok() && !stop_builder_.load()) {
        driver_models_types::VehicleConfig cfg;
        {
            std::unique_lock<std::mutex> lk(cfg_mtx_);
            cfg_cv_.wait(lk, [this] {
                return stop_builder_.load() || !pending_cfgs_.empty();
            });
            if (stop_builder_.load() && pending_cfgs_.empty()) break;
            auto it = pending_cfgs_.begin();
            cfg = std::move(it->second);
            pending_cfgs_.erase(it);
        }
        spawnDriver(cfg);
    }
}

void DriverModelManager::spawnDriver(const driver_models_types::VehicleConfig& cfg) {
    std::shared_ptr<driver_model::BaseDriver> drv;
    try {
        if (cfg.driver_model == "auto_driver") {
            drv = std::make_shared<driver_model::AutoDriver>(nh_, cfg, world_);
        } else if (cfg.driver_model == "manual_driver") {
            drv = std::make_shared<driver_model::ManualDriver>(nh_, cfg, world_);
        } else {
            ROS_WARN("Unknown driver_model %s for %s", cfg.driver_model.c_str(), cfg.role_name.c_str());
            return;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Driver construct failed for %s: %s", cfg.role_name.c_str(), e.what());
        return;
    }

    // ============================
    //   添加任务（但不立即启动！）
    // ============================
    Task plan_task;
    plan_task.vehicle_id = cfg.carla_id;
    plan_task.period = std::chrono::milliseconds(100);
    plan_task.next_deadline = std::chrono::steady_clock::now() + plan_task.period;
    plan_task.func = [drv]() {
        if (!drv->has_reached_goal()) drv->run_step();
    };
    scheduler_.addTask(plan_task);

    Task control_task;
    control_task.vehicle_id = cfg.carla_id;
    control_task.period = std::chrono::milliseconds(20);
    control_task.next_deadline = std::chrono::steady_clock::now() + control_task.period;
    control_task.func = [drv]() {
        if (!drv->has_reached_goal()) drv->get_local_planner()->run_step();
    };
    scheduler_.addTask(control_task);

    ROS_ERROR("Driver spawned: %s(%d) model=%s", 
              cfg.role_name.c_str(), cfg.carla_id, cfg.driver_model.c_str());

    int now = ++created_vehicles_;

    driver_models_types::VehicleConfigAck ack;
    ack.role_name = cfg.role_name;
    ack.carla_id = cfg.carla_id;
    ack.received = true;

    ack_pub_.publish(ack);
    ROS_INFO("ACK sent for vehicle: %s", cfg.role_name.c_str());

    // ============================
    //     判断是否可以启动调度器
    // ============================
    ROS_INFO("[DriverModelManager] Created %d drivers (expected %d)",
             created_vehicles_.load(), expected_total_vehicles_);

    // Case 1: 不等待 —— 直接启动 scheduler
    if (expected_total_vehicles_ == -1) {
        bool expected = false;
        if (scheduler_started_.compare_exchange_strong(expected, true)) {
            scheduler_.start();
            ROS_WARN("🚀 Scheduler started immediately (expected_total_vehicles = -1)");
        }
        return;
    }

    // Case 2: 车辆达到预设数量
    if (expected_total_vehicles_ > 0 && now >= expected_total_vehicles_) {
        bool expected = false;
        if (scheduler_started_.compare_exchange_strong(expected, true)) {
            scheduler_.start();
            ROS_WARN("🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀🚀 All %d vehicles created. Scheduler started!", expected_total_vehicles_);
        }
    }
}

