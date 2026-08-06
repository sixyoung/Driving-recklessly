#include "road_side_system/scheduler_system.h"
#include <nlohmann/json.hpp>
// using ordered_json = nlohmann::ordered_json;
#include <fstream>
using json = nlohmann::json;

namespace road_side_system
{

SchedulerSystem::SchedulerSystem()
{
    // 1️⃣ 订阅对象与配置
    objects_sub_ = nh_.subscribe("/carla/objects", 1,
        &SchedulerSystem::ObjectsCallback, this);
    vehicle_config_sub_ = nh_.subscribe("/carla/vehicle_config", 30,
        &SchedulerSystem::VehicleConfigCallback, this);
    goal_status_sub_ = nh_.subscribe("/carla/goal_status", 10,
        &SchedulerSystem::GoalStatusCallback, this);
    // 2️⃣ 发布车辆信息
    vehicle_info_pub_ = nh_.advertise<road_side_system_type::VehicleInfoArray>(
        "/road_side/vehicle_info_array", 1, true);
    scheduler_info_srv_ = nh_.advertiseService(
        "get_scheduler_info",
        &SchedulerSystem::GetSchedulerInfo,
        this);
    vehicle_info_sub_ = nh_.subscribe(
        "/road_side_system_out", 10,
        &SchedulerSystem::vehicleInfoArrayCallback, this);
    // 3️⃣ 启动线程池（结构支持多线程）
    for (int i = 0; i < worker_count_; ++i)
        workers_.emplace_back(&SchedulerSystem::WorkerThread, this, i);

    ROS_INFO("[SchedulerSystem] Initialized with %d worker thread(s).", worker_count_);
}

SchedulerSystem::~SchedulerSystem()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_threads_ = true;
    }
    queue_cv_.notify_all();

    for (auto &t : workers_)
        if (t.joinable()) t.join();

    ROS_INFO("[SchedulerSystem] Worker thread(s) stopped safely.");
}

void SchedulerSystem::ObjectsCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    carla_objects_.clear();
    carla_objects_.reserve(msg->objects.size());
    for (const auto& obj : msg->objects)
        carla_objects_.push_back(obj);
}

void SchedulerSystem::VehicleConfigCallback(const driver_models_types::VehicleConfig::ConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.push(*msg);
    }
    queue_cv_.notify_one();

    ROS_INFO("[SchedulerSystem] Queued VehicleConfig: id=%d, role=%s, path_id=%d",
             msg->carla_id, msg->role_name.c_str(), msg->path_id);
}

void SchedulerSystem::WorkerThread(int thread_id)
{
    // 每个线程有自己独立的 ServiceClient
    ros::ServiceClient client = nh_.serviceClient<driver_models_types::PathWithOptionsService>(
        "/carla_waypoint_publisher/get_path");

    while (ros::ok() && !stop_threads_)
    {
        driver_models_types::VehicleConfig cfg;

        // 等待任务
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !task_queue_.empty() || stop_threads_;
            });
            if (stop_threads_) break;

            cfg = std::move(task_queue_.front());
            task_queue_.pop();
        }

        // 调用服务
        auto path_ptr = GetPath(cfg.role_name, cfg.spawn_point.pose, cfg.goal_point.pose, client);

        // 构造 PathInfo
        road_side_system_type::PathInfo path_info;
        path_info.path_id = cfg.path_id;
        path_info.stop_line_pose = cfg.stop_line_pose;
        path_info.exit_intersection_pose = cfg.exit_intersection_pose;
        path_info.road_option = cfg.road_option;

        if (path_ptr) {
            path_info.path = *path_ptr;
            ROS_INFO("[Worker-%d][%s] Path generated (%zu waypoints).",
                     thread_id, cfg.role_name.c_str(), path_ptr->waypoints.size());
            // SavePathInfoToJson(path_info);
        } else {
            ROS_WARN("[Worker-%d][%s] Path generation failed.", thread_id, cfg.role_name.c_str());
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            vehicle_config_map_[cfg.carla_id] = cfg;
            vehicle_path_map_[cfg.carla_id] = path_info;
        }
    }
}

std::shared_ptr<driver_models_types::PathWithOptions>
SchedulerSystem::GetPath(const std::string &role_name,
                         const geometry_msgs::Pose &start,
                         const geometry_msgs::Pose &goal,
                         ros::ServiceClient &client)
{
    driver_models_types::PathWithOptionsService srv;
    srv.request.role_name = role_name;
    srv.request.start = start;
    srv.request.goal = goal;
    if (client.call(srv)) {
        if(srv.response.success){
            ROS_INFO("[SchedulerSystem][%s]: %s", role_name.c_str(), srv.response.message.c_str());
            return std::make_shared<driver_models_types::PathWithOptions>(srv.response.path);
        }else{
            ROS_ERROR("[SchedulerSystem][%s]: %s", role_name.c_str(), srv.response.message.c_str());
            return nullptr;
        }
    } else {
        ROS_ERROR("[SchedulerSystem][%s]: Service call 'get_path' failed at goal: (%.2f, %.2f, %.2f)", 
            role_name.c_str(), goal.position.x, goal.position.y, goal.position.z);
        return nullptr;
    }
}

void SchedulerSystem::BuildVehicleInfoArray(road_side_system_type::VehicleInfoArray &out_msg)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
     std::lock_guard<std::mutex> lock2(config_mutex_);  // ✅ 新增这行，保护 vehicle_path_map_
    out_msg.VehicleInfoArray.clear();

    for (const auto& obj : carla_objects_)
    {
        road_side_system_type::VehicleInfo info;
        info.vehicle_id = obj.id;
        info.length = static_cast<int>(obj.shape.dimensions[0]);
        info.width  = static_cast<int>(obj.shape.dimensions[1]);
        info.speed  = std::sqrt(obj.twist.linear.x * obj.twist.linear.x +
                                obj.twist.linear.y * obj.twist.linear.y);
        info.acceleration = 0.0;
        info.pose = obj.pose;

        // ✅ 检查 vehicle_config_map_ 是否有该 id
        auto cfg_it = vehicle_config_map_.find(obj.id);
        if (cfg_it != vehicle_config_map_.end()) {
            info.role_name = cfg_it->second.role_name;  // 预留字段
        } else {
            ROS_WARN("[SchedulerSystem] No vehicle_config found for id=%d; role_name set to empty.", obj.id);
            info.role_name = "";
        }
        // 若已有路径信息则附加
        if (vehicle_path_map_.count(obj.id)){
            info.path = vehicle_path_map_[obj.id];
            out_msg.VehicleInfoArray.push_back(info);
                if (vehicle_path_map_[obj.id].path.waypoints.empty()) {
                    ROS_ERROR("[SchedulerSystem] Path info for vehicle_id=%d is empty; skipping.", obj.id);
                }
        }else{
            ROS_WARN("[SchedulerSystem] No path info for vehicle_id=%d; skipping.", obj.id);
        }
    }
}

void SchedulerSystem::PublishOnce()
{
    road_side_system_type::VehicleInfoArray msg;
    BuildVehicleInfoArray(msg);
    vehicle_info_pub_.publish(msg);

    ROS_INFO("[SchedulerSystem] Published VehicleInfoArray (%zu vehicles)",
             msg.VehicleInfoArray.size());
}

void SchedulerSystem::GoalStatusCallback(const driver_models_types::VehicleGoalStatus::ConstPtr& msg)
{
    // 只处理已到达目标的车辆
    if (!msg->reached_goal)
        return;

    int carla_id = msg->carla_id;
    std::string vehicle_id = msg->vehicle_id;

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        auto it = vehicle_path_map_.find(carla_id);
        if (it != vehicle_path_map_.end()) {
            vehicle_path_map_.erase(it);
            ROS_INFO("[SchedulerSystem] Vehicle '%s' (id=%d) reached goal -> PathInfo erased.",
                     vehicle_id.c_str(), carla_id);
        } else {
            ROS_WARN("[SchedulerSystem] Vehicle '%s' (id=%d) reached goal, but no PathInfo found.",
                     vehicle_id.c_str(), carla_id);
        }
        auto it_ = vehicle_config_map_.find(carla_id);
        if (it_ != vehicle_config_map_.end()) {
            vehicle_config_map_.erase(it_);
            ROS_INFO("[SchedulerSystem] Vehicle '%s' (id=%d) reached goal -> vehicle_config erased.",
                     vehicle_id.c_str(), carla_id);
        } else {
            ROS_WARN("[SchedulerSystem] Vehicle '%s' (id=%d) reached goal, but no vehicle_config found.",
                     vehicle_id.c_str(), carla_id);
        }
        
    }
}
// void road_side_system::SchedulerSystem::SavePathInfoToJson(
//     const road_side_system_type::PathInfo& path_info)
// {
//     std::lock_guard<std::mutex> lock(file_mutex_);

//     const std::string filename = path_save_file_;
//     ordered_json j_new;  // 新path数据

//     // 1. 构造新对象
//     j_new["path_id"] = path_info.path_id;
//     j_new["road_option"] = path_info.road_option;

//     j_new["stop_line_pose"] = {
//         {"position", {
//             {"x", path_info.stop_line_pose.position.x},
//             {"y", path_info.stop_line_pose.position.y},
//             {"z", path_info.stop_line_pose.position.z}
//         }},
//         {"orientation", {
//             {"x", path_info.stop_line_pose.orientation.x},
//             {"y", path_info.stop_line_pose.orientation.y},
//             {"z", path_info.stop_line_pose.orientation.z},
//             {"w", path_info.stop_line_pose.orientation.w}
//         }}
//     };

//     j_new["exit_intersection_pose"] = {
//         {"position", {
//             {"x", path_info.exit_intersection_pose.position.x},
//             {"y", path_info.exit_intersection_pose.position.y},
//             {"z", path_info.exit_intersection_pose.position.z}
//         }},
//         {"orientation", {
//             {"x", path_info.exit_intersection_pose.orientation.x},
//             {"y", path_info.exit_intersection_pose.orientation.y},
//             {"z", path_info.exit_intersection_pose.orientation.z},
//             {"w", path_info.exit_intersection_pose.orientation.w}
//         }}
//     };

//     j_new["path"]["header"] = {
//         {"frame_id", path_info.path.header.frame_id},
//         {"stamp", path_info.path.header.stamp.toSec()}
//     };

//     for (const auto& wp : path_info.path.waypoints)
//     {
//         ordered_json j_wp;
//         j_wp["pose"] = {
//             {"position", {
//                 {"x", wp.pose.position.x},
//                 {"y", wp.pose.position.y},
//                 {"z", wp.pose.position.z}
//             }},
//             {"orientation", {
//                 {"x", wp.pose.orientation.x},
//                 {"y", wp.pose.orientation.y},
//                 {"z", wp.pose.orientation.z},
//                 {"w", wp.pose.orientation.w}
//             }}
//         };
//         j_wp["road_option"] = wp.road_option;
//         j_new["path"]["waypoints"].push_back(j_wp);
//     }

//     // 2. 尝试读取已有文件（若存在）
//     ordered_json j_all = ordered_json::array();
//     std::ifstream ifs(filename);
//     if (ifs.is_open()) {
//         try {
//             ifs >> j_all;
//             if (!j_all.is_array()) {
//                 ROS_WARN("Existing file is not an array, will be replaced.");
//                 j_all = ordered_json::array();
//             }
//         } catch (...) {
//             ROS_WARN("Failed to parse existing JSON, will overwrite.");
//             j_all = ordered_json::array();
//         }
//         ifs.close();
//     }

//     // 3. 追加新的 path_info
//     j_all.push_back(j_new);

//     // 4. 写回文件（覆盖旧文件）
//     std::ofstream ofs(filename, std::ios::trunc);
//     if (!ofs.is_open()) {
//         ROS_ERROR("Failed to open path file for writing: %s", filename.c_str());
//         return;
//     }
//     ofs << j_all.dump(2) << std::endl;
//     ofs.close();

//     ROS_INFO("[PathLogger] Path %d appended to %s", path_info.path_id, filename.c_str());
// }


bool road_side_system::SchedulerSystem::GetSchedulerInfo(
        road_side_system_type::RoadSideSysInit::Request &req,
        road_side_system_type::RoadSideSysInit::Response &res)
{
    const std::string filename = "/home/bob/文档/备份/demo05/src/road_side_system/data/all_paths.json";

    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        ROS_ERROR("[SchedulerSystem] Failed to open %s", filename.c_str());
        return false;
    }

    json j_all;
    try {
        ifs >> j_all;
    } catch (const std::exception &e) {
        ROS_ERROR("[SchedulerSystem] Failed to parse %s: %s", filename.c_str(), e.what());
        return false;
    }

    if (!j_all.is_array()) {
        ROS_ERROR("[SchedulerSystem] JSON root is not an array.");
        return false;
    }

    // 清空旧内容
    res.MapPathInfo.clear();

    // 逐个解析 path
    for (const auto &j_path : j_all)
    {
        road_side_system_type::PathInfo path_info;

        // ===== 基本字段 =====
        path_info.path_id = j_path.value("path_id", 0);
        path_info.road_option = j_path.value("road_option", 0);

        // ===== stop_line_pose =====
        if (j_path.contains("stop_line_pose")) {
            auto stop_pos = j_path["stop_line_pose"]["position"];
            auto stop_ori = j_path["stop_line_pose"]["orientation"];
            path_info.stop_line_pose.position.x = stop_pos.value("x", 0.0);
            path_info.stop_line_pose.position.y = stop_pos.value("y", 0.0);
            path_info.stop_line_pose.position.z = stop_pos.value("z", 0.0);
            path_info.stop_line_pose.orientation.x = stop_ori.value("x", 0.0);
            path_info.stop_line_pose.orientation.y = stop_ori.value("y", 0.0);
            path_info.stop_line_pose.orientation.z = stop_ori.value("z", 0.0);
            path_info.stop_line_pose.orientation.w = stop_ori.value("w", 1.0);
        }

        // ===== exit_intersection_pose =====
        if (j_path.contains("exit_intersection_pose")) {
            auto exit_pos = j_path["exit_intersection_pose"]["position"];
            auto exit_ori = j_path["exit_intersection_pose"]["orientation"];
            path_info.exit_intersection_pose.position.x = exit_pos.value("x", 0.0);
            path_info.exit_intersection_pose.position.y = exit_pos.value("y", 0.0);
            path_info.exit_intersection_pose.position.z = exit_pos.value("z", 0.0);
            path_info.exit_intersection_pose.orientation.x = exit_ori.value("x", 0.0);
            path_info.exit_intersection_pose.orientation.y = exit_ori.value("y", 0.0);
            path_info.exit_intersection_pose.orientation.z = exit_ori.value("z", 0.0);
            path_info.exit_intersection_pose.orientation.w = exit_ori.value("w", 1.0);
        }

        // ===== path.header =====
        if (j_path.contains("path") && j_path["path"].contains("header")) {
            path_info.path.header.frame_id = j_path["path"]["header"].value("frame_id", "map");
            path_info.path.header.stamp = ros::Time(j_path["path"]["header"].value("stamp", 0.0));
        }

        // ===== waypoints =====
        if (j_path.contains("path") && j_path["path"].contains("waypoints")) {
            for (const auto &wp : j_path["path"]["waypoints"]) {
                driver_models_types::WaypointWithOption waypoint;
                auto pos = wp["pose"]["position"];
                auto ori = wp["pose"]["orientation"];
                waypoint.pose.position.x = pos.value("x", 0.0);
                waypoint.pose.position.y = pos.value("y", 0.0);
                waypoint.pose.position.z = pos.value("z", 0.0);
                waypoint.pose.orientation.x = ori.value("x", 0.0);
                waypoint.pose.orientation.y = ori.value("y", 0.0);
                waypoint.pose.orientation.z = ori.value("z", 0.0);
                waypoint.pose.orientation.w = ori.value("w", 1.0);
                waypoint.road_option = wp.value("road_option", 0);

                path_info.path.waypoints.push_back(waypoint);
            }
        }

        // 添加到结果
        res.MapPathInfo.push_back(path_info);
    }

    // ===== 构建路侧系统配置 =====
    res.RoadSideSysConfig.ProjectFrequence = 10;   // 规划频率 10Hz
    res.RoadSideSysConfig.RegionType = 1;          // 1 表示矩形区域

    // 区域中心点
    res.RoadSideSysConfig.Center.position.x = 0.0;
    res.RoadSideSysConfig.Center.position.y = 0.0;
    res.RoadSideSysConfig.Center.position.z = 0.0;
    res.RoadSideSysConfig.Center.orientation.x = 0.0;
    res.RoadSideSysConfig.Center.orientation.y = 0.0;
    res.RoadSideSysConfig.Center.orientation.z = 0.0;
    res.RoadSideSysConfig.Center.orientation.w = 1.0;

    // 如果是圆形区域，设置半径
    res.RoadSideSysConfig.CirculeRadius = 50.0f;   // 半径 50m

    // 如果是矩形区域，设置四个顶点
    geometry_msgs::Pose p1, p2, p3, p4;
    p1.position.x = -50.0; p1.position.y = -50.0; p1.position.z = 0.0;
    p2.position.x =  50.0; p2.position.y = -50.0; p2.position.z = 0.0;
    p3.position.x =  50.0; p3.position.y =  50.0; p3.position.z = 0.0;
    p4.position.x = -50.0; p4.position.y =  50.0; p4.position.z = 0.0;

    p1.orientation.w = p2.orientation.w = p3.orientation.w = p4.orientation.w = 1.0;
    res.RoadSideSysConfig.corners = {p1, p2, p3, p4};

    ROS_INFO("[SchedulerSystem] Loaded %lu paths from %s",
             res.MapPathInfo.size(), filename.c_str());
    return true;
}

void SchedulerSystem::vehicleInfoArrayCallback(
    const road_side_system_type::VehicleInfoOutarray::ConstPtr& msg)
{
    for (const auto& vinfo : msg->VehicleInfos)
    {
        int id = vinfo.vehicle_id;

        // 动态创建 publisher
        std::string topic_name = "/scheduler/vehicle_" + std::to_string(id);
        if (vehicle_pubs_.find(id) == vehicle_pubs_.end()) {
            vehicle_pubs_[id] = nh_.advertise<road_side_system_type::PoseWithTimeWindowArray>(topic_name, 1);
            ROS_INFO("[SchedulerDistributor] 创建Publisher: %s", topic_name.c_str());
        }
        // ✅ 用数组消息包装 vector
        road_side_system_type::PoseWithTimeWindowArray out_msg;
        out_msg.poses = vinfo.PointsWithTimeWindow;
        // 发布消息
        vehicle_pubs_[id].publish(out_msg);
    }
}


}  // namespace road_side_system
