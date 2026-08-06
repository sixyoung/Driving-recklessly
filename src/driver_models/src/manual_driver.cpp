#include "driver_models/manual_driver.h"

namespace driver_model {

ManualDriver::ManualDriver(ros::NodeHandle& nh, 
                         const driver_models_types::VehicleConfig &vehicle_config, 
                         const carla::client::World& world)
: BaseDriver(nh, vehicle_config, world),
  detection_range(50.0),
  scene_type_(vehicle_config.scene_type), 
  jerk_speed_planner_(data_pool_)
{
    road_lane_to_entry_stop_line_ = {
        {{5, -1}, carla::geom::Transform({-2.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{118, -1}, carla::geom::Transform({-2.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{50, -1}, carla::geom::Transform({-2.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{5, -2}, carla::geom::Transform({-6.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{118, -2}, carla::geom::Transform({-6.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{73, -1}, carla::geom::Transform({-6.04f, -17.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},

        {{6, 1}, carla::geom::Transform({1.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{117, 1}, carla::geom::Transform({1.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{178, 1}, carla::geom::Transform({1.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{6, 2}, carla::geom::Transform({5.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{117, 2}, carla::geom::Transform({5.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{158, 1}, carla::geom::Transform({5.96f, 17.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},

        {{8, 1}, carla::geom::Transform({17.00f, -1.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{139, 1}, carla::geom::Transform({17.00f, -1.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{140, 1}, carla::geom::Transform({17.00f, -1.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{8, 2}, carla::geom::Transform({17.00f, -5.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{139, 2}, carla::geom::Transform({17.00f, -5.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{64, 1}, carla::geom::Transform({17.00f, -5.98f, 0.00f}, {0.0f, 180.0f, 0.0f})},

        {{7, -1}, carla::geom::Transform({-17.00f, 2.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{138, -1}, carla::geom::Transform({-17.00f, 2.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{89, -1}, carla::geom::Transform({-17.00f, 2.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{7, -2}, carla::geom::Transform({-17.00f, 6.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{138, -2}, carla::geom::Transform({-17.00f, 6.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{165, -1}, carla::geom::Transform({-17.00f, 6.03f, 0.00f}, {0.0f, 0.0f, 0.0f})},
    };
                        
    road_lane_to_exit_reference_ = {
        {{118, -1}, carla::geom::Transform({-2.04f, 16.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{6, -1}, carla::geom::Transform({-2.04f, 16.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{50, -1}, carla::geom::Transform({15.88f, 2.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{8, -1}, carla::geom::Transform({15.88f, 2.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{118, -2}, carla::geom::Transform({-6.04f, 16.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{6, -2}, carla::geom::Transform({-6.04f, 16.00f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{73, -1}, carla::geom::Transform({-16.21f, -5.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{7, 2}, carla::geom::Transform({-16.21f, -5.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},

        {{117, 1}, carla::geom::Transform({1.96f, -16.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{5, 1}, carla::geom::Transform({1.96f, -16.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{178, 1}, carla::geom::Transform({-16.03f, -1.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{7, 1}, carla::geom::Transform({-16.03f, -1.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{117, 2}, carla::geom::Transform({5.96f, -16.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{5, 2}, carla::geom::Transform({5.96f, -16.00f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{158, 1}, carla::geom::Transform({16.19f, 6.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{8, -2}, carla::geom::Transform({16.19f, 6.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},

        {{139, 1}, carla::geom::Transform({-16.00f, -1.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{140, 1}, carla::geom::Transform({-2.04f, 15.95f, 0.00f}, {0.0f, 90.0f, 0.0f})},
        {{139, 2}, carla::geom::Transform({-16.00f, -5.97f, 0.00f}, {0.0f, 180.0f, 0.0f})},
        {{64, 1}, carla::geom::Transform({5.96f, -16.14f, 0.00f}, {0.0f, 270.0f, 0.0f})},

        {{138, -1}, carla::geom::Transform({16.00f, 2.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{89, -1}, carla::geom::Transform({1.96f, -15.97f, 0.00f}, {0.0f, 270.0f, 0.0f})},
        {{138, -2}, carla::geom::Transform({16.00f, 6.02f, 0.00f}, {0.0f, 0.0f, 0.0f})},
        {{165, -1}, carla::geom::Transform({-6.04f, 16.27f, 0.00f}, {0.0f, 90.0f, 0.0f})},
    };

    road_to_traffic_light_ = {
        {5, 5}, {50, 5}, {73, 5}, {118, 5},
        {6, 6}, {117, 6}, {158, 6}, {178, 6},
        {8, 8}, {139, 8}, {140, 8}, {64, 8},
        {7, 7}, {138, 7}, {89, 7}, {165, 7}
    };
    
    reset();
    Initialize();
}

ManualDriver::~ManualDriver() {
    destroy();   
}

void ManualDriver::Initialize() {
    SetAvoidTrafficLight(true);
    SetAvoidVehicle(true);
    InitializeROSResources();

    if(avoid_vehicle_) {   
        objects_sub = nh_.subscribe("/carla/objects", 10, &ManualDriver::objects_call_back, this);    
    }
    if(avoid_traffic_light_){
        traffic_lights_phases_sub = nh_.subscribe("/traffic_light/phases", 10, &ManualDriver::trafficlight_phase_call_back, this);    
    }

    // 获取路径并设置全局规划
    auto start_time = std::chrono::system_clock::now();
    while (ros::ok()) {
        global_path_ = GetPath(vehicle_config_.spawn_point.pose, vehicle_config_.goal_point.pose);
        vehicle_carla_actor_ = boost::dynamic_pointer_cast<carla::client::Vehicle>(world_.GetActor(id_));

        if (global_path_ && vehicle_carla_actor_) {
            break; 
        }

        auto current_time = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (duration > 5.0) {
            break;
        }

        ROS_WARN("%s:ID %d 初始化驾驶员模型过程中获取global_path 和 carla_actor 失败，正在重试...", role_name_.c_str(), id_);
    }

    // 设置全局路径
    if (global_path_ && vehicle_carla_actor_) {
        local_planner_->set_global_plan(global_path_);
        ROS_INFO("[MD]:车辆%s初始化完毕！", role_name_.c_str());
    }else{
        ROS_ERROR("[MD]:初始化超时(5s),车辆%s初始化失败,销毁CARLA车辆%d！", role_name_.c_str(), id_);
        local_planner_->set_reached_goal(true);
        // 请求销毁该车辆对象
        if (!destroy_carla_object(id_)) {
            ROS_ERROR("[MD]:尝试销毁CARLA车辆%d失败，请检查 DestroyObject 服务状态", id_);
        }else{
            ROS_INFO("[MD]:销毁CARLA车辆%d成功", id_);
        }
    }
}

void ManualDriver::objects_call_back(const derived_object_msgs::ObjectArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> objects_lock(objects_mutex_);
    carla_objects_.clear();
    if (msg->objects.empty()) return;
    for (const auto& obj : msg->objects) {
        carla_objects_[obj.id] = obj;
    }
}

void ManualDriver::trafficlight_phase_call_back(const road_side_system::TrafficLightPhaseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> traffic_light_lock(traffic_light_mutex_);
    traffic_light_map_.clear();
    if (msg->phases.empty()) return;
    for (const auto& phase : msg->phases) {
        traffic_light_map_[phase.road_id] = phase;
    }
}

void ManualDriver::run_step() {
    std::lock_guard<std::mutex> objects_lock(objects_mutex_);
    std::lock_guard<std::mutex> traffic_light_lock(traffic_light_mutex_);

    vehicle_carla_actor_ = boost::dynamic_pointer_cast<carla::client::Vehicle>(world_.GetActor(id_));
    
    if (is_destroyed_ || !vehicle_carla_actor_ || !local_planner_->get_odometry_received_flag()||
        !global_path_ || local_planner_->get_reached_goal_flag()) {
        return;
    }
 
    // lane_change(carla::geom::Location(-60, 2, 0));

    // 更新车辆状态
    update_infomation(vehicle_carla_actor_);

    // 感知环境
    PerceiveEnvironment();

    if(scene_type_ == "tl_intersection"){
        handle_trafficlight();
    }

    // 目标速度
    const double target_speed_planner = PlanSpeed();
    // 发布控制
    carla_msgs::CarlaEgoVehicleControl control_msg;
    control_msg = local_planner_->run_step(true);
    control_cmd_pub_.publish(control_msg);
}

/**
 * @brief 更新车辆信息、状态和转向灯
 * @param is_active 车辆是否激活车灯系统
 * @param vehicle 车辆对象
 */
void ManualDriver::update_infomation(carla::SharedPtr<carla::client::Vehicle> vehicle) {
    // 1. 获取车辆基本信息
    uodate_vehicle_info();
    // 2. 更新车辆所在路径点
    update_path_waypoint();
    // 3. 确定车辆行驶状态
    update_intention();
    // 5. 根据状态更新转向灯
    update_vehicle_light(true, vehicle);
}

void ManualDriver::PerceiveEnvironment() {
    obstacles_.clear();
    inter_obs_.clear();
    const double DIRECTION_THRESHOLD_DEG = 20.0; 
    const double COS_THRESHOLD = std::cos(DIRECTION_THRESHOLD_DEG * M_PI / 180.0);

    if (carla_objects_.empty() || ego_path.empty() || !ego_wpt) {
        return;
    }
    double closest_follow_dist = std::numeric_limits<double>::max();
    double closest_cross_dist = std::numeric_limits<double>::max();
    std::optional<ObstacleIDM> nearest_follow_obstacle = std::nullopt;
    std::optional<ObstacleIDM> nearest_cross_obstacle = std::nullopt;

    // 遍历所有检测到的物体
    for (const auto& [obj_id, obj] : carla_objects_) {
        // 跳过自车
        if (obj.id == id_) {
            continue;
        }
        // 检查是否为车辆类型
        if (std::find(OBJECT_VEHICLE_CLASSIFICATION.begin(), OBJECT_VEHICLE_CLASSIFICATION.end(),
                    obj.classification) == OBJECT_VEHICLE_CLASSIFICATION.end()) {
            continue;
        }
        // 检查距离
        const double distance = distance_vehicle(ego_pose.position, obj.pose.position);
        if (distance > detection_range) {
            continue;
        }
        carla::SharedPtr<carla::client::Vehicle> tar_vehicle = boost::dynamic_pointer_cast<carla::client::Vehicle>(world_.GetActor(obj.id));
        if (!tar_vehicle) {
            continue;
        }

        std::string target_role_name = "unknown";
        for (const auto& attr : tar_vehicle->GetAttributes()) {
            if (attr.GetId() == "role_name") {
                target_role_name = attr.GetValue();
                break;
            }
        }

        const carla::geom::Transform tar_transform = ros_pose_to_carla_transform(obj.pose);
        // 排除不在车辆前方的目标车辆
        if (!is_within_distance_and_angle(ego_transform, tar_transform, detection_range, 0.0, 90.0)) {
            continue;
        }

        const int32_t tar_id = static_cast<int32_t>(obj.id);
        const carla::geom::Vector3D tar_forward = tar_transform.GetForwardVector().MakeUnitVector();
        carla::SharedPtr<carla::client::Waypoint> tar_wpt = nullptr;      

        // 获取目标车辆的waypoint
        tar_wpt = find_match_waypoint(tar_id, tar_transform, COS_THRESHOLD, 0.5, last_valid_tar_wpts_);

        if (!tar_wpt) {
            ROS_WARN("目标车辆 ID=%d 没有符合方向要求的有效waypoint", tar_id);
            continue;
        }

        // 目标车辆的尺寸和corners和多边形
        double tar_length_ = obj.shape.dimensions[0];
        double tar_width_ = obj.shape.dimensions[1];
        std::vector<carla::geom::Location> tar_corners = GetRectangleCorners(tar_transform, tar_length_, tar_width_);

        Polygon tar_polygon;
        for (const auto& pt : tar_corners) {
            bg::append(tar_polygon.outer(), Point(pt.x, pt.y));
        }
        bg::correct(tar_polygon);

        // 目标车辆的速度信息
        carla::geom::Vector3D tar_speed(obj.twist.linear.x, -obj.twist.linear.y, obj.twist.linear.z);
        double tar_speed_mps = std::sqrt(tar_speed.x * tar_speed.x + tar_speed.y * tar_speed.y + tar_speed.z * tar_speed.z);

        // 获取目标车辆的灯光状态
        int tar_light_state = static_cast<int>(GetCurrentLightState(tar_vehicle));

        // 预测目标车辆的未来路径
        std::vector<carla::geom::Transform> target_predicted_path_;
        if (!(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
            !(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("🚘 [预测] 目标车辆无转向灯，预测直行路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_id, "straight", 1.0, 50);
        } else if ((tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
                !(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("↩️ [预测] 目标车辆打左转灯，预测左转路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_id, "left",  1.0, 50);
        } else if (!(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
                (tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("↪️ [预测] 目标车辆打右转灯，预测右转路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_id, "right",  1.0, 50);
        } else {
            ROS_DEBUG_STREAM("⚠️ [预测] 双闪或未知转向状态，默认预测直行");
            target_predicted_path_ = get_target_path(tar_wpt, tar_id, "straight",  1.0, 50);
        }

        // 提取以车头为起点的有效路径
        std::vector<carla::geom::Transform> tar_front_path;
        std::vector<double> tar_s_list;
        get_forward_path(tar_transform, tar_length_, target_predicted_path_, tar_front_path, tar_s_list, tar_polygon);

        // 插值并存入结果
        std::vector<carla::geom::Transform> tar_interp_path;
        std::vector<double> tar_interp_s_list;
        std::vector<double> tar_limit_v_list;
        interpolate_path(tar_front_path, tar_s_list, 1.0, 8.0, tar_interp_path, tar_interp_s_list, tar_limit_v_list);
        Polygon tar_path_poly = PathToPolygon(tar_interp_path, tar_width_ / 2.0);

        // 如果目标车辆在自车路径上
        if (is_box_intersect_path(ego_path_poly, tar_transform, tar_length_, tar_width_)){
            // 检查相互阻挡情况
            if (is_box_intersect_path(tar_path_poly, ego_transform, ego_length, ego_width) && 
                is_box_intersect_path(ego_path_poly, tar_transform, tar_length_, tar_width_)) {
                ROS_ERROR_STREAM("[相互阻挡路径] 自车" << vehicle_config_.carla_id << "目标车ID: " << obj.id);
                draw_ego_box();
            }

            double min_dist = std::numeric_limits<double>::max();
            int nearest_wp_idx = 1;

            auto proj1 = project_point_to_path(tar_transform.location, interp_path, s_list, 0.25);
            if (proj1.has_value()) {
                auto [s1_, nearest_candidate_idx] = *proj1;
                if (nearest_candidate_idx >= 1) {
                    nearest_wp_idx = nearest_candidate_idx;
                }
            } else {
                ROS_WARN("%s:[TargetNM=%s]Failed to project tar_transform.location to path.", role_name_.c_str(), target_role_name.c_str());
            }

            // Step 2: 提取该最近路径点的朝向
            carla::geom::Vector3D path_forward = interp_path[nearest_wp_idx].GetForwardVector().MakeUnitVector();
            double direction_dot = carla::geom::Math::Dot(tar_forward, path_forward);

            // Step 3: 判断方向是否一致
            if (direction_dot >= COS_THRESHOLD) {
                int out_nearest_index = 1;
                int out_candidate_index = -1;
                carla::geom::Vector3D offset = 0.5 * tar_length_ * tar_forward;
                carla::geom::Location tar_rear_loc(
                    tar_transform.location.x - offset.x,
                    tar_transform.location.y - offset.y,
                    tar_transform.location.z - offset.z);
                auto proj2 = project_point_to_path(tar_rear_loc, interp_path, s_list, 0.25);
                if (proj2.has_value()) {
                    out_candidate_index = proj2->second;
                    // ✅ 可安全使用 s2_ 和 out_candidate_index
                } else {
                    ROS_WARN("%s:[TargetNM=%s]Failed to project tar_rear_loc to path.", role_name_.c_str(), target_role_name.c_str());

                }
                if (out_candidate_index >= 1) {
                    for (int i = out_candidate_index; i >= 1; --i) {
                        const auto& tf = interp_path[i];
                        const auto& center = tf.location;
                        auto right_vec = tf.GetRightVector();

                        // 左右偏移点  0.5 * ego_width;
                        carla::geom::Location left_loc = center + carla::geom::Location(-0.5 * ego_width * right_vec.x, -0.5 * ego_width * right_vec.y, 0.0);
                        carla::geom::Location right_loc = center + carla::geom::Location(0.5 * ego_width * right_vec.x, 0.5 * ego_width * right_vec.y, 0.0);

                        Point left_pt(left_loc.x, left_loc.y);
                        Point right_pt(right_loc.x, right_loc.y);

                        // 两侧都不在重叠区域
                        if (!bg::covered_by(left_pt, tar_polygon) && !bg::covered_by(right_pt, tar_polygon)) {
                            out_nearest_index = i;
                            break;
                        }
                    }
                }
                // Step 1: 计算 s_ini，使用 rear_nearest_idx
                double s_ini = s_list[out_nearest_index];

                // Step 2: 计算 s_end = s_ini + v * Δt
                double duration = 10.0;  // 预测时间窗口
                double v_tar = tar_speed_mps;  // 从感知消息中获取
                double t_ini = 0.0;
                double t_end = t_ini + duration;
                double s_end = s_ini + v_tar * duration;

                // Step 3: 构造 Obstacle 对象（注意宽度/长度不要搞反）
                if (s_ini >= 0 && s_ini < closest_follow_dist) {
                    ObstacleIDM follow_obstacle(
                        t_ini, t_end,
                        s_ini, s_end,
                        v_tar, target_role_name  // margin_s, 主动将障碍物提前 margin_s
                    );
                    nearest_follow_obstacle = follow_obstacle;
                    closest_follow_dist = s_ini;  // 更新最近距离
                }
            } else {
                std::vector<carla::geom::Transform> merged_point;

                // 计算 3 个关键点位置（尾部 / 中心 / 车头）
                carla::geom::Location tail_loc = tar_transform.location - static_cast<carla::geom::Location> (0.5 * tar_length_ * tar_forward);
                carla::geom::Location center_loc = tar_transform.location;
                carla::geom::Location head_loc = tar_transform.location + static_cast<carla::geom::Location> (0.5 * tar_length_ * tar_forward);

                // 构造 Transform 对象
                carla::geom::Transform tail_tf(tail_loc, tar_transform.rotation);
                carla::geom::Transform center_tf(center_loc, tar_transform.rotation);
                carla::geom::Transform head_tf(head_loc, tar_transform.rotation);

                // 按顺序加入 merged_point
                merged_point.push_back(tail_tf);
                merged_point.push_back(center_tf);
                merged_point.push_back(head_tf);

                // 拼接上原始路径
                merged_point.insert(merged_point.end(), tar_interp_path.begin(), tar_interp_path.end());
                Polygon merged_polygon = PathToPolygon(merged_point, tar_width_ / 2.0);
                auto [intersection, inter_points] = CheckPolygonIntersection(ego_path_poly, merged_polygon);
                
                if(intersection){
                    bg::model::ring<Point> ring_input;
                    ring_input.assign(inter_points.begin(), inter_points.end());  // 将点赋值给合法 ring 类型

                    Polygon overlap_poly;
                    bg::convex_hull(ring_input, overlap_poly);  // 使用合法 Geometry 类型
                    const double half_width = ego_width * 0.5;
                    // DrawPolygon(overlap_poly, *debug_helper_, 0.5f, 0.1f, 0.1f, {255, 0, 0});
                    // DrawPolygon(merged_polygon, *debug_helper_, 0.5f, 0.1f, 0.1f, {0, 0, 255});

                    int ego_entry_index = -1;
                    int entry_candidate_index = -1;
                    // === Step 1: 先顺序查找 entry 候选点 ===
                    for (size_t i = 0; i + 1 < interp_path.size(); ++i) {
                        const auto& curr_loc = interp_path[i].location;
                        const auto& next_loc = interp_path[i + 1].location;

                        Point curr_pt(curr_loc.x, curr_loc.y);
                        Point next_pt(next_loc.x, next_loc.y);

                        if (!bg::covered_by(curr_pt, overlap_poly) && bg::covered_by(next_pt, overlap_poly)) {
                            entry_candidate_index = static_cast<int>(i);
                            break;
                        }
                    }

                    // === Step 2: 回退查找，验证两侧点是否都不在 overlap 区域 ===
                    if (entry_candidate_index >= 0) {
                        for (int i = entry_candidate_index; i >= 0; --i) {
                            const auto& tf = interp_path[i];
                            const auto& center = tf.location;
                            auto right_vec = tf.GetRightVector();

                            // 左右偏移点
                            carla::geom::Location left_loc = center + carla::geom::Location(-half_width * right_vec.x, -half_width * right_vec.y, 0.0);
                            carla::geom::Location right_loc = center + carla::geom::Location(half_width * right_vec.x, half_width * right_vec.y, 0.0);

                            Point left_pt(left_loc.x, left_loc.y);
                            Point right_pt(right_loc.x, right_loc.y);

                            // 两侧都不在重叠区域
                            if (!bg::covered_by(left_pt, overlap_poly) && !bg::covered_by(right_pt, overlap_poly)) {
                                ego_entry_index = i;
                                ROS_DEBUG("%s: ✅ Valid ego_entry_index = %d", role_name_.c_str(), ego_entry_index);
                                break;
                            }
                        }
                    }
                    if (ego_entry_index >= 0 && ego_entry_index < interp_path.size()) {
                        const auto& entry_tf = interp_path[ego_entry_index];

                        carla::geom::Location loc = entry_tf.location;
                        float life_time = 0.3f;  // 每帧刷新
                        float size = 0.2f;

                        // debug_helper_->DrawPoint(loc, size, {0, 0 , 255}, life_time, false);
                    }
                    double s_ini = s_list[ego_entry_index];
                    double s_end = s_ini;

                    // 时间段计算
                    double duration = 10.0;
                    double t_ini = 0.0;
                    double t_end = t_ini + duration;

                    if (s_ini >= 0 && s_ini < closest_cross_dist) {
                        ObstacleIDM cross_obstacle(
                            t_ini, t_end,
                            s_ini, s_end,
                            0.0, target_role_name 
                        );
                        nearest_cross_obstacle = cross_obstacle;
                        closest_cross_dist = s_ini;  // 更新最近距离
                    } 
                }else{
                    ROS_ERROR("[%s] ❌ 路径无交集，交叉口障碍构建失败！！！", role_name_.c_str());
                }
            }
        }else{
            if (is_box_intersect_path(tar_path_poly, ego_transform, ego_length, ego_width)) {
                continue;
            }
            // 检查路径是否相交 
            auto [has_intersection, points] = CheckPolygonIntersection(ego_path_poly, tar_path_poly);
            if(has_intersection){
                if(scene_type_ == "tl_intersection"){
                    obstacle obs;
                    obs.id = tar_id;
                    obs.transform = tar_transform;
                    obs.corners = tar_corners;
                    obs.polygon = tar_polygon;
                    obs.speed = tar_speed_mps;
                    obs.velocity_vec = tar_speed;
                    obs.length = tar_length_;
                    obs.width = tar_width_;
                    obs.predicted_path = target_predicted_path_;
                    obs.interp_path = tar_interp_path;
                    obs.waypoint = tar_wpt;
                    obs.overlap_points = points;
                    obs.interp_s_list = tar_interp_s_list;
                    inter_obs_.emplace_back(std::move(obs)); 
                }else if(scene_type_ == "untl_intersection"){
                    // 构建冲突区域overlap_poly
                    bg::model::ring<Point> ring_input;
                    ring_input.assign(points.begin(), points.end());  // 将点赋值给合法 ring 类型

                    Polygon overlap_poly;
                    bg::convex_hull(ring_input, overlap_poly);  // 使用合法 Geometry 类型

                    const auto& hull_pts = overlap_poly.outer();  // 多边形外圈点集
                    size_t num_pts = hull_pts.size();

                    // DrawPolygon(overlap_poly, *debug_helper_, 0.5f, 0.1f, 0.1f, {255, 0, 0});


                    // interp_path 遍历 + ego_entry_index
                    const double half_width = ego_width * 0.5;

                    int ego_entry_index = -1;
                    int entry_candidate_index = -1;
                    // === Step 1: 先顺序查找 entry 候选点 ===
                    for (size_t i = 0; i + 1 < interp_path.size(); ++i) {
                        const auto& curr_loc = interp_path[i].location;
                        const auto& next_loc = interp_path[i + 1].location;

                        Point curr_pt(curr_loc.x, curr_loc.y);
                        Point next_pt(next_loc.x, next_loc.y);

                        if (!bg::covered_by(curr_pt, overlap_poly) && bg::covered_by(next_pt, overlap_poly)) {
                            entry_candidate_index = static_cast<int>(i);
                            break;
                        }
                    }

                    // === Step 2: 回退查找，验证两侧点是否都不在 overlap 区域 ===
                    if (entry_candidate_index >= 0) {
                        for (int i = entry_candidate_index; i >= 0; --i) {
                            const auto& tf = interp_path[i];
                            const auto& center = tf.location;
                            auto right_vec = tf.GetRightVector();

                            // 左右偏移点
                            carla::geom::Location left_loc = center + carla::geom::Location(-half_width * right_vec.x, -half_width * right_vec.y, 0.0);
                            carla::geom::Location right_loc = center + carla::geom::Location(half_width * right_vec.x, half_width * right_vec.y, 0.0);

                            Point left_pt(left_loc.x, left_loc.y);
                            Point right_pt(right_loc.x, right_loc.y);

                            // 两侧都不在重叠区域
                            if (!bg::covered_by(left_pt, overlap_poly) && !bg::covered_by(right_pt, overlap_poly)) {
                                ego_entry_index = i;
                                ROS_DEBUG("%s: ✅ Valid ego_entry_index = %d", role_name_.c_str(), ego_entry_index);
                                break;
                            }
                        }
                    }

                    if (ego_entry_index >= 0 && ego_entry_index < interp_path.size()) {
                        const auto& entry_tf = interp_path[ego_entry_index];

                        carla::geom::Location loc = entry_tf.location;
                        float life_time = 0.3f;  // 每帧刷新
                        float size = 0.2f;

                        // debug_helper_->DrawPoint(loc, size, {255, 0 , 0}, life_time, false);
                    }

                    // tar_interp_path 遍历 + tar_entry_index + tar_out_index
                    int tar_entry_index = -1;
                    int tar_entry_candidate = -1;
                    // === Step 1: 从前往后找候选点（从区域外进入区域内）===
                    for (size_t i = 0; i + 1 < tar_interp_path.size(); ++i) {
                        const auto& curr_loc = tar_interp_path[i].location;
                        const auto& next_loc = tar_interp_path[i + 1].location;

                        Point curr_pt(curr_loc.x, curr_loc.y);
                        Point next_pt(next_loc.x, next_loc.y);

                        if (!bg::covered_by(curr_pt, overlap_poly) && bg::covered_by(next_pt, overlap_poly)) {
                            tar_entry_candidate = static_cast<int>(i);
                            break;
                        }
                    }

                    // === Step 2: 从候选点回退，验证左右两点是否都不在区域内 ===
                    if (tar_entry_candidate >= 0) {
                        for (int i = tar_entry_candidate; i >= 0; --i) {
                            const auto& tf = tar_interp_path[i];
                            const auto& center = tf.location;
                            auto right_vec = tf.GetRightVector();

                            carla::geom::Location left_loc = center + carla::geom::Location(-half_width * right_vec.x, -half_width * right_vec.y, 0.0);
                            carla::geom::Location right_loc = center + carla::geom::Location(half_width * right_vec.x, half_width * right_vec.y, 0.0);

                            Point left_pt(left_loc.x, left_loc.y);
                            Point right_pt(right_loc.x, right_loc.y);

                            if (!bg::covered_by(left_pt, overlap_poly) && !bg::covered_by(right_pt, overlap_poly)) {
                                tar_entry_index = i;
                                ROS_DEBUG("✅ Target vehicle entry point found: tar_entry_index = %d", tar_entry_index);
                                break;
                            }
                        }
                    }

                    //1.先无障碍规划一下，获取自车到达交叠区域入口点的时间;
                    //2.当自车到达交叠区域入口点的 s >= 10m :
                        //2.1如果自车到达时间小于目标车辆到达时间，自车继续行驶，否则自车让行
                    //3.当自车到达交叠区域入口点的 s < 10m:
                        //3.1如果目标车辆到达交叠区域入口点的 s >= 10m，如果自车到达时间小于目标车辆到达时间，自车继续行驶，否则自车让行
                        //3,2如果目标车辆到达交叠区域入口点的 s < 10m，如果自车到达时间和目标车辆到达时间差值的绝对值 >= 3s，自车继续行驶，否则谁近谁先行

                    if (tar_entry_index >= 0 && ego_entry_index >= 0) {
                        std::vector<speed_planner::Obstacle> empty_obstacles;
                        auto result = EstimateTravelTime(empty_obstacles, s_list, v_limit_list, false);
                        double s_ego_to_entry = s_list[ego_entry_index];
                        double s_target_to_entry = tar_interp_s_list[tar_entry_index];
                        double t_ego_to_entry = std::get<1>(result).time[ego_entry_index];

                        // auto result2 = EstimateTravelTime(empty_obstacles, s_list, v_limit_list, true);

                        double t_tar_to_entry = s_target_to_entry / std::max(tar_speed_mps, 1e-4);  // 目标车匀速时间
                        double v_ego = current_speed_;
                        double v_tar = tar_speed_mps;
        
                        // ✅ 情况 1：
                        if (s_ego_to_entry < 5.0) {
                            if(s_target_to_entry < 5.0){
                                if (s_ego_to_entry < s_target_to_entry) {
                                    continue;
                                } else if (s_ego_to_entry == s_target_to_entry){
                                    if(v_ego < 1e-2 && v_tar < 1e-2 && t_ego_to_entry < t_tar_to_entry){
                                        continue;
                                    }
                                }
                            }else if(s_ego_to_entry >= 5.0 && s_target_to_entry <= 20.0){
                                if (t_ego_to_entry - t_tar_to_entry < -2.0) {
                                    continue;
                                }
                            }else{
                                    continue;
                            }                                             
                        }
                        // ✅ 情况 2：
                        else if (s_ego_to_entry >= 5.0) {
                            if(s_target_to_entry >= 20.0){
                                continue;
                            }else if (s_target_to_entry >= 5.0) {
                                if (t_ego_to_entry - t_tar_to_entry < -3.0) {
                                    continue;
                                }
                            }
                        }
                        // 让行
                        ROS_WARN("%s: ⛔️ 让行 [TargetNM=%s] (s_ego=%.2f, s_tar=%.2f, v_ego=%.2f, v_tar=%.2f, t_ego=%.2f, t_tar=%.2f)", 
                                role_name_.c_str(), target_role_name.c_str(),
                                s_ego_to_entry, s_target_to_entry, v_ego, v_tar, t_ego_to_entry, t_tar_to_entry);

                        double s_ini = s_list[ego_entry_index]; 
                        double s_end = s_ini;
                        double t_ini = 0.0;
                        double t_end = t_tar_to_entry + 10.0;
                        ObstacleIDM inter_obstacle(
                            t_ini, t_end,
                            s_ini, s_end,
                            0.0, target_role_name 
                        );
                        closest_obstacle_info_.update(inter_obstacle, 0);
                        obstacles_.emplace_back(inter_obstacle);
                    }  
                }
            }else{
                continue;
            }
        }
    }
    // 将最佳跟随和交叉障碍物添加到障碍物池
    if (nearest_follow_obstacle.has_value()) {
        closest_obstacle_info_.update(*nearest_follow_obstacle, 1);
        obstacles_.emplace_back(*nearest_follow_obstacle);
        ROS_WARN("%s: 跟随障碍物 s=%f， [%s]", role_name_.c_str(), nearest_follow_obstacle->s_ini, nearest_follow_obstacle->name.c_str());
    }

    if (nearest_cross_obstacle.has_value()) {
        closest_obstacle_info_.update(*nearest_cross_obstacle, 0);
        obstacles_.emplace_back(*nearest_cross_obstacle);
        ROS_WARN("%s: 目标障碍 s=%f， [%s]", role_name_.c_str(), nearest_cross_obstacle->s_ini, nearest_cross_obstacle->name.c_str());

    }
}

std::tuple<bool, BaseSolver::OutputInfo> ManualDriver::EstimateTravelTime(
const std::vector<speed_planner::Obstacle>& obstacles,
const std::vector<double>& s_list,
const std::vector<double>& v_limit_list,
bool should_output_results){
    // Step 0: 重置数据池
    data_pool_.reset();

    // Step 1: 设置规划输入参数
    data_pool_.positions_ = s_list;
    data_pool_.max_velocities_ = v_limit_list;
    data_pool_.obs_ = obstacles;
    data_pool_.ds_ = 1.0;
    data_pool_.max_acc_ = 5.0;
    data_pool_.min_acc_ = -5.0;
    data_pool_.max_jerk_ = 3.0;
    data_pool_.min_jerk_ = -3.0;
    data_pool_.v0_ = current_speed_;  // km/h → m/s
    data_pool_.a0_ = 0.0;
    
    // Step 4: 调用速度规划器
    auto result = jerk_speed_planner_.plan(should_output_results);
    return result;
}


/**
 * @brief 更新车辆基本信息、路径点和驾驶员意图
 */
void ManualDriver::uodate_vehicle_info() {
    closest_obstacle_info_ = {};
    traffic_light_obs.reset();

    ego_length = vehicle_carla_actor_->GetBoundingBox().extent.x*2.0;
    ego_width = vehicle_carla_actor_->GetBoundingBox().extent.y*2.0;
    ego_pose = local_planner_->get_current_pose_();
    ego_transform = ros_pose_to_carla_transform(ego_pose);
    ego_forward = ego_transform.GetForwardVector().MakeUnitVector();
    current_speed_ = local_planner_->get_current_speed_();

    ego_corners = GetRectangleCorners(ego_transform, ego_length, ego_width);
    ego_polygon.outer().clear();

    for (const auto& pt : ego_corners) {
        bg::append(ego_polygon.outer(), Point(pt.x, pt.y));
    }
    bg::correct(ego_polygon);

    // 如果 role_name 是 1、100 或 4，则绘制红色框
    if (role_name_ == "hero1000" || role_name_ == "hero100" || role_name_ == "hero454") {
        ROS_INFO("[DrawPolygon] ego_polygon 点数: %zu", ego_polygon.outer().size());
        for (size_t i = 0; i < ego_polygon.outer().size(); ++i) {
            const auto &pt = ego_polygon.outer()[i];
            ROS_INFO("  Point[%zu]: (%.2f, %.2f)", i, pt.x(), pt.y());
        }
        ROS_INFO("[DrawPolygon] 绘制红色框：role_name=%s", role_name_.c_str());
        DrawPolygon(ego_polygon, *debug_helper_, 1.5f, 0.3f, 1.0f, {255, 0, 0});
        ROS_INFO("[DrawPolygon] 已完成绘制。");
    }

    if (vehicle_carla_actor_) {
        // 获取车顶位置（z 增加一部分，防止文字被车体遮挡）
        auto location = vehicle_carla_actor_->GetTransform().location;
        location.z += 2.5;  // 偏移到车顶上方

        // 设置显示文本为 role_name_ 或 Carla ID
        std::string text = role_name_;  // 或者 std::to_string(vehicle_config_.carla_id);

        debug_helper_->DrawString(
            location,
            text,
            false,  // draw_shadow：不绘制阴影
            carla::client::DebugHelper::Color{0U, 0U, 255U},  // 蓝色
            0.1f,   // 显示时间 0.1 秒，适合频繁更新
            false   // 不持久显示（下一帧刷新）
        );
    }
    // carla::geom::Vector3D box_location(-20.0f, 2.0f, 1.5f);   // 中心位置
    // carla::geom::Vector3D box_extent(0.5f, 2.0f, 1.0f);        // 半长、半宽、半高
    // carla::geom::Rotation box_rotation(0.0f, 0.0f, 0.0f);      // pitch, yaw, roll (单位：度)

    // carla::geom::Transform box_transform(box_location, box_rotation);

    // debug_helper_->DrawBox(
    //     carla::geom::BoundingBox(box_location, box_extent),
    //     box_rotation,
    //     0.2f,   // 线条厚度
    //     carla::client::DebugHelper::Color{255U, 0, 0U},  // 绿色
    //     0.1f     // 显示时间（秒）
    // );
}
        
/**
 * @brief 更新车辆所在路径点、局部路径和多边形以及车头位置在局部路径上的投影
 */
void ManualDriver::update_path_waypoint() {
    const double cos_threshold = std::cos(20.0 * M_PI / 180.0);  // 合理方向差余弦阈值

    ego_wpt = find_match_waypoint(id_, ego_transform, cos_threshold, 1.0, last_valid_ego_wpts_);

    ego_path = get_ego_path(50);

    s_list.clear(); 
    v_limit_list.clear();
    interp_path.clear();
    
    std::vector<carla::geom::Transform> ego_front_path;
    std::vector<double> ego_s_list;
    
    // 第一步：提取以车头为起点的有效路径
    get_forward_path(ego_transform, ego_length, ego_path, ego_front_path, ego_s_list, ego_polygon);

    // 第二步：插值并存入结果
    interpolate_path(ego_front_path, ego_s_list, 1.0, vehicle_config_.cruise_speed.data, interp_path, s_list, v_limit_list);
    
    // 将路径转换为多边形，用于碰撞检测
    ego_path_poly = PathToPolygon(interp_path, ego_width/2.0);


    auto path = PathToPolygon(interp_path, ego_width/2.0);
    if (!bg::is_valid(path)) {
        std::ostringstream oss;
        oss << "[" << role_name_ << "]: Invalid polygon, path points: ";
        const auto& ring = path.outer();
        for (size_t i = 0; i < ring.size(); ++i) {
            oss << "(" << std::fixed << std::setprecision(2) 
                << ring[i].x() << ", " << ring[i].y() << ")";
            if (i != ring.size() - 1) oss << ", ";
        }
        ROS_WARN("%s", oss.str().c_str());

        ROS_WARN("[%s] front_path: x,y,right_x,right_y", role_name_.c_str());

        for (const auto& tf : ego_front_path) {
            auto right = tf.GetRightVector().MakeSafeUnitVector(1e-6f);
            const auto& loc = tf.location;

            // 打印为 CSV 格式：x,y,right_x,right_y
            ROS_WARN("[%s] %.3f,%.3f,%.6f,%.6f", 
                    role_name_.c_str(), loc.x, loc.y, right.x, right.y);
        }

        const auto& ring1 = ego_polygon.outer();
        ROS_WARN("[%s] ego_polygon contains %zu points:", role_name_.c_str(), ring1.size());

        for (size_t i = 0; i < ring1.size(); ++i) {
            const auto& pt = ring1[i];
            ROS_WARN("[%s]   pt[%zu] = (%.3f, %.3f)", role_name_.c_str(), i, pt.x(), pt.y());
        }
    }
}

/**
 * @brief 更新驾驶员意图
 */
void ManualDriver::update_intention() {
    auto road_option_wp = get_road_option();
    if (!road_option_wp) {
        ROS_ERROR_THROTTLE(2.0, "%s: road_option_wp is none.", role_name_.c_str());
        return;
    }
    // 根据路径选项更新状态
    switch (road_option_wp->second) {
        case RoadOption::LANEFOLLOW:
                driver_intention_ = DrivingIntention::NAVIGATING;
            break;
        case RoadOption::LEFT:
                driver_intention_ = DrivingIntention::TURNING_LEFT;
            break;
        case RoadOption::RIGHT:
                driver_intention_ = DrivingIntention::TURNING_RIGHT;
            break;
        case RoadOption::STRAIGHT:
                driver_intention_ = DrivingIntention::GOING_STRAIGHT;
            break;
        case RoadOption::CHANGELANELEFT:
            driver_intention_ = DrivingIntention::CHANGELANELEFT;
            break;
        case RoadOption::CHANGELANERIGHT:
            driver_intention_ = DrivingIntention::CHANGELANERIGHT;
            break;
        case RoadOption::VOID:
                driver_intention_ = DrivingIntention::VOID;
            break;
        default:
            ROS_WARN("%s: Unknown road option.", role_name_.c_str());
            break;
    }
}

/**
 * @brief 根据驾驶员状态更新车辆灯光
 * @param vehicle 车辆对象
 */
void ManualDriver::update_vehicle_light(bool is_active, carla::SharedPtr<carla::client::Vehicle> vehicle) {
    if(!is_active || !vehicle){
        return;
    }
    switch (driver_intention_) {
        case DrivingIntention::TURNING_LEFT:
            TurnOnLeft(vehicle);
            TurnOffRight(vehicle);
            break;
        case DrivingIntention::TURNING_RIGHT:
            TurnOnRight(vehicle);
            TurnOffLeft(vehicle);
            break;
        case DrivingIntention::CHANGELANELEFT:
            TurnOnLeft(vehicle);
            TurnOffRight(vehicle);
            break;
        case DrivingIntention::CHANGELANERIGHT:
            TurnOnRight(vehicle);
            TurnOffLeft(vehicle);
            break;
        default:
            TurnOffLeft(vehicle);
            TurnOffRight(vehicle);
            break;
    }
}

void ManualDriver::handle_trafficlight() {
    // 🚦 读取红绿灯信息并构建虚拟障碍物
    int road_id = ego_wpt->GetRoadId();
    int lane_id = ego_wpt->GetLaneId();
    auto key = std::make_pair(road_id, lane_id);
    auto entry_line = road_lane_to_entry_stop_line_.find(key);
    if (entry_line == road_lane_to_entry_stop_line_.end()) return;
    const auto& entry_line_loc = entry_line->second.location;

    double dist_to_entry = carla::geom::Math::Distance(ego_transform.location, entry_line_loc);

    auto entry_line_proj = project_point_to_path(entry_line_loc, interp_path, s_list, 1e-2);
    if (!entry_line_proj.has_value()) {
        ROS_ERROR("%s:Failed to project stop_line_loc onto path.", role_name_.c_str());
    }
    double entry_line_s = entry_line_proj->first;

    int light_id = -1;  // 默认无效值
    auto road_it = road_to_traffic_light_.find(road_id);
    if (road_it != road_to_traffic_light_.end()) {
        light_id = road_it->second;
    }else { 
        ROS_WARN("%s: 未找到 road_id=%d 的交通灯信息，无法构造交通灯障碍物", role_name_.c_str(), road_id);
    }
    auto it = traffic_light_map_.find(light_id);
    if (!traffic_light_map_.empty() && it != traffic_light_map_.end()){
        // 🚦 判断信号灯状态
        const auto& tl_info = it->second;
        if (tl_info.state == "Red") {
            // 🚧 构造静态障碍物
            double t_ini = 0.0;
            double t_end = 0.0; 
            if (tl_info.remaining_time < 10.0) {
                t_end = tl_info.remaining_time; 
            }else {
                t_end = 10.0;  
            }
            double s_ini = entry_line_s;
            double s_end = entry_line_s; 

            traffic_light_obs = ObstacleIDM(
                t_ini, t_end,
                s_ini, s_end,
                0.0, "Red");
            obstacles_.emplace_back(*traffic_light_obs);
            closest_obstacle_info_.update(*traffic_light_obs, 0);
            // ROS_INFO("%s:🔴构造红灯静态障碍物:s=%.2f,t=%.2f", role_name_.c_str(), entry_line_s, tl_info.remaining_time);
        } else if (tl_info.state == "Yellow") {
            // 🚧 构造静态障碍物
            double t_ini = 0.0;
            double t_end = 10.0;
            double s_ini = entry_line_s;
            double s_end = entry_line_s; 

            traffic_light_obs = ObstacleIDM(
                t_ini, t_end,
                s_ini, s_end,
                0.0, "Yellow");
            obstacles_.emplace_back(*traffic_light_obs);
            closest_obstacle_info_.update(*traffic_light_obs, 0);
            // ROS_INFO("%s:🟡构造黄灯静态障碍物:s=%.2f,t=%.2f", role_name_.c_str(), entry_line_s, tl_info.remaining_time);
        }else if (tl_info.state == "Green") {
            // 🚧 构造静态障碍物
            double t_ini = 0.0;
            double t_end = 10.0;
            double s_ini = entry_line_s;
            double s_end = entry_line_s; 

            traffic_light_obs = ObstacleIDM(
                t_ini, t_end,
                s_ini, s_end,
                0.0, "Green");
            // ROS_INFO("%s:🟡构造黄灯静态障碍物:s=%.2f,t=%.2f", role_name_.c_str(), entry_line_s, tl_info.remaining_time);
        }
    } 
}

/**
 * @brief 确定车辆的行驶选项
 * @return 返回路径点和行驶选项的pair，如果无效则返回nullptr和VOID
 */
std::shared_ptr<std::pair<geometry_msgs::Pose, local_planner::RoadOption>> ManualDriver::get_road_option() {
    // 检查是否在路口
    bool is_junction = ego_wpt && ego_wpt->IsJunction();
    if (is_junction) {
        // 在路口时，检查下一个路径点 
        auto next_road_wp = local_planner_->get_incoming_waypoint_and_direction(2);
        if (next_road_wp) {
            auto next_transform = ros_pose_to_carla_transform(next_road_wp->first);
            auto next_wpt = world_.GetMap()->GetWaypoint(next_transform.location);
            bool next_is_junction = next_wpt && next_wpt->IsJunction();
            
            if (!next_is_junction) {
                // 如果下一个点不是路口，往回看
                return local_planner_->get_incoming_waypoint_and_direction(-4);
            } else {
                // 如果下一个点也是路口，使用当前路径点
                return local_planner_->get_incoming_waypoint_and_direction(0);
            }
        } else {
            // 如果没有下一个路径点，使用当前路径点
            return local_planner_->get_incoming_waypoint_and_direction(0);
        }
    } else {
        // 不在路口时，检查前一个路径点
        auto last_last_road_wp = local_planner_->get_incoming_waypoint_and_direction(-4);
        if (last_last_road_wp) {
            auto last_last_transform = ros_pose_to_carla_transform(last_last_road_wp->first);
            auto last_last_wpt = world_.GetMap()->GetWaypoint(last_last_transform.location);
            bool last_last_is_junction = last_last_wpt && last_last_wpt->IsJunction();
            
            if (last_last_is_junction) {
                // 如果前一个点是路口，往回看
                return local_planner_->get_incoming_waypoint_and_direction(-4);
            }else {
                // 如果前一个点不是路口，往前看
                return local_planner_->get_incoming_waypoint_and_direction(20);
            }
        }
    }
    return nullptr;
}

/**
 * @brief 获取自车路径
 * @param num_points 点数
 * @return 路径
 */
std::vector<carla::geom::Transform> ManualDriver::get_ego_path(int num_points) {
    std::vector<carla::geom::Transform> path;
    if (!local_planner_) return path;

    // 获取当前路径点
    auto current_wp = local_planner_->get_incoming_waypoint_and_direction(0);
    if (!current_wp) return path;

    // 获取当前路径点的transform
    carla::geom::Transform current_transform = ros_pose_to_carla_transform(current_wp->first);
    path.push_back(current_transform);

    // 获取前方路径点
    for (int i = 1; i < num_points; ++i) {
        auto next_wp = local_planner_->get_incoming_waypoint_and_direction(i);
        if (!next_wp) break;

        carla::geom::Transform next_transform = ros_pose_to_carla_transform(next_wp->first);
        path.push_back(next_transform);
    }

    return path;
}

/**
 * @brief 检查目标框是否与路径相交，并确定相交类型
 * @param path_poly 路径多边形
 * @param ego_transform 自车框的变换矩阵
 * @param target_transform 目标框的变换矩阵
 * @param target_extent_x 目标框x方向尺寸
 * @param target_extent_y 目标框y方向尺寸
 */
bool ManualDriver::is_box_intersect_path(
    const Polygon& path_poly,
    const carla::geom::Transform& target_transform,
    const double target_extent_x,
    const double target_extent_y) {
        
    // 参数验证
    if (target_extent_x <= 0.0 || target_extent_y <= 0.0) {
        ROS_WARN("Invalid target dimensions: x=%.2f, y=%.2f", target_extent_x, target_extent_y);
        return false;
    }

    // 验证路径多边形
    if (!bg::is_valid(path_poly) ) {
        ROS_WARN("[%s](isBoxIntersectingPath): Invalid path polygon", role_name_.c_str());
        return false;
    }
    if (bg::is_empty(path_poly)) {
        ROS_WARN("[%s](isBoxIntersectingPath): Empty path polygon", role_name_.c_str());
        return false;
    }
    
    // 创建目标框多边形
    Polygon box_polygon;
    const std::vector<carla::geom::Location> corners = 
        GetRectangleCorners(target_transform, target_extent_x, target_extent_y);

    // 构建并验证多边形
    if (corners.size() != 8) {
        ROS_ERROR("Invalid number of corners: %zu", corners.size());
        return false;
    }

    // 预分配内存并构建目标框多边形
    box_polygon.outer().reserve(9);  // 4个角点 + 4个中点 + 1个闭合点
    for (const auto& corner : corners) {
        box_polygon.outer().push_back({corner.x, corner.y});
    }
    box_polygon.outer().push_back({corners.front().x, corners.front().y});  // 闭合多边形
    // 检查是否相交
    bool intersects = boost::geometry::intersects(path_poly, box_polygon);
    if (!intersects) {
        return false;
    }
    return true;
}

std::vector<carla::geom::Transform> ManualDriver::get_target_path(
    const carla::SharedPtr<carla::client::Waypoint>& tar_wp,
    const int32_t& target_id,
    const std::string& direction,
    double spacing,
    int num_points) {

    std::vector<carla::geom::Transform> future_path;

    if (spacing <= 0.0 || num_points <= 0) {
        ROS_WARN("Invalid parameters: spacing=%.2f, num_points=%d", spacing, num_points);
        return future_path;
    }

    if (!tar_wp) {
        ROS_WARN("无法为目标车辆（ID=%d）找到路径预测的起始点。", target_id);
        return future_path;
    }
    // 开始沿着方向生成路径
    const double COS_THRESHOLD = std::cos(M_PI / 3.0);  // ≈ 0.5
    auto current_wp = tar_wp;

    for (int i = 0; i < num_points; ++i) {
        auto next_wps = current_wp->GetNext(spacing);
        if (next_wps.empty()) break;

        // 如果只有一个候选，直接使用
        if (next_wps.size() == 1) {
            current_wp = next_wps[0];
            future_path.push_back(current_wp->GetTransform());
            continue;
        }

        // 多个分支：先过滤掉方向差异过大的
        carla::geom::Vector3D curr_dir = current_wp->GetTransform().GetForwardVector().MakeUnitVector();
        std::vector<std::pair<carla::SharedPtr<carla::client::Waypoint>, double>> filtered_wps;

        for (auto& wp : next_wps) {
            auto next_dir = wp->GetTransform().GetForwardVector().MakeUnitVector();
            double dot = curr_dir.x * next_dir.x + curr_dir.y * next_dir.y;
            if (dot >= COS_THRESHOLD) {
                filtered_wps.emplace_back(wp, dot);
            }
        }

        if (filtered_wps.empty()) break;

        // 根据意图选择最合适的 waypoint
        carla::SharedPtr<carla::client::Waypoint> chosen_wp = nullptr;

        if (direction == "left") {
            double min_cross = std::numeric_limits<double>::max();
            for (const auto& [wp, _] : filtered_wps) {
                auto next_wp = wp->GetNext(6)[0];
                auto next_dir = next_wp->GetTransform().GetForwardVector().MakeUnitVector();
                double cross_z = curr_dir.x * next_dir.y - curr_dir.y * next_dir.x;
                if (cross_z < min_cross) {
                    min_cross = cross_z;
                    chosen_wp = wp;
                }
            }
        } else if (direction == "right") {
            double max_cross = -std::numeric_limits<double>::max();
            for (const auto& [wp, _] : filtered_wps) {
                auto next_wp = wp->GetNext(6)[0];
                auto next_dir = next_wp->GetTransform().GetForwardVector().MakeUnitVector();
                double cross_z = curr_dir.x * next_dir.y - curr_dir.y * next_dir.x;
                if (cross_z > max_cross) {
                    max_cross = cross_z;
                    chosen_wp = wp;
                }
            }
        } else if (direction == "straight") {
            double max_dot = -1.0;
            for (const auto& [wp, _] : filtered_wps) {
                auto next_wp = wp->GetNext(6)[0];
                auto next_dir = next_wp->GetTransform().GetForwardVector().MakeUnitVector();
                double dot = curr_dir.x * next_dir.x + curr_dir.y * next_dir.y;
                if (dot > max_dot) {
                    max_dot = dot;
                    chosen_wp = wp;
                }
            }
        } else {
            // "any" 模式 fallback
            chosen_wp = filtered_wps[0].first;
        }

        if (!chosen_wp) break;

        current_wp = chosen_wp;
        future_path.push_back(current_wp->GetTransform());
    }

    return future_path;
}

/**
 * @brief 绘制自车边界框
 */
void ManualDriver::draw_ego_box() {
    for (size_t i = 0; i < ego_corners.size(); ++i) {
        size_t next_i = (i + 1) % ego_corners.size();
        carla::geom::Location start(ego_corners[i].x, ego_corners[i].y, 1.0f);
        carla::geom::Location end(ego_corners[next_i].x, ego_corners[next_i].y, 1.0f);

        debug_helper_->DrawLine(
            start,
            end,
            0.1f,  // 线条宽度
            {255, 0, 0},  // 红色
            0.1f,  // 持续时间
            false
        );
    }
}

carla::SharedPtr<carla::client::Waypoint> ManualDriver::find_match_waypoint(
    int id, const carla::geom::Transform& transform, double max_dot, double search_resolution,
    std::unordered_map<int32_t, carla::SharedPtr<carla::client::Waypoint>>& last_valid_map) 
{
    const auto& loc = transform.location;
    const auto forward = transform.GetForwardVector().MakeUnitVector();
    const auto right = transform.GetRightVector().MakeUnitVector();  // 右方向向量

    std::vector<carla::geom::Location> candidates = {
        loc,
        loc + carla::geom::Location(right.x * search_resolution, right.y * search_resolution, 0.0),
        loc - carla::geom::Location(right.x * search_resolution, right.y * search_resolution, 0.0),
        loc + carla::geom::Location(forward.x * search_resolution, forward.y * search_resolution, 0.0),
        loc - carla::geom::Location(forward.x * search_resolution, forward.y * search_resolution, 0.0),
    };

    carla::SharedPtr<carla::client::Waypoint> best_wp = nullptr;

    for (const auto& candidate_loc : candidates) {
        auto wp = world_.GetMap()->GetWaypoint(candidate_loc, true, 2);
        if (!wp) continue;

        auto wp_forward = wp->GetTransform().GetForwardVector().MakeUnitVector();
        double dot = forward.x * wp_forward.x + forward.y * wp_forward.y;
        if (dot > max_dot) {
            max_dot = dot;
            best_wp = wp;
        }
    }
    if (best_wp) {
        last_valid_map[id] = best_wp;
        return best_wp;
    } else if (last_valid_map.count(id)) {
        ROS_DEBUG("FindBestMatchingWaypoint: no new valid waypoint found for id=%d, trying forward GetNext()", id);
        const auto& last_wp = last_valid_map[id];
        const auto& last_loc = last_wp->GetTransform().location;

        // Step 1: 自车位置 → last_wp 位置的距离
        double dx = last_loc.x - loc.x;
        double dy = last_loc.y - loc.y;
        double dz = last_loc.z - loc.z;
        double base_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (base_distance > 0.2) {
            ROS_DEBUG("base_distance = %.3f too large, reset to 0", base_distance);
            base_distance = 0.1;
        }
        const auto& ego_forward = transform.GetForwardVector().MakeUnitVector();

        const double max_offset = 0.5;
        const double step = 0.1;

        for (double offset = 0.0; offset <= max_offset + 1e-3; offset += step) {
            double d = base_distance + offset;

            auto next_wps = last_wp->GetNext(d);
            if (next_wps.empty()) continue;

            carla::SharedPtr<carla::client::Waypoint> best_wp = nullptr;
            double best_dot = -1.0;

            for (const auto& wp : next_wps) {
                auto wp_forward = wp->GetTransform().GetForwardVector().MakeUnitVector();
                double dot = wp_forward.x * ego_forward.x + wp_forward.y * ego_forward.y;

                if (dot > best_dot) {
                    best_dot = dot;
                    best_wp = wp;
                }
            }

            if (best_wp && best_dot > max_dot) {
                last_valid_map[id] = best_wp;
                ROS_DEBUG("✅ Found best matching wp (dot=%.3f > %.3f) at d=%.2f (offset=%.2f)", best_dot, max_dot, d, offset);
                return best_wp;
            } else {
                ROS_DEBUG("No wp at d=%.2f exceeded dot=%.3f (best=%.3f)", d, max_dot, best_dot);
            }
        }

        ROS_DEBUG("FindBestMatchingWaypoint: all forward candidates failed, returning last_valid_map[%d]", id);
        return last_valid_map[id];
    } else {
        ROS_DEBUG("FindBestMatchingWaypoint: no valid waypoint found for id=%d, returning nullptr", id);
        return nullptr;
    }
}

void ManualDriver::handle_intersection() {
}


void ManualDriver::handle_merge() {
    // TODO: 实现并道逻辑
    // 1. 检测周围车辆
    // 2. 判断是否可以并道
    // 3. 执行并道操作
    ROS_DEBUG("HandleMerge: 并道功能待实现");
}

void ManualDriver::MakeDecision() {
    // TODO: 实现决策逻辑
    // 1. 根据当前状态做出决策
    // 2. 更新控制命令
    ROS_DEBUG("MakeDecision: 决策功能待实现");
}

void ManualDriver::state_transition_tl() {
    switch (driver_state_) {
        case DriverState::INIT: {
            if (has_valid_position()) {
                driver_state_ = DriverState::CRUISE;
                ROS_DEBUG("%s: 状态转移 INIT → CRUISE", role_name_.c_str());
            }
            break;
        }

        case DriverState::CRUISE: {
            int road_id = ego_wpt->GetRoadId();
            int lane_id = ego_wpt->GetLaneId();
            auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
            if (it == road_lane_to_entry_stop_line_.end()) return;
            const auto& entry_line_tf = it->second;

            carla::geom::Location front_loc = ego_transform.location + carla::geom::Location(
                ego_forward.x * (ego_length * 0.5),
                ego_forward.y * (ego_length * 0.5),
                ego_forward.z * 0  
            );
            double dist_to_entry = carla::geom::Math::Distance(front_loc, entry_line_tf.location);
            if (dist_to_entry <= 50.0) {
                driver_state_ = DriverState::RUNING_IN_CONTROL_AREA;
                ROS_DEBUG("%s: 状态转移 CRUISE → RUNING_IN_CONTROL_AREA", role_name_.c_str());
            }
            break;
        }

        case DriverState::RUNING_IN_CONTROL_AREA: {
            int road_id = ego_wpt->GetRoadId();
            int lane_id = ego_wpt->GetLaneId();
            auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
            if (it == road_lane_to_entry_stop_line_.end()) return;
            const auto& entry_line_tf = it->second;

            carla::geom::Location front_loc = ego_transform.location + carla::geom::Location(
                ego_forward.x * (ego_length * 0.5),
                ego_forward.y * (ego_length * 0.5),
                ego_forward.z * 0  
            );
            double entry_to_stop = carla::geom::Math::Distance(front_loc, entry_line_tf.location);
            if (entry_to_stop <= 20.0) {
                driver_state_ = DriverState::RUNING_IN_DECISION_AREA;
                ROS_DEBUG("%s: 状态转移 RUNING_IN_CONTROL_AREA → RUNING_IN_DECISION_AREA", role_name_.c_str());
            }
            break;
        }

        case DriverState::RUNING_IN_DECISION_AREA:{
            double exceeded_entry = 0.0;
            if(has_emergency_conditon(inter_obs_, 1.0 )){
                last_non_emergency_state_ = driver_state_;
                driver_state_ = DriverState::EMERGENCY_STOP;
                ROS_DEBUG("%s: 状态转移 RUNING_IN_DECISION_AREA → EMERGENCY_STOP", role_name_.c_str());
            } else if(is_crossed_entry_line(ego_wpt, ego_transform, ego_length, ego_width, &exceeded_entry)){
                driver_state_ = DriverState::CROSSING_INTERSECTION;
                ROS_DEBUG("%s: 车尾已通过停止线，状态转移 RUNING_IN_DECISION_AREA → CROSSING_INTERSECTION", role_name_.c_str());
            } else if(yield_in_decision_area()){
                driver_state_ = DriverState::WAITING_IN_DECISION_AREA;
                ROS_DEBUG("%s: 状态转移 RUNING_IN_DECISION_AREA → WAITING_IN_DECISION_AREA", role_name_.c_str());
            }
            break;
        }

        case DriverState::WAITING_IN_DECISION_AREA: {
            double exceeded_entry = 0.0;
            if(is_crossed_entry_line(ego_wpt, ego_transform, ego_length, ego_width, &exceeded_entry)){
                if (exceeded_entry >= 1.0) {
                    driver_state_ = DriverState::CROSSING_INTERSECTION;
                    ROS_DEBUG("%s: 车尾已通过停止线，状态转移 WAITING_IN_DECISION_AREA → CROSSING_INTERSECTION", role_name_.c_str());
                }
            } else if (!yield_in_decision_area()) {
                driver_state_ = DriverState::RUNING_IN_DECISION_AREA;
                ROS_DEBUG("%s: 状态转移 WAITING_IN_DECISION_AREA → RUNING_IN_DECISION_AREA", role_name_.c_str());
            }
            break;
        }

        case DriverState::CROSSING_INTERSECTION: {
            double exceeded_exit = 0.0;
            if(has_emergency_conditon(inter_obs_, 1.0 )){
                last_non_emergency_state_ = driver_state_;
                driver_state_ = DriverState::EMERGENCY_STOP;
                ROS_DEBUG("%s: 状态转移 CROSSING_INTERSECTION → EMERGENCY_STOP", role_name_.c_str());
            } else if (is_crossed_exit_line(ego_wpt, ego_transform, ego_length, ego_width, &exceeded_exit)) {
                if (exceeded_exit >= 3.0) {
                    driver_state_ = DriverState::CRUISE;
                    ROS_DEBUG("%s: 已越过出口线，状态转移 CROSSING_INTERSECTION → CRUISE", role_name_.c_str());
                }
            }
            break;
        }

        case DriverState::EMERGENCY_STOP: {
            // ⚠️ 当前是绿灯，但仍需检查是否有“闯红灯”干涉车辆
            if (has_emergency_conditon(inter_obs_, 1.0 )) {
                ROS_WARN_THROTTLE(2.0, "%s: 检测到路口有闯红灯干涉车辆，维持停车", role_name_.c_str());
                break;
            }

            // ✅ 满足所有条件，安全恢复
            if (last_non_emergency_state_ != DriverState::INIT) {
                ROS_INFO("[%s] ✅ 当前为绿灯，且无干涉车辆/目标障碍物，准备恢复至原状态 [%s]",
                        role_name_.c_str(), DriverStateToString(last_non_emergency_state_).c_str());
                driver_state_ = last_non_emergency_state_;
            } else {
                ROS_WARN("[%s] ⚠️ 无有效记忆状态，默认恢复为 WAITING_IN_DECISION_AREA", role_name_.c_str());
                driver_state_ = DriverState::WAITING_IN_DECISION_AREA;
            }

            // 🧹 清除记忆状态，防止误用
            last_non_emergency_state_ = DriverState::INIT;
            break;
        }

        case DriverState::DESTROYED:
        default:
            break;
    }

    if (is_destroyed_) {
        driver_state_ = DriverState::DESTROYED;
        ROS_DEBUG("%s: 状态转移 → DESTROYED", role_name_.c_str());
    }
}

void ManualDriver::state_transition_untl() {
    switch (driver_state_) {
        case DriverState::INIT: {
            if (has_valid_position()) {
                driver_state_ = DriverState::CRUISE;
                ROS_DEBUG("%s: 状态转移 INIT → CRUISE", role_name_.c_str());
            }
            break;
        }

        case DriverState::CRUISE: {
            int road_id = ego_wpt->GetRoadId();
            int lane_id = ego_wpt->GetLaneId();
            auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
            if (it == road_lane_to_entry_stop_line_.end()) return;
            const auto& entry_line_tf = it->second;

            carla::geom::Location front_loc = ego_transform.location + carla::geom::Location(
                ego_forward.x * (ego_length * 0.5),
                ego_forward.y * (ego_length * 0.5),
                ego_forward.z * 0  
            );
            double dist_to_entry = carla::geom::Math::Distance(front_loc, entry_line_tf.location);
            if (dist_to_entry <= 50.0) {
                driver_state_ = DriverState::RUNING_IN_CONTROL_AREA;
                ROS_DEBUG("%s: 状态转移 CRUISE → RUNING_IN_CONTROL_AREA", role_name_.c_str());
            }
            break;
        }

        case DriverState::RUNING_IN_CONTROL_AREA: {
            double exceeded_entry = 0.0;
            if(is_crossed_entry_line(ego_wpt, ego_transform, ego_length, ego_width, &exceeded_entry)){
                driver_state_ = DriverState::CROSSING_INTERSECTION;
                ROS_DEBUG("%s: 车尾已通过停止线，状态转移 RUNING_IN_CONTROL_AREA → CROSSING_INTERSECTION", role_name_.c_str());
            } 
            break;
        }

        case DriverState::CROSSING_INTERSECTION: {
            double exceeded_exit = 0.0;
            if (is_crossed_exit_line(ego_wpt, ego_transform, ego_length, ego_width, &exceeded_exit)) {
                if (exceeded_exit >= 3.0) {
                    driver_state_ = DriverState::CRUISE;
                    ROS_DEBUG("%s: 已越过出口线，状态转移 CROSSING_INTERSECTION → CRUISE", role_name_.c_str());
                }
            }
            break;
        }

        case DriverState::DESTROYED:
        default:
            break;
    }

    if (is_destroyed_) {
        driver_state_ = DriverState::DESTROYED;
        ROS_DEBUG("%s: 状态转移 → DESTROYED", role_name_.c_str());
    }
}

void ManualDriver::handle_state() {
    switch (driver_state_) {
        case DriverState::INIT: {
            if (has_valid_position()) {
                driver_state_ = DriverState::CRUISE;
                ROS_DEBUG("%s: 状态转移 INIT → CRUISE", role_name_.c_str());
            }
            break;
        }

        case DriverState::CRUISE: {
            break;
        }

        case DriverState::RUNING_IN_CONTROL_AREA: {
            break;
        }

        case DriverState::RUNING_IN_DECISION_AREA: {
            break;
        }

        case DriverState::WAITING_IN_DECISION_AREA: {
            if(1){
                handle_trafficlight();
            }
            break;
        }

        case DriverState::CROSSING_INTERSECTION: {
            break;
        }

        case DriverState::EMERGENCY_STOP: {
            handle_emergency_stop_state(emergency_obs_);
            break;
        }

        case DriverState::DESTROYED: {
            break;
        }

        default: {
            ROS_WARN("%s: ⚠️ Unknown driver state!", role_name_.c_str());
            break;
        }
    }  
}

double ManualDriver::PlanSpeed() {
    double target_speed = 0.0;
// !单独考虑交通灯，同时考虑前方最近障碍物，取最小速度，交通灯的最小idm参数需要修改！10hz
    // 🚗 当前状态
    bool is_starting = current_speed_ < 1.0;

    // 🧠 IDM 参数
    const double v0 = vehicle_config_.cruise_speed.data;
    const double a = 5.0;
    const double b = 2.0;
    const double T = 1.5;
    const double s0 = 2.0;
    const double dt = 0.1;

    // 🧱 找出最近前方障碍物（s_ini > s_ego）
    ObstacleIDM* closest_obs = nullptr;
    double min_gap = 1e9;

    for (auto& obs : obstacles_) {
        if (obs.s_ini < min_gap) {
            min_gap = obs.s_ini;
            closest_obs = &obs;
        }
    }

    // 🔍 打印障碍物选择结果
    if (closest_obs) {
        // ROS_INFO("[%s] 🧱 最近障碍物: s=%.2f, v=%.2f, name=%s",
        //          role_name_.c_str(), closest_obs->s_ini, closest_obs->velocity, closest_obs->name.c_str());
    } else {
        // ROS_INFO("[%s] ✅ 当前无前方障碍，准备自由行驶", role_name_.c_str());
    }

    // 🚦 起步阶段处理逻辑
    if (is_starting && closest_obs && min_gap < 3.0 && closest_obs->name != "Red" && closest_obs->name != "Yellow") {
        ROS_WARN("[%s] 🚦 起步阶段：前方 %.2fm 有障碍[%s]，等待", role_name_.c_str(), min_gap, closest_obs->name.c_str());
        target_speed = 0.0;
    }else if (closest_obs) {
        double s_gap = std::max(closest_obs->s_ini - s0, 0.1);  // ego 到障碍物距离
        double v_lead = closest_obs->velocity;
        double delta_v = current_speed_ - v_lead;
        double s_star = s0 + current_speed_ * T + (current_speed_ * delta_v) / (2.0 * std::sqrt(a * b));
        double acc = a * (1 - std::pow(current_speed_ / v0, 4) - std::pow(s_star / s_gap, 2));
        double v_idm = std::max(0.0, current_speed_ + acc * dt);

        // 🚦 特别处理红灯停车线不能越线
        if (closest_obs->name == "Red" || closest_obs->name == "Yellow") {
            double stop_line_s = closest_obs->s_ini;
            double dist_to_stop_line = stop_line_s;  // s_ego = 0
            // 如果 IDm 给出速度过小，可能会停得太远，我们希望更靠近停止线
            if (dist_to_stop_line > 1.5 && v_idm < 1.0 && vehicle_config_.cruise_speed.data > 1.0) {
                v_idm = 2.0;  // 拉近距离
                ROS_WARN("[%s] 🚦 红灯等待，IDM停车太远，微调靠近：dist=%.2f, v=%.2f",
                        role_name_.c_str(), dist_to_stop_line, v_idm);
            }

            // 如果距离 < 0.5m，直接强制停车
            if (dist_to_stop_line <= 0.5) {
                v_idm = 0.0;
                ROS_WARN("[%s] 🛑 接近红灯停止线（%.2f m），强制停车", role_name_.c_str(), dist_to_stop_line);
            }
        }

        target_speed = v_idm;

        // ROS_INFO("[%s] 🧠 IDM 跟车：s_gap=%.2f, s_star=%.2f, Δv=%.2f, acc=%.2f, v_target=%.2f",
        //         role_name_.c_str(), s_gap, s_star, delta_v, acc, target_speed);
    }else {
        double acc = a * (1 - std::pow(current_speed_ / v0, 4));
        target_speed = std::max(0.0, current_speed_ + acc * dt);
        // ROS_INFO("[%s] 🛣️ 自由行驶，加速至 v_target=%.2f，当前速度=%.2f，加速度=%.2f, 期望速度=%.2f", role_name_.c_str(), target_speed, current_speed_, acc, v0);
    }
    // ✅ 目标点设置
    geometry_msgs::Pose target_pose;
    if (interp_path.size() > 6) {
        target_pose = carla_transform_to_ros_pose(interp_path[6]);
    } else if (!interp_path.empty()) {
        target_pose = carla_transform_to_ros_pose(interp_path.back());
    }
    local_planner_->set_target_pose(target_pose);
    local_planner_->set_target_speed(target_speed);

    return target_speed;
}


std::string ManualDriver::DriverStateToString(DriverState state) {
    switch (state) {
        case DriverState::INIT:
            return "INIT";
        case DriverState::CRUISE:
            return "CRUISE";
        case DriverState::RUNING_IN_CONTROL_AREA:
            return "RUNING_IN_CONTROL_AREA";
        case DriverState::RUNING_IN_DECISION_AREA:
            return "RUNING_IN_DECISION_AREA";
        case DriverState::WAITING_IN_DECISION_AREA:
            return "WAITING_IN_DECISION_AREA";
        case DriverState::CROSSING_INTERSECTION:
            return "CROSSING_INTERSECTION";
        case DriverState::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case DriverState::DESTROYED:
            return "DESTROYED";
        default:
            return "UNKNOWN_STATE";
    }
}
bool ManualDriver::has_valid_position() const {
    // 1. 判断 Transform 是否被正确初始化（不是默认值）
    const auto& loc = ego_transform.location;
    bool is_location_valid = std::isfinite(loc.x) && std::isfinite(loc.y);

    // 2. Waypoint 不为空
    bool has_waypoint = ego_wpt != nullptr;

    return is_location_valid && has_waypoint;
}

bool ManualDriver::has_emergency_conditon(
    const std::vector<obstacle>& inter_obstacles,
    double emergency_speed_threshold)
{
    emergency_obs_.clear();  // 每次检测前先清空

    bool has_emergency = false;

    for (const auto& obs : inter_obstacles) {
        int road_id = obs.waypoint->GetRoadId();
        int lane_id = obs.waypoint->GetLaneId();
        auto key = std::make_pair(road_id, lane_id);

        auto it = road_lane_to_entry_stop_line_.find(key);
        if (it == road_lane_to_entry_stop_line_.end()) {
            ROS_WARN("%s:未找到 road_id=%d, lane_id=%d 对应的 entry 停车线，跳过该目标",
                     role_name_.c_str(), road_id, lane_id);
            continue;
        }

        int light_id = -1;
        auto road_it = road_to_traffic_light_.find(road_id);
        if (road_it != road_to_traffic_light_.end()) {
            light_id = road_it->second;
        }

        auto phase_it = traffic_light_map_.find(light_id);
        if (phase_it == traffic_light_map_.end()) {
            ROS_WARN("%s:未找到 road_id=%d 对应的信号灯状态，跳过该目标",
                     role_name_.c_str(), road_id);
            continue;
        }

        const auto& phase = phase_it->second;
        const carla::geom::Location& entry_line = it->second.location;
        carla::geom::Vector3D forward = obs.transform.GetForwardVector().MakeUnitVector();
        carla::geom::Location veh_rear_loc = obs.transform.location - static_cast<carla::geom::Location>(forward * (obs.length * 0.5));

        auto stop_to_vehicle = veh_rear_loc - entry_line;
        double dot = stop_to_vehicle.x * forward.x + stop_to_vehicle.y * forward.y;

        if (dot > 0.0 && phase.state == "Red" && obs.speed > emergency_speed_threshold) {
            emergency_obs_.push_back(obs);  // ➕ 添加到成员变量
            ROS_WARN("[%s] 🚨 紧急干涉目标加入列表！", role_name_.c_str());
            ROS_WARN("[%s] ➤ ID=%d, Road=%d, Lane=%d, Speed=%.2f m/s",
                     role_name_.c_str(), obs.id, road_id, lane_id, obs.speed);
            ROS_WARN("[%s] 🚧 条件：闯红灯 + 越过停止线 + 速度超过 %.2f m/s",
                     role_name_.c_str(), emergency_speed_threshold);
            has_emergency = true;
        }
    }
    return has_emergency;
}


void ManualDriver::handle_emergency_stop_state(const std::vector<obstacle>& emergency_obs_) {
    for (const auto& obs : emergency_obs_) {
        // 获取当前目标车轨迹
        const auto& tar_interp_path = obs.interp_path;
        const auto& tar_interp_s_list = obs.interp_s_list;
        const double tar_speed_mps = obs.speed;
        const auto& points = obs.overlap_points;

        if (tar_interp_path.empty() || tar_interp_s_list.empty() || points.size() < 3) {
            continue;
        }

        // Step 1: 构建重叠区域
        bg::model::ring<Point> ring_input;
        ring_input.assign(points.begin(), points.end());
        Polygon overlap_poly;
        bg::convex_hull(ring_input, overlap_poly);
        const double half_width = ego_width * 0.5;

        // Step 2: 查找 ego_entry_index
        int ego_entry_index = -1;
        int entry_candidate_index = -1;
        for (size_t i = 0; i + 1 < interp_path.size(); ++i) {
            Point pt1(interp_path[i].location.x, interp_path[i].location.y);
            Point pt2(interp_path[i + 1].location.x, interp_path[i + 1].location.y);
            if (!bg::covered_by(pt1, overlap_poly) && bg::covered_by(pt2, overlap_poly)) {
                entry_candidate_index = static_cast<int>(i);
                break;
            }
        }
        if (entry_candidate_index >= 0) {
            for (int i = entry_candidate_index; i >= 0; --i) {
                const auto& tf = interp_path[i];
                const auto& center = tf.location;
                auto right_vec = tf.GetRightVector();
                Point left_pt(center.x - half_width * right_vec.x, center.y - half_width * right_vec.y);
                Point right_pt(center.x + half_width * right_vec.x, center.y + half_width * right_vec.y);
                if (!bg::covered_by(left_pt, overlap_poly) && !bg::covered_by(right_pt, overlap_poly)) {
                    ego_entry_index = i;
                    break;
                }
            }
        }

        // Step 3: 查找 tar_entry_index
        int tar_entry_index = -1;
        int tar_entry_candidate = -1;
        for (size_t i = 0; i + 1 < tar_interp_path.size(); ++i) {
            Point pt1(tar_interp_path[i].location.x, tar_interp_path[i].location.y);
            Point pt2(tar_interp_path[i + 1].location.x, tar_interp_path[i + 1].location.y);
            if (!bg::covered_by(pt1, overlap_poly) && bg::covered_by(pt2, overlap_poly)) {
                tar_entry_candidate = static_cast<int>(i);
                break;
            }
        }
        if (tar_entry_candidate >= 0) {
            for (int i = tar_entry_candidate; i >= 0; --i) {
                const auto& tf = tar_interp_path[i];
                const auto& center = tf.location;
                auto right_vec = tf.GetRightVector();
                Point left_pt(center.x - half_width * right_vec.x, center.y - half_width * right_vec.y);
                Point right_pt(center.x + half_width * right_vec.x, center.y + half_width * right_vec.y);
                if (!bg::covered_by(left_pt, overlap_poly) && !bg::covered_by(right_pt, overlap_poly)) {
                    tar_entry_index = i;
                    break;
                }
            }
        }

        if (ego_entry_index < 0 || tar_entry_index < 0) continue;

        // Step 4: 判断是否让行
        std::vector<speed_planner::Obstacle> empty_obs;
        auto speed_result = EstimateTravelTime(empty_obs, s_list, v_limit_list, false);
        double s_ego_entry = s_list[ego_entry_index];
        double s_tar_entry = tar_interp_s_list[tar_entry_index];
        double t_ego_entry = std::get<1>(speed_result).time[ego_entry_index];
        double t_tar_entry = s_tar_entry / std::max(tar_speed_mps, 1e-3);
        double v_ego = current_speed_;
        double v_tar = tar_speed_mps;

        bool should_pass = false;

        if (s_ego_entry < 10.0) {
            if (s_tar_entry < 10.0) {
                // 自车先到达，允许通过
                if (t_ego_entry - t_tar_entry <= -5.0) {
                    should_pass = true;
                }
            } else if (s_tar_entry >= 10.0 && s_tar_entry <= 20.0) {
                if (t_ego_entry - t_tar_entry < -3.0) {
                    should_pass = true;
                }
            } else {
                should_pass = true;
            }
        } else {
            if (s_tar_entry >= 20.0) {
                should_pass = true;
            } else if (s_tar_entry >= 10.0 && t_ego_entry - t_tar_entry < -5.0) {
                should_pass = true;
            }
        }

        if (should_pass) {
            // 🚗 自车优先通过，不添加障碍物
            ROS_INFO("%s: ✅ 决策为优先通行，跳过障碍物添加 (s_ego=%.2f, s_tar=%.2f, t_ego=%.2f, t_tar=%.2f)",
                    role_name_.c_str(), s_ego_entry, s_tar_entry, t_ego_entry, t_tar_entry);
            continue;
        }

        // Step 5: 添加虚拟障碍物

        ObstacleIDM virtual_obs(
            0.0, 10.0,
            s_ego_entry, s_ego_entry,
            0.0
        );
        obstacles_.emplace_back(virtual_obs);
        closest_obstacle_info_.update(virtual_obs, 0);

        ROS_WARN("%s: ⛔ 进入 EMERGENCY_STOP 状态，等待目标车通过 (s_ego=%.2f, s_tar=%.2f, t_ego=%.2f, t_tar=%.2f)",
                 role_name_.c_str(), s_ego_entry, s_tar_entry, t_ego_entry, t_tar_entry);
    }
}

double ManualDriver::calcu_distance_to_entry_line(const carla::SharedPtr<carla::client::Waypoint>& wp,
                                           const carla::geom::Location& ego_loc) const {
    if (!wp) return -1.0;

    int road_id = wp->GetRoadId();
    int lane_id = wp->GetLaneId();

    auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
    if (it == road_lane_to_entry_stop_line_.end()) return -1.0;

    const auto& stop_line = it->second.location;
    return carla::geom::Math::Distance(ego_loc, stop_line);
}

 
bool ManualDriver::yield_in_decision_area() {

    // 1. 基本参数
    int road_id = ego_wpt->GetRoadId();
    int lane_id = ego_wpt->GetLaneId();
    auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
    if (it == road_lane_to_entry_stop_line_.end()) {
        ROS_WARN("%s: 无法找到停车线信息，默认等待 (road_id = %d, lane_id = %d)", 
                role_name_.c_str(), road_id, lane_id);
        return true;
    }
    const auto& entry_line_tf = it->second;

    carla::geom::Location front_loc = ego_transform.location + carla::geom::Location(
        ego_forward.x * (ego_length * 0.5),
        ego_forward.y * (ego_length * 0.5),
        ego_forward.z * 0  
    );
    double distance_to_entry = carla::geom::Math::Distance(front_loc, entry_line_tf.location);

    double ego_speed = current_speed_;
    double remaining_time = 0.0;
    std::string phase_state;

    // 4. 查询交通灯状态
    int light_id = -1;  // 默认无效值

    auto road_it = road_to_traffic_light_.find(road_id);
    if (road_it != road_to_traffic_light_.end()) {
        light_id = road_it->second;
    }else {
        ROS_WARN("%s: 未找到 road_id=%d 的交通灯信息，默认等待", role_name_.c_str(), road_id);
        return true;
    }

    auto phase_it = traffic_light_map_.find(light_id);
    if (phase_it == traffic_light_map_.end()) {
        ROS_WARN("%s: 无法获取交通灯状态，默认等待", role_name_.c_str());
        return true;
    }

    const auto& phase = phase_it->second;
    remaining_time = phase.remaining_time;
    phase_state = phase.state;

    // 4. 简化模糊规则判断

    constexpr double D_NEAR = 8.0;
    constexpr double D_FAR = 15.0;

    constexpr double V_SLOW = 2.0;
    constexpr double V_FAST = 6.0;

    constexpr double T_SHORT = 5.0;
    constexpr double T_LONG = 10.0;

    ROS_DEBUG("%s: 距离 %.2f m，速度 %.2f m/s，%s灯剩余 %.2f s", 
             role_name_.c_str(), distance_to_entry, ego_speed, phase_state.c_str(), remaining_time);

    if (phase_state == "Red") {
        // 规则 1：远 → 通行
        if (distance_to_entry >= D_FAR) {
            ROS_DEBUG("%s: 红灯 + 距离远 → 通行", role_name_.c_str());
            return false;
        }
        ROS_DEBUG("%s: 当前为红灯，等待", role_name_.c_str());
        return true;
    }

    if (phase_state == "Yellow") {
        if (distance_to_entry >= D_FAR) {
            ROS_DEBUG("%s: 黄灯 + 距离远 → 通行", role_name_.c_str());
            return false;
        }
        ROS_DEBUG("%s: 黄灯条件不足，等待", role_name_.c_str());
        return true;
    }

    // 绿灯时判断模糊组合
    if (phase_state == "Green") {
        // 规则 1：远 + 剩余长 → 通行
        if (distance_to_entry >= D_FAR && remaining_time >= T_LONG) {
            ROS_DEBUG("%s: 绿灯 + 距离远 + 剩余长 → 通行", role_name_.c_str());
            return false;
        }

        // 规则 2：近 + 剩余时间尚可 → 通行
        if (distance_to_entry >= D_NEAR && ego_speed > V_FAST && remaining_time >= T_SHORT) {
            ROS_DEBUG("%s: 绿灯 + 距离近 + 剩余尚可 → 通行", role_name_.c_str());
            return false;
        }

        if (3.0 < distance_to_entry && distance_to_entry < D_NEAR && remaining_time >= T_SHORT) {
            ROS_DEBUG("%s: 绿灯 + 距离近 + 剩余尚可 → 通行", role_name_.c_str());
            return false;
        }

        if (distance_to_entry <= 3.0 && remaining_time >= T_SHORT) {
            ROS_DEBUG("%s: 绿灯 + 距离近 + 剩余尚可 → 通行", role_name_.c_str());
            return false;
        }

        ROS_DEBUG("%s: 绿灯默认条件满足 → 通行", role_name_.c_str());
        return true;
    }

    // 其它未知灯 → 保守等待
    ROS_DEBUG("%s: 未知灯相位，默认等待", role_name_.c_str());
    return true;
}


bool ManualDriver::is_crossed_exit_line(
    const carla::SharedPtr<carla::client::Waypoint>& wp, 
    const carla::geom::Transform& transform,
    const double extent_x,
    const double extent_y,
    double* exceeded_distance) {

    const int road_id = wp->GetRoadId();
    const int lane_id = wp->GetLaneId();
    auto key = std::make_pair(road_id, lane_id);

    auto it = road_lane_to_exit_reference_.find(key);
    if (it == road_lane_to_exit_reference_.end()) {
        ROS_WARN("%s: 找不到 road_id=%d, lane_id=%d 对应的停车线", 
                 role_name_.c_str(), road_id, lane_id);
        if (exceeded_distance) *exceeded_distance = 0.0;
        return false;
    }

    const carla::geom::Location& exit_line = it->second.location;

    // 计算车后轴位置
    auto forward = transform.GetForwardVector().MakeUnitVector();
    carla::geom::Location rear_loc = transform.location - 
        static_cast<carla::geom::Location>(forward * (extent_x * 0.5));

    // 点积判断：rear_loc 是否在停止线前方
    auto vec = rear_loc - exit_line;
    carla::geom::Vector3D exit_forward = it->second.GetForwardVector().MakeUnitVector();

    double dot = vec.x * exit_forward.x + vec.y * exit_forward.y;  // 单位方向 ⇒ dot 即为距离

    if (exceeded_distance) *exceeded_distance = dot;

    return dot > 0.0;
}

bool ManualDriver::is_crossed_entry_line(
const carla::SharedPtr<carla::client::Waypoint>& wp, 
const carla::geom::Transform& transform,
const double extent_x,
const double extent_y,
double* exceeded_distance) {

    const int road_id = wp->GetRoadId();
    const int lane_id = wp->GetLaneId();
    auto key = std::make_pair(road_id, lane_id);

    auto it = road_lane_to_entry_stop_line_.find(key);
    if (it == road_lane_to_entry_stop_line_.end()) {
        ROS_WARN("%s: 找不到 road_id=%d, lane_id=%d 对应的停车线", 
                 role_name_.c_str(), road_id, lane_id);
        if (exceeded_distance) *exceeded_distance = 0.0;
        return false;
    }

    const carla::geom::Location& entry_line = it->second.location;

    // 计算车后轴位置
    auto forward = transform.GetForwardVector().MakeUnitVector();
    carla::geom::Location rear_loc = transform.location - 
        static_cast<carla::geom::Location>(forward * (extent_x * 0.5));

    // 点积判断：rear_loc 是否在停止线前方
    auto vec = rear_loc - entry_line;

    carla::geom::Vector3D entry_forward = it->second.GetForwardVector().MakeUnitVector();

    double dot = vec.x * entry_forward.x + vec.y * entry_forward.y;  // 单位方向 ⇒ dot 即为距离

    if (exceeded_distance) *exceeded_distance = dot;

    return dot > 0.0;
}

void ManualDriver::interpolate_path(
    const std::vector<carla::geom::Transform>& path,
    const std::vector<double>& s_list,
    double spacing,
    double v_limit,
    std::vector<carla::geom::Transform>& interp_path,
    std::vector<double>& s_interp,
    std::vector<double>& v_interp)
{
    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;

    if (path.empty() || s_list.size() != path.size()) return;

    interp_path.clear();
    s_interp.clear();
    v_interp.clear();

    auto NormalizeAngle = [](double angle_rad) -> double {
        while (angle_rad > M_PI)  angle_rad -= 2.0 * M_PI;
        while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
        return angle_rad;
    };

    double s_max = s_list.back();
    size_t N = s_list.size();
    size_t i = 0;
    for (double s = 0.0; s < s_max; s += spacing) {
        // 查找插值区间 i，满足 s_list[i] <= s < s_list[i+1]
        while (i + 1 < N && s_list[i + 1] < s) ++i;

        if (i + 1 >= N) break;

        double s0 = s_list[i];
        double s1 = s_list[i + 1];
        double ratio = (s - s0) / (s1 - s0);

        const auto& tf0 = path[i];
        const auto& tf1 = path[i + 1];

        // 插值位置
        carla::geom::Location loc;
        loc.x = tf0.location.x + ratio * (tf1.location.x - tf0.location.x);
        loc.y = tf0.location.y + ratio * (tf1.location.y - tf0.location.y);
        loc.z = tf0.location.z + ratio * (tf1.location.z - tf0.location.z);

        // 插值角度
        double yaw0   = tf0.rotation.yaw   * DEG2RAD;
        double yaw1   = tf1.rotation.yaw   * DEG2RAD;
        double pitch0 = tf0.rotation.pitch * DEG2RAD;
        double pitch1 = tf1.rotation.pitch * DEG2RAD;
        double roll0  = tf0.rotation.roll  * DEG2RAD;
        double roll1  = tf1.rotation.roll  * DEG2RAD;

        double dyaw   = NormalizeAngle(yaw1 - yaw0);
        double dpitch = NormalizeAngle(pitch1 - pitch0);
        double droll  = NormalizeAngle(roll1 - roll0);

        double yaw_interp   = NormalizeAngle(yaw0 + ratio * dyaw);
        double pitch_interp = NormalizeAngle(pitch0 + ratio * dpitch);
        double roll_interp  = NormalizeAngle(roll0 + ratio * droll);

        carla::geom::Rotation rot;
        rot.yaw   = yaw_interp   * RAD2DEG;
        rot.pitch = pitch_interp * RAD2DEG;
        rot.roll  = roll_interp  * RAD2DEG;

        interp_path.emplace_back(loc, rot);
        s_interp.push_back(s);
        v_interp.push_back(v_limit);
    }

    // ➕ 补上最后一个点（s = s_list.back()）
    interp_path.push_back(path.back());
    s_interp.push_back(s_list.back());
    v_interp.push_back(v_limit);
}

std::optional<std::pair<double, size_t>> ManualDriver::project_point_to_path(
    const carla::geom::Location& point,
    const std::vector<carla::geom::Transform>& interp_path,
    const std::vector<double>& s_list,
    double eps_dist_sq)
{

    const auto& start = interp_path.front().location;
    const auto& end = interp_path.back().location;
    const auto& start_next = interp_path[1].location;
    const auto& end_prev = interp_path[interp_path.size() - 2].location;

    // 方向向量（整条路径起点方向）
    auto dir_start = start_next - start;
    auto dir_end = end - end_prev;

    auto vec_to_start = point - start;
    auto vec_to_end = point - end;

    // 若点在路径整体前方
    if (carla::geom::Math::Dot(vec_to_start, dir_start) <= 0) {
        return std::make_pair(s_list[0], 0);
    }

    // 若点在路径整体后方
    if (carla::geom::Math::Dot(vec_to_end, dir_end) >= 0) {
        return std::make_pair(s_list.back(), interp_path.size() - 1);
    }


    double min_s = 0.0;
    int min_index = -1;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i + 1 < interp_path.size(); ++i) {
        const auto& p0 = interp_path[i].location;
        const auto& p1 = interp_path[i + 1].location;

        auto diff = point - p0;
        double dist_sq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        if (dist_sq < eps_dist_sq) {
            // 认为 point 就在路径段起点上，直接返回
            return std::make_pair(s_list[i], i);
        }
        auto seg = p1 - p0;
        auto vec = point - p0;
        double seg_len_sq = carla::geom::Math::Dot(seg, seg);
        if (seg_len_sq < 1e-6) continue;

        double proj = carla::geom::Math::Dot(vec, seg);
        double ratio = proj / seg_len_sq;

        constexpr double epsilon = 2*1e-1;
        if (ratio >= -epsilon && ratio <= 1.0 + epsilon) {
            // 投影在 [p0, p1] 上
            double s_proj = s_list[i] + ratio * (s_list[i + 1] - s_list[i]);

            // 可选：计算投影点与原点的距离平方，用于判断最接近段
            carla::geom::Location proj_pt = p0 + carla::geom::Location(seg * ratio);
            auto diff = point - proj_pt;
            double dist_sq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;


            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                min_s = s_proj;
                min_index = i;
            }
        }
    }
    if (min_index != -1) {
        return std::make_pair(min_s, static_cast<size_t>(min_index));
    } else {
        return std::nullopt;
    }
}

void ManualDriver::get_forward_path(
    const carla::geom::Transform& transform,
    double extent_x,
    const std::vector<carla::geom::Transform>& path,
    std::vector<carla::geom::Transform>& trimmed_path,
    std::vector<double>& raw_s_list,
    const Polygon vehicle_polygon)
{
    using namespace carla::geom;

    trimmed_path.clear();
    raw_s_list.clear();

    Vector3D forward = transform.GetForwardVector().MakeUnitVector();
    Location front_loc = transform.location + static_cast<Location>(forward * (extent_x * 0.5));

    auto Distance = [](const Location& a, const Location& b) {
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    bool found_start = false;
    double eps_dist_sq = 0.5;

    for (size_t i = 0; i < path.size()-1; ++i) {
        const auto& wp = path[i];
        const auto& wp_next = path[i + 1];

        Point pt(wp.location.x, wp.location.y);
        Point pt_next(wp_next.location.x, wp_next.location.y);

        if (!bg::covered_by(pt, vehicle_polygon) && !bg::covered_by(pt_next, vehicle_polygon) && !found_start) {
          
            auto seg = forward;
            auto vec = wp.location - front_loc;
            double dot = carla::geom::Math::Dot(vec, seg);

            if (dot >= 0) {
                // 2. 计算 wp 相对 front_loc 在车辆 forward 上的投影长度
                Vector3D to_wp = wp.location - front_loc;
                double forward_proj_len = carla::geom::Math::Dot(to_wp, forward);

                // 3. 判断是否满足“沿车头方向的前方 ≥ 0.3m”
                if (forward_proj_len >= 0.3) {
                    found_start = true;

                    trimmed_path.emplace_back(front_loc, wp.rotation);
                    raw_s_list.push_back(0.0);

                    trimmed_path.push_back(wp);
                    raw_s_list.push_back(Distance(wp.location, front_loc));

                    for (size_t j = i + 1; j < path.size(); ++j) {
                        const auto& curr_wp = path[j];
                        const Location& prev = trimmed_path.back().location;
                        const Location& curr = curr_wp.location;

                        double dseg = Distance(curr, prev);
                        if (dseg < 1e-2) continue;

                        trimmed_path.push_back(curr_wp);
                        raw_s_list.push_back(raw_s_list.back() + dseg);
                    }
                    break;
                }
            }
        }
    }

    if (!found_start) {
        ROS_WARN("[%s] No forward path found in ExtractForwardPath", role_name_.c_str());
    }
}

void ManualDriver::reset() {
    // 重置状态变量
    driver_state_ = DriverState::INIT;  // 重置为初始化状态
    is_destroyed_ = false;              // 标记为非销毁状态

    // 清空路径相关信息
    ego_path.clear();                   // 清空自车路径
    interp_path.clear();                // 清空插值路径
    s_list.clear();                     // 清空路径的 s 值
    v_limit_list.clear();               // 清空速度限制列表

    // 清空感知相关的信息
    inter_obs_.clear();                 // 重置当前周期的跟车目标
    tar_obs_.clear();                   // 清空目标障碍物集合
    obstacles_.clear();                 // 清空障碍物集合
    last_valid_ego_wpts_.clear();       // 清空自车上次有效的目标路径点
    last_valid_tar_wpts_.clear();       // 清空目标车辆上次有效的目标路径点

    // 清空车辆信息
    ego_pose = {};                      // 清空自车位置
    ego_transform = {};                 // 清空自车变换信息
    ego_forward = {};                   // 清空自车朝向向量
    ego_wpt.reset();                    // 重置自车路径点
    ego_corners.clear();                // 清空自车四个角点
    ego_polygon.clear();                // 清空自车多边形
    ego_path_poly.clear();              // 清空自车路径多边形

    // 重置速度和加速度
    current_speed_ = 0.0;            // 重置当前速度
    current_acc_ = 0.0;                 // 重置当前加速度

    // 清空交通灯信息
    traffic_light_map_.clear();         // 清空交通灯相位信息
    affecting_traffic_light = nullptr;  // 清空交通灯对象

    // 清空周围物体信息
    carla_objects_.clear();             // 清空物体信息
    target_stationary_counter_.clear(); // 清空目标静止计数器
}

void ManualDriver::destroy() {
    std::lock_guard<std::mutex> objects_lock(data_mutex);
    if (is_destroyed_) {
        ROS_WARN("%s(%d): destroy() called, but already destroyed.", role_name_.c_str(), vehicle_config_.carla_id);
        return;
    }

    is_destroyed_ = true;

    try {
        ROS_INFO("%s(%d): Destroying ManualDriver...", role_name_.c_str(), vehicle_config_.carla_id);

        // 1. 关闭 ROS 订阅者
        if (objects_sub) {
            objects_sub.shutdown();
        }
        if (traffic_lights_phases_sub) {
            traffic_lights_phases_sub.shutdown();
        }

        // 3. 清空路径与状态数据
        global_path_ = nullptr;
        ego_wpt = nullptr;
        ego_path.clear();
        interp_path.clear();
        s_list.clear();
        v_limit_list.clear();
        ego_corners.clear();
        ego_polygon.outer().clear();
        ego_path_poly.outer().clear();

        // 4. 清空动态对象、规划器、感知数据
        carla_objects_.clear();
        inter_obs_.clear();                 // 重置当前周期的跟车目标
        tar_obs_.clear();                   // 清空目标障碍物集合
        obstacles_.clear();
        last_valid_tar_wpts_.clear();
        last_valid_ego_wpts_.clear();
        traffic_light_map_.clear();
        road_lane_to_entry_stop_line_.clear();
        road_lane_to_exit_reference_.clear();
        road_to_traffic_light_.clear();

        data_pool_.reset();

        ROS_INFO("%s(%d): ManualDriver successfully destroyed.", role_name_.c_str(), vehicle_config_.carla_id);

    } catch (const std::exception& e) {
        ROS_ERROR("%s(%d): Exception during destroy(): %s", role_name_.c_str(), vehicle_config_.carla_id, e.what());
    } catch (...) {
        ROS_ERROR("%s(%d): Unknown exception occurred during destroy().", role_name_.c_str(), vehicle_config_.carla_id);
    }
}

std::shared_ptr<local_planner::LocalPlanner> ManualDriver::get_local_planner() {
    return local_planner_;
}

int ManualDriver::get_vehicle_id() const {
    return id_;
}

void ManualDriver::lane_change(const carla::geom::Location& trigger_loc) {
    // 如果已经执行过换道，不再重复执行
    if (lane_change_triggered_ ) {
        return;
    }

    // 当前车辆位置
    auto current_transform = vehicle_carla_actor_->GetTransform();
    carla::geom::Location current_loc = current_transform.location;

    // 判断是否到达触发区域（1米范围内）
    double dist = current_loc.Distance(trigger_loc);
    if (dist > 1.0) {
        ROS_INFO_THROTTLE(1.0, "[%s] 未到达触发位置(%.2f m)，等待中...", role_name_.c_str(), dist);
        return;
    }

    ROS_WARN("[%s] 🚗 已到达触发位置，执行换道重规划 ...", role_name_.c_str());

    // ===============================
    // 1. 从 CARLA Map 获取当前和目标 waypoint
    // ===============================
    auto carla_map = world_.GetMap();
    if (!carla_map) {
        ROS_ERROR("[%s] 获取 Map 失败，无法重规划", role_name_.c_str());
        return;
    }

    // 获取车辆当前位置的车道 waypoint
    auto current_wp = carla_map->GetWaypoint(current_loc, true);
    if (!current_wp) {
        ROS_ERROR("[%s] 无法获取当前车道 waypoint", role_name_.c_str());
        return;
    }
    
    auto next_wp = current_wp->GetNext(4.0).front();

    // 获取目标点右侧车道 waypoint（如果存在）
    auto goal_wp = carla_map->GetWaypoint(ros_pose_to_carla_transform(vehicle_config_.goal_point.pose).location, true);
    auto right_wp = goal_wp->GetRight();
  
    // ===============================
    // 2. 构造新的 ROS Pose 起点和终点
    // ===============================
    geometry_msgs::Pose start_pose = carla_transform_to_ros_pose(next_wp->GetTransform());
    geometry_msgs::Pose goal_pose = carla_transform_to_ros_pose(right_wp->GetTransform());

    // ===============================
    // 3. 调用路径服务 GetPath() 并带重试机制
    // ===============================
    auto start_time = std::chrono::system_clock::now();
    std::shared_ptr<driver_models_types::PathWithOptions> new_global_path = nullptr;

    while (ros::ok()) {
        auto new_global_path = GetPath(start_pose, goal_pose);
        if (new_global_path) {
            ROS_INFO("[%s] ✅ 成功获取右侧道路新路径 (%.2fm, %zu个点)",
                     role_name_.c_str(), dist, new_global_path->waypoints.size());

            global_path_ = new_global_path;
            local_planner_->set_global_plan(global_path_);

            ROS_INFO("[%s] 🛣️ 全局路径已更新为右侧道路路径。", role_name_.c_str());
            lane_change_triggered_ = true;  // ✅ 标记为已触发
            break;
        }

        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (duration > 5.0) {
            ROS_ERROR("[%s] ❌ 获取右侧路径超时 (>5s)，放弃重规划。", role_name_.c_str());
            return;
        }

        ROS_WARN("[%s] 重规划失败，%.1fs后重试...", role_name_.c_str(), 0.5);
        ros::Duration(0.5).sleep();
    }

    // ===============================
    // 4. 替换全局路径
    // ===============================
    if (lane_change_triggered_) {
        ROS_INFO("[%s] 🛣️ 全局路径已更新为右侧道路路径。", role_name_.c_str());
    } else {
        ROS_ERROR("[%s] ❌ 最终未能获取有效路径。", role_name_.c_str());
    }
}

} // namespace driver_model 