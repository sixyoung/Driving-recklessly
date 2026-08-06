#include "driver_models/auto_driver.h"
#include <filesystem> 
#include <std_msgs/String.h>
namespace fs = std::filesystem;

namespace driver_model {

AutoDriver::AutoDriver(ros::NodeHandle& nh, 
                         const driver_models_types::VehicleConfig &vehicle_config, 
                         const carla::client::World& world)
    : BaseDriver(nh, vehicle_config, world),
      cruise_speed_kph(vehicle_config_.cruise_speed.data * 3.6),
      scene_type_(vehicle_config.scene_type),
      Config_(),oba(Config_),
      EM(Config_) {
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
            {5, 5},
            {50, 5},
            {73, 5},
            {118, 5},
        
            {6, 6},
            {117, 6},
            {158, 6},
            {178, 6},

            {8, 8},
            {139, 8},
            {140, 8},
            {64, 8},

            {7, 7},
            {138, 7},
            {89, 7},
            {165, 7}
        };
        
        reset();
        Config_.role_name_ = vehicle_config.role_name;
        Config_.default_cruise_speed = vehicle_config.cruise_speed.data;
        Config_.planning_upper_speed_limit = vehicle_config.cruise_speed.data;  
        Config_.high_speed_threshold = 16.0; 
        Config_.low_speed_threshold = 15.0; 
        srand(static_cast<unsigned int>(time(nullptr)));
        // ================== 随意驾驶行为配置自动同步 ==================
        if (!vehicle_config.random_behavior.type.empty() && vehicle_config.random_behavior.type != "none")
        {
            const auto& rb = vehicle_config.random_behavior;
            ROS_WARN("[%s] 启用随意驾驶行为: %s", vehicle_config.role_name.c_str(), rb.type.c_str());
            for (size_t i = 0; i < rb.keys.size() && i < rb.values.size(); ++i)
            {
                const std::string& key = rb.keys[i];
                double value = rb.values[i];

                // 调用 Param_Configs 的反射式更新接口
                if (Config_.SetParamValueAuto(key, value)) {
                    ROS_INFO("✅ %s = %.3f", key.c_str(), value);
                } else {
                    ROS_WARN("未识别的参数名: %s", key.c_str());
                }
            }
        }
        if(!vehicle_config_.random_behavior.lane_change_point.empty()){
            lane_change_index_ = rand() % vehicle_config_.random_behavior.lane_change_point.size();
        }
        if(!vehicle_config_.random_behavior.change_speed_point.empty()){
            speed_change_index_ = rand() % vehicle_config_.random_behavior.change_speed_point.size();
        }

        double p = static_cast<double>(rand()) / RAND_MAX;

        if(vehicle_config.random_behavior.scene_type == "late_lane_change_at_highway_exit"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            speed = 35 + rand() % (35 - 25 + 1);
            lane_change_index_ = 0;    
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.last_cruise_speed = speed;
            Config_.last_planning_upper_speed_limit = speed;
            distance = (carla::geom::Location(vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));

        }  
        else if(vehicle_config.random_behavior.scene_type == "highway_normal_driving"){ 
            speed = 35 + rand() % (35 - 25 + 1);
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                lane_change_index_ = rand() % 4;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                lane_change_index_ = rand() % 4;
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                lane_change_index_ = rand() % 4;
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                lane_change_index_ = rand() % 4;
            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                lane_change_index_ = 3 + rand() % (4 - 3 + 1);

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                lane_change_index_ = 3 + rand() % (4 - 3 + 1);
            }
            if(p<0.2){
                Config_.abnormal =false;
                Config_.default_cruise_speed = speed;
                Config_.planning_upper_speed_limit = speed; 
                Config_.last_cruise_speed = speed;
                Config_.last_planning_upper_speed_limit = speed;
                Config_.lane_change = false;
            }else{
                Config_.abnormal = false;
                Config_.default_cruise_speed = speed;
                Config_.planning_upper_speed_limit = speed; 
                Config_.last_cruise_speed = speed;
                Config_.last_planning_upper_speed_limit = speed;
                distance = (carla::geom::Location(vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                        -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                        vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z).Distance(
                                                        carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                        -vehicle_config_.spawn_point.pose.position.y,
                                                        vehicle_config_.spawn_point.pose.position.z)));
            }

        } 
        else if(vehicle_config.random_behavior.scene_type == "tl_intersection_cut_in"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 10 + rand() % (14 - 10 + 1);
                if(speed == 10){
                    lane_change_index_ = rand() % 4;
                }else if(speed == 11){
                    const int candidates[] = {0, 1, 3};
                    lane_change_index_ = candidates[rand() % 3];
                }else if(speed == 12){
                    lane_change_index_ = 2;
                }else if(speed == 13){
                    const int candidates[] = {3, 4};
                    lane_change_index_ = candidates[rand() % 2];
                }else if(speed == 14){
                    const int candidates[] = {1, 2, 3};
                    lane_change_index_ = candidates[rand() % 3];
                }     
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 11 + rand() % (14 - 11 + 1);
                if(speed == 11){
                    lane_change_index_ = rand() % 4;
                }else if(speed == 12){
                    lane_change_index_ = rand() % 4;
                }else if(speed == 13){
                    lane_change_index_ = rand() % 4;
                }else if(speed == 14){
                    lane_change_index_ = rand() % 4;            
                }  
            }
            distance = (carla::geom::Location(vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.lane_change_cruise_speed = speed;
            Config_.lane_change_planning_upper_speed_limit = speed;

        }
        else if(vehicle_config.random_behavior.scene_type == "intersection_normal_driving"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 12 + rand() % (12 - 11 + 1);
                if(speed == 12){
                    const int candidates[] = {0, 1, 3};
                    lane_change_index_ = candidates[rand() % 3];
                }else if(speed == 13){
                    const int candidates[] = {0, 1};
                    lane_change_index_ = candidates[rand() % 2];
                }else if(speed == 14){
                    lane_change_index_ = 0;
                }     
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 10;
                lane_change_index_ = rand() % 4;
            }
            Config_.abnormal = false;
            distance = (carla::geom::Location(vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.lane_change_cruise_speed = speed;
            Config_.lane_change_planning_upper_speed_limit = speed;

        }
        else if(vehicle_config.random_behavior.scene_type == "tl_intersection_lane_change"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;

            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 12 + rand() % (15 - 12 + 1);
                lane_change_index_ = rand() % 5;
            }
            distance = (carla::geom::Location(vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.last_cruise_speed = speed;
            Config_.last_planning_upper_speed_limit = speed;

        }    
        else if(vehicle_config.random_behavior.scene_type == "intersection_sudden_acceleration"){

            // if(p<0.1){
            //     speed = 8 + rand() % (12 - 8 + 1);
            //     Config_.default_cruise_speed = speed;
            //     Config_.planning_upper_speed_limit = speed; 
            //     Config_.abnormal =false;
            // }else{
            //     speed = 28 + rand() % (37 - 28 + 1);
            //     distance = (carla::geom::Location(vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
            //                                             -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
            //                                             vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z).Distance(
            //                                             carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
            //                                             -vehicle_config_.spawn_point.pose.position.y,
            //                                             vehicle_config_.spawn_point.pose.position.z)));
            // }
            // Config_.speed_change_cruise_speed = speed;
            // Config_.speed_change_planning_upper_speed_limit = speed;
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = rand() % 5;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = 1 + rand() % ((5 - 1 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = 2 + rand() % ((5 - 2 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));


            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 28 + rand() % (40 - 28 + 1);
                speed_change_index_ = 4 + rand() % ((5 - 4 + 1));
            }

            distance = (carla::geom::Location(vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;

        }
        else if(vehicle_config.random_behavior.scene_type == "intersection_sudden_deceleration"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = rand() % 5;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = 2 + rand() % ((5 - 2 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = 3+ rand() % ((5 - 3 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 9 + rand() % (15 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));
            }
            distance = (carla::geom::Location(vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.speed_change_cruise_speed = 0;
            Config_.speed_change_planning_upper_speed_limit = 20;
            Config_.last_cruise_speed = speed;
            Config_.last_planning_upper_speed_limit = speed;

        }
        else if(vehicle_config.random_behavior.scene_type == "signalized_intersection_normal_acceleration"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = rand() % 5;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = 2 + rand() % ((5 - 2 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = 3+ rand() % ((5 - 3 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));
            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 9 + rand() % (12 - 9 + 1);
                speed_change_index_ = 3 + rand() % ((5 - 3 + 1));
            }
            Config_.abnormal = false;
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
            Config_.last_cruise_speed = speed;
            Config_.last_planning_upper_speed_limit = speed;
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 

        }
        else if(vehicle_config.random_behavior.scene_type == "untl_intersection_left_ignore_straight"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 9 + rand() % (15 - 9 + 1);    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 9 + rand() % (15 - 9 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 10 + rand() % (13 - 10 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 11 + rand() % (15 - 11 + 1);
            }
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "tl_intersection_over_speed"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = rand() % 4;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = 1 + rand() % (3 - 1 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = 1 + rand() % (3 - 1 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = 4 + rand() % (6 - 4 + 1);

            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = 5 + rand() % (6 - 5 + 1);

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 30 + rand() % (50 - 30 + 1);
                speed_change_index_ = 5 + rand() % (6 - 5 + 1);
            }
            distance = (carla::geom::Location(vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;  
        }
        else if(vehicle_config.random_behavior.scene_type == "untl_intersection_abnormal_speed"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;

            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = rand() % 5;    
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = rand() % 3;
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = rand() % 2;
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = 6 + rand() % (9 - 6 + 1);

            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = 7 + rand() % (9 - 7 + 1);

            }
            else if(vehicle_config.spawn_key == "spawn_point5"){
                speed = 2 + rand() % (5 - 2 + 1);
                speed_change_index_ = 8 + rand() % (9 - 8 + 1);
            }
            distance = (carla::geom::Location(vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z).Distance(
                                                    carla::geom::Location(vehicle_config_.spawn_point.pose.position.x,
                                                    -vehicle_config_.spawn_point.pose.position.y,
                                                    vehicle_config_.spawn_point.pose.position.z)));
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;

        }
        else if(vehicle_config.random_behavior.scene_type == "tl_intersection_ignore_red"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 10 + rand() % (13 - 10 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 8 + rand() % (12 - 8 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 8 + rand() % (10 - 8 + 1);
            }
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "tl_intersection_ignore_yellow"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 9 + rand() % (13 - 9 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 9 + rand() % (12 - 9 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 9 + rand() % (12 - 9 + 1);
            }
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "merge_onramp_abnormal_speed"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 6 + rand() % (10 - 6 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 6 + rand() % (10 - 6 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 6 + rand() % (10 - 6 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point3"){
                speed = 6 + rand() % (10 - 6 + 1);
            }
            else if(vehicle_config.spawn_key == "spawn_point4"){
                speed = 6 + rand() % (10 - 6 + 1); 
            } 
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "merge_onramp_normol_speed"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            speed = 20 + rand() % (25 - 20 + 1); 
            Config_.abnormal =false;
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "signalized_intersection_signal_compliant"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            Config_.abnormal =false;
            speed = 8 + rand() % (15 - 8 + 1); 
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }
        else if(vehicle_config.random_behavior.scene_type == "merge_onramp_over_speed"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 21 + rand() % (23 - 21 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 22 + rand() % (25 - 22 + 1); 
            } 
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 20 + rand() % (21 - 20 + 1); 
            } 
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        }        
        else if(vehicle_config.random_behavior.scene_type == "merge_onramp_over_speed_normol"){
            spawn_position_.x = vehicle_config_.spawn_point.pose.position.x;
            spawn_position_.y = -vehicle_config_.spawn_point.pose.position.y;
            spawn_position_.z = vehicle_config_.spawn_point.pose.position.z;
            if(vehicle_config.spawn_key == "spawn_point0"){
                speed = 18 + rand() % (22 - 18 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point1"){
                speed = 18 + rand() % (19 - 18 + 1); 
            }
            else if(vehicle_config.spawn_key == "spawn_point2"){
                speed = 18 + rand() % (19 - 18 + 1); 
            }
            Config_.abnormal =false;
            Config_.default_cruise_speed = speed;
            Config_.planning_upper_speed_limit = speed; 
            Config_.speed_change_cruise_speed = speed;
            Config_.speed_change_planning_upper_speed_limit = speed;
        } 
        Initialize();
}

AutoDriver::~AutoDriver() {
    destroy();
}

void AutoDriver::Initialize() {
    // 初始化基本设置
    InitializeROSResources();
    // 订阅物体信息
    if(Config_.avoid_vehicle_) {   
        objects_sub = nh_.subscribe("/carla/objects", 1, &AutoDriver::objects_call_back, this);    
    }
    if(Config_.avoid_traffic_light_){
        traffic_lights_phases_sub = nh_.subscribe("/traffic_light/phases", 1, &AutoDriver::trafficlight_phase_call_back, this);    
    }
    reckless_pub_ = nh_.advertise<std_msgs::String>("/reckless_detection/result", 10 , true);
    // 构造专属调度话题名称
    std::string topic_name = "/scheduler/vehicle_" + std::to_string(vehicle_config_.carla_id);
    scheduler_sub_ = nh_.subscribe<road_side_system_type::PoseWithTimeWindowArray>(
        topic_name, 1, &AutoDriver::schedulerCallback, this);

    // 获取路径并设置全局规划
    auto start_time = std::chrono::system_clock::now();
    while (ros::ok()) {
        global_path_ = GetPath(vehicle_config_.spawn_point.pose, vehicle_config_.goal_point.pose);
        global_paths_.push_back(global_path_);
        if(Config_.lane_change){ 
            const auto& other_lanes = vehicle_config_.random_behavior.other_lanes;
            for (size_t i = 0; i < other_lanes.size(); ++i){
                const auto& lane = other_lanes[i];
                auto new_path = GetPath(lane.start_point, lane.end_point);
                if (new_path) {
                    global_paths_.push_back(new_path);
                    ROS_INFO("[%s] 成功生成并保存新路径，global_paths_ 总数 = %zu", 
                            role_name_.c_str(), global_paths_.size());
                } else {
                    ROS_ERROR("[%s] 其他路径生成失败！", role_name_.c_str());
                }
            }
        }
        vehicle_carla_actor_ = boost::dynamic_pointer_cast<carla::client::Vehicle>(world_.GetActor(id_));
        
        if (global_path_ && vehicle_carla_actor_) {
            break; 
        }

        auto current_time = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (duration > 5.0) {
            break;
        }

        ROS_WARN("%s(%d):初始化驾驶员模型过程中获取global_path 和 carla_actor 失败，正在重试...", role_name_.c_str(), id_);
        ros::Duration(0.1).sleep(); 
    }

    if (global_path_ && vehicle_carla_actor_) {
        local_planner_->set_global_plan(global_path_);
        global_path_updated_ = true;
        last_matched_point_idx_ = -1;
        has_first_matched_point_ = false;
        ROS_INFO("%s(%d):车辆初始化完毕！", role_name_.c_str(), id_);
    }else{
        ROS_ERROR("%s(%d):车辆初始化超时(5s),车辆初始化失败！", role_name_.c_str(), id_);
        local_planner_->set_reached_goal(true); 
        if (!destroy_carla_object(id_)) {
            ROS_WARN("%s(%d):尝试销毁车辆失败，请检查 DestroyObject 服务状态！", role_name_.c_str(), id_);
        }else{
            ROS_INFO("%s(%d):销毁车辆成功！", role_name_.c_str(), id_);
        }
    }
}

void AutoDriver::objects_call_back(const derived_object_msgs::ObjectArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> objects_lock(objects_mutex_);
    carla_objects_.clear();
    carla_objects_.reserve(msg->objects.size());  // 预分配，避免多次扩容
    for (const auto& obj : msg->objects) {
        carla_objects_.push_back(obj);  // 顺序存储
    }
}

void AutoDriver::trafficlight_phase_call_back(const road_side_system::TrafficLightPhaseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> traffic_light_lock(traffic_light_mutex_);
    traffic_light_map_.clear();
    if (msg->phases.empty()) return;
    for (const auto& phase : msg->phases) {
        traffic_light_map_[phase.road_id] = phase;
    }
}

void AutoDriver::run_step() {
    std::lock_guard<std::mutex> objects_lock(objects_mutex_);
    std::lock_guard<std::mutex> traffic_light_lock(traffic_light_mutex_);
    Config_.frame_id++;
    if (is_destroyed_ || !vehicle_carla_actor_ || !global_path_||
                        !local_planner_->get_odometry_received_flag() ||
                        local_planner_->get_reached_goal_flag()) {return;}
    std_msgs::String msg;
    std_msgs::String ctx_msg;
    double now = ros::Time::now().toSec();
    double elapsed = now-0.0;

    if((elapsed + (static_cast<double>(rand()) / RAND_MAX) * 2.0) > 12 && !Config_.abnormal && !Config_.elapsed_time && vehicle_config_.is_random_behavior_vehicle){
        std::string spawn_pos_str;
        if (std::fabs(spawn_position_.x) < 1e-6 &&
            std::fabs(spawn_position_.y) < 1e-6 &&
            std::fabs(spawn_position_.z) < 1e-6) {
            spawn_pos_str = "无";
        } else {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1)
                << "x=" << spawn_position_.x
                << ",y=" << spawn_position_.y
                << ",z=" << spawn_position_.z;
            spawn_pos_str = oss.str();
        }
        // distance 字符串
        std::string distance_str =
            (distance <= 1e-6) ? "无" : std::to_string(distance);

        // speed 字符串（如果你有 speed=0 表示无效，也可以改判断）
        std::string speed_str =
            (speed <= 0) ? "无" : std::to_string(speed);

        msg.data =
            std::string("未识别随意驾驶场景") + "/" +
            "未识别随意驾驶车辆" + "/" +
            "无" + "/" +
            std::string("正常场景") + "/" +
            "无随意驾驶车辆" + "/" +
            speed_str + "/" +
            distance_str + "/" +
            spawn_pos_str;
        reckless_pub_.publish(msg);
        Config_.elapsed_time = true;
    }
    if (Config_.lane_change) {
        if(current_lane_change_index_ >= 1){
            Config_.lane_change = false;
            return;
        }
        // 当前车辆位置
        auto current_transform = vehicle_carla_actor_->GetTransform();
        carla::geom::Location current_loc = current_transform.location;
        carla::geom::Location target_point = carla::geom::Location(
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.y,
                                                    vehicle_config_.random_behavior.lane_change_point[lane_change_index_].position.z);

        // 判断是否到达触发区域（1米范围内）
        double dist = current_loc.Distance(target_point);
        if (dist <= 2.0) {
            if (trigger_time_first_ < 0.0) {
                if(ros::Time::now().toSec() < 17.0){
                    double noise = -0.5 + (static_cast<double>(rand()) / RAND_MAX) * 2.5;
                    trigger_time_first_ = ros::Time::now().toSec() + noise; 
                }else{
                    trigger_time_first_ = ros::Time::now().toSec();
                }
            }
            if(Config_.abnormal){
                std::string spawn_pos_str;
                if (std::fabs(spawn_position_.x) < 1e-6 &&
                    std::fabs(spawn_position_.y) < 1e-6 &&
                    std::fabs(spawn_position_.z) < 1e-6) {
                    spawn_pos_str = "无";
                } else {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(1)
                        << "x=" << spawn_position_.x
                        << ",y=" << spawn_position_.y
                        << ",z=" << spawn_position_.z;
                    spawn_pos_str = oss.str();
                }
            // distance 字符串
                std::string distance_str =
                    (distance <= 1e-6) ? "无" : std::to_string(distance);

                // speed 字符串（如果你有 speed=0 表示无效，也可以改判断）
                std::string speed_str =
                    (speed <= 0) ? "无" : std::to_string(speed);
                msg.data = vehicle_config_.scene_class + "/" + 
                    role_name_ + "/" + std::to_string(trigger_time_first_) + "/" + 
                    vehicle_config_.scene_class + "/" + 
                    role_name_ + "/" + speed_str + "/" + 
                    distance_str+ "/" + spawn_pos_str;
                reckless_pub_.publish(msg);

            }
            ROS_WARN("[%s] 🚗 已到达触发位置，执行换道重规划 ...", role_name_.c_str());
            local_planner_->set_global_plan(global_paths_[current_lane_change_index_ + 1]);
            global_path_ = global_paths_[current_lane_change_index_ + 1];
            global_path_updated_ = true;
            last_matched_point_idx_ = -1;
            has_first_matched_point_ = false;
            Config_.IsChangeLanePath = true;
            Config_.default_cruise_speed=Config_.lane_change_cruise_speed;
            Config_.planning_upper_speed_limit=Config_.lane_change_planning_upper_speed_limit;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";        // 或其他坐标系
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose = global_path_->waypoints.back().pose; 
            local_planner_->set_goal_point(pose_stamped);
            current_lane_change_index_++;
        }else{
            ROS_INFO("[%s] 未到达触发位置(%.2f m)，等待中...", role_name_.c_str(), dist);
        }
    }
    if(Config_.speed_abnormal){
        auto current_transform = vehicle_carla_actor_->GetTransform();
        carla::geom::Location current_loc = current_transform.location;
        carla::geom::Location target_point = carla::geom::Location(
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                    -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                    vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z);
        ROS_INFO("[%s] 当前位置: (%.3f, %.3f, %.3f)", 
                role_name_.c_str(),
                current_loc.x, current_loc.y, current_loc.z);

        ROS_INFO("[%s] change_speed_point 触发点: (%.3f, %.3f, %.3f)", 
        role_name_.c_str(),
        target_point.x, target_point.y, target_point.z);
        // 判断是否到达触发区域（1米范围内）
        double dist = current_loc.Distance(target_point);
        if (dist <= 2.0 ) {
            ROS_WARN("[%s] 🚗 已到达触发位置，执行突然变速 ...", role_name_.c_str());
            if (trigger_time_first_ < 0.0) {
                if(ros::Time::now().toSec() < 17.0){
                    double noise = -0.5 + (static_cast<double>(rand()) / RAND_MAX) * 2.5;
                    trigger_time_first_ = ros::Time::now().toSec() + noise; 
                }else{
                    trigger_time_first_ = ros::Time::now().toSec();
                }
            }
            Config_.default_cruise_speed = Config_.speed_change_cruise_speed;
            Config_.planning_upper_speed_limit = Config_.speed_change_planning_upper_speed_limit;
            if(Config_.abnormal){
                std::string spawn_pos_str;
                if (std::fabs(spawn_position_.x) < 1e-6 &&
                    std::fabs(spawn_position_.y) < 1e-6 &&
                    std::fabs(spawn_position_.z) < 1e-6) {
                    spawn_pos_str = "无";
                } else {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(1)
                        << "x=" << spawn_position_.x
                        << ",y=" << spawn_position_.y
                        << ",z=" << spawn_position_.z;
                    spawn_pos_str = oss.str();
                }
                    // distance 字符串
                std::string distance_str =
                    (distance <= 1e-6) ? "无" : std::to_string(distance);

                // speed 字符串（如果你有 speed=0 表示无效，也可以改判断）
                std::string speed_str =
                    (speed <= 0) ? "无" : std::to_string(speed);
                msg.data = vehicle_config_.scene_class + "/" + 
                    role_name_ + "/" + std::to_string(trigger_time_first_) + "/" + 
                    vehicle_config_.scene_class + "/" + 
                    role_name_ + "/" + speed_str + "/" + 
                    distance_str+ "/" + spawn_pos_str;
                reckless_pub_.publish(msg);
            }
            Config_.speed_abnormal =false;
        }
    }
    if(Config_.sudden_speed_change){
        if(!Config_.change_speed){
            // 当前车辆位置
            auto current_transform = vehicle_carla_actor_->GetTransform();
            carla::geom::Location current_loc = current_transform.location;
            carla::geom::Location target_point = carla::geom::Location(
                                                        vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.x,
                                                        -vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.y,
                                                        vehicle_config_.random_behavior.change_speed_point[speed_change_index_].position.z);

            ROS_INFO("[%s] 当前位置: (%.3f, %.3f, %.3f)", 
                    role_name_.c_str(),
                    current_loc.x, current_loc.y, current_loc.z);

            ROS_INFO("[%s] change_speed_point 触发点: (%.3f, %.3f, %.3f)", 
            role_name_.c_str(),
            target_point.x, target_point.y, target_point.z);
            // 判断是否到达触发区域（1米范围内）
            double dist = current_loc.Distance(target_point);
            if (dist <= 2.0 ) {
                ROS_WARN("[%s] 🚗 已到达触发位置，执行突然变速 ...", role_name_.c_str());
                if (trigger_time_first_ < 0.0) {
                    if(ros::Time::now().toSec() < 17.0){
                        double noise = -0.5 + (static_cast<double>(rand()) / RAND_MAX) * 2.5;
                        trigger_time_first_ = ros::Time::now().toSec() + noise; 
                    }else{
                        trigger_time_first_ = ros::Time::now().toSec();
                    }
                }
                if(Config_.abnormal){
                    std::string spawn_pos_str;
                    if (std::fabs(spawn_position_.x) < 1e-6 &&
                        std::fabs(spawn_position_.y) < 1e-6 &&
                        std::fabs(spawn_position_.z) < 1e-6) {
                        spawn_pos_str = "无";
                    } else {
                        std::ostringstream oss;
                        oss << std::fixed << std::setprecision(1)
                            << "x=" << spawn_position_.x
                            << ",y=" << spawn_position_.y
                            << ",z=" << spawn_position_.z;
                        spawn_pos_str = oss.str();
                    }
                     // distance 字符串
                    std::string distance_str =
                        (distance <= 1e-6) ? "无" : std::to_string(distance);

                    // speed 字符串（如果你有 speed=0 表示无效，也可以改判断）
                    std::string speed_str =
                        (speed <= 0) ? "无" : std::to_string(speed);
                    msg.data = vehicle_config_.scene_class + "/" + 
                        role_name_ + "/" + std::to_string(trigger_time_first_) + "/" + 
                        vehicle_config_.scene_class + "/" + 
                        role_name_ + "/" + speed_str + "/" + 
                        distance_str+ "/" + spawn_pos_str;
                    reckless_pub_.publish(msg);
                }
                Config_.default_cruise_speed = Config_.speed_change_cruise_speed;
                Config_.planning_upper_speed_limit = Config_.speed_change_planning_upper_speed_limit;
                Config_.change_speed = true;
                // 如果使用按时间恢复，则开始计时
                if (!Config_.use_recovery_by_point)
                {
                    Config_.recovery_start_time = ros::Time::now().toSec();
                    ROS_INFO("[%s] ⏱ 速度恢复计时开始...", role_name_.c_str());
                }
            }else{
                ROS_INFO("[%s] 未到达触发位置(%.2f m)，等待中...", role_name_.c_str(), dist);
            }   
        }else if(Config_.change_speed && !Config_.recovery_speed){
            if (Config_.use_recovery_by_point){
                // 当前车辆位置
                auto current_transform = vehicle_carla_actor_->GetTransform();
                carla::geom::Location current_loc = current_transform.location;
                carla::geom::Location target_point = carla::geom::Location(
                                                            vehicle_config_.random_behavior.recovery_speed_point[0].position.x,
                                                            -vehicle_config_.random_behavior.recovery_speed_point[0].position.y,
                                                            vehicle_config_.random_behavior.recovery_speed_point[0].position.z);
                // 判断是否到达触发区域（1米范围内）
                double dist = current_loc.Distance(target_point);
                if (dist <= 1.0 ) {
                    ROS_WARN("[%s] 🚗 已到达触发位置，执行速度恢复 ...", role_name_.c_str());
                    Config_.default_cruise_speed = Config_.last_cruise_speed;
                    Config_.planning_upper_speed_limit = Config_.last_planning_upper_speed_limit;
                    Config_.recovery_speed = true;
                    Config_.change_speed = false;
                    Config_.sudden_speed_change = false;
                    Config_.recovery_start_time = ros::Time::now().toSec();
                }else{
                    ROS_INFO("[%s] 未到达触发位置(%.2f m)，等待中...", role_name_.c_str(), dist);

                }
            }else{
                double now = ros::Time::now().toSec();
                double elapsed = now - Config_.recovery_start_time;

                ROS_INFO("[%s] 恢复检查（计时模式）：已等待 %.2f 秒 / 目标 %.2f 秒",
                        role_name_.c_str(), elapsed, Config_.recovery_wait_time);

                if (elapsed >= Config_.recovery_wait_time)
                {
                    ROS_WARN("[%s] ⏱ 恢复时间到 — 速度恢复！", role_name_.c_str());
                    Config_.default_cruise_speed = Config_.last_cruise_speed;
                    Config_.planning_upper_speed_limit = Config_.last_planning_upper_speed_limit;

                    Config_.recovery_speed = true;
                    Config_.change_speed   = false;
                    Config_.sudden_speed_change = false;
                }
            }
        }     
    }

    update(vehicle_carla_actor_);
    PerceiveEnvironment();
    dynamic_routing();
}

void AutoDriver::update(carla::SharedPtr<carla::client::Vehicle> vehicle) {
    // 1. 获取车辆基本信息
    uodate_vehicle_info();
    // 2. 更新车辆所在路径点
    update_path_waypoint();
    // 3. 确定车辆行驶状态
    update_intention();
    // 5. 根据状态更新转向灯
    update_vehicle_light(true, vehicle);
}

void AutoDriver::PerceiveEnvironment() {
    AllObstacle.clear();
    const double DIRECTION_THRESHOLD_DEG = 20.0; 
    const double COS_THRESHOLD = std::cos(DIRECTION_THRESHOLD_DEG * M_PI / 180.0);
    if (carla_objects_.empty() || ego_path.empty() || !ego_wpt) {
        return;
    }
    // 遍历所有检测到的物体
    for (const auto& obj : carla_objects_) {
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
        if (distance > Config_.detection_range) {
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

        if ((std::find(vehicle_config_.random_behavior.ignored_vehicle_roles.begin(),
                    vehicle_config_.random_behavior.ignored_vehicle_roles.end(),
                    target_role_name)
            != vehicle_config_.random_behavior.ignored_vehicle_roles.end()) && Config_.abnormal )
        {
            continue;
        }
        const carla::geom::Transform tar_transform = tar_vehicle->GetTransform();
        // 排除不在车辆前方的目标车辆
        if (!is_within_distance_and_angle(ego_transform, tar_transform, Config_.detection_range, 0.0, 90.0)) {
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

        std::vector<carla::geom::Location> tar_corners;
        double tar_length_ = 4.8;
        double tar_width_ = 2.1;
        // 目标车辆的尺寸和corners和多边形
        if (obj.shape.dimensions.size() < 2) {
             tar_corners = GetRectangleCorners(tar_transform, tar_length_, tar_width_);
        } else {
             tar_length_ = obj.shape.dimensions[0];
             tar_width_  = obj.shape.dimensions[1];
             tar_corners = GetRectangleCorners(tar_transform, tar_length_, tar_width_);
        }

        // Polygon tar_polygon;
        // for (const auto& pt : tar_corners) {
        //     bg::append(tar_polygon.outer(), Point(pt.x, pt.y));
        // }
        // bg::correct(tar_polygon);

        // // 目标车辆的速度信息
        // carla::geom::Vector3D tar_speed(obj.twist.linear.x, -obj.twist.linear.y, obj.twist.linear.z);
        // double tar_speed_mps = std::sqrt(tar_speed.x * tar_speed.x + tar_speed.y * tar_speed.y + tar_speed.z * tar_speed.z);

        // 获取目标车辆的灯光状态
        int tar_light_state = static_cast<int>(GetCurrentLightState(tar_vehicle));

        // 预测目标车辆的未来路径
        std::vector<carla::geom::Transform> target_predicted_path_;
        if (!(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
            !(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("🚘 [预测] 目标车辆无转向灯，预测直行路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_transform, tar_id, "straight", 1.0, 50);
        } else if ((tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
                !(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("↩️ [预测] 目标车辆打左转灯，预测左转路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_transform, tar_id, "left",  1.0, 50);
        } else if (!(tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::LeftBlinker)) && 
                (tar_light_state & static_cast<int>(carla::client::Vehicle::LightState::RightBlinker))) {
            ROS_DEBUG_STREAM("↪️ [预测] 目标车辆打右转灯，预测右转路径");
            target_predicted_path_ = get_target_path(tar_wpt, tar_transform, tar_id, "right",  1.0, 50);
        } else {
            ROS_DEBUG_STREAM("⚠️ [预测] 双闪或未知转向状态，默认预测直行");
            target_predicted_path_ = get_target_path(tar_wpt, tar_transform, tar_id, "straight",  1.0, 50);
        }

        // 插值并存入结果
        std::vector<carla::geom::Transform> tar_interp_path;
        interpolate_path(target_predicted_path_, 1.0, tar_interp_path);
        Polygon tar_path_poly = PathToPolygon(tar_interp_path, tar_width_ / 2.0);

        // 如果目标车辆在自车路径上
        if (is_box_intersect_path(ego_path_poly, tar_transform, tar_length_, tar_width_)){
            // 检查相互阻挡情况
            if (is_box_intersect_path(tar_path_poly, ego_transform, ego_length, ego_width) && 
                is_box_intersect_path(ego_path_poly, tar_transform, tar_length_, tar_width_)) {
                ROS_ERROR_STREAM("[相互阻挡路径] 自车" << vehicle_config_.carla_id << "目标车ID: " << obj.id);
            }
            geometry_msgs::PoseArray tar_interp_ros_path = carla_path_to_ros_posearray(tar_interp_path);
            creat_obstacle(AllObstacle, obj, tar_interp_ros_path);
        }else{
            if (is_box_intersect_path(tar_path_poly, ego_transform, ego_length, ego_width)) {
                continue;
            }
            // 检查路径是否相交 
            auto [has_intersection, points] = CheckPolygonIntersection(ego_path_poly, tar_path_poly);
            if(has_intersection){
                geometry_msgs::PoseArray tar_interp_ros_path = carla_path_to_ros_posearray(tar_interp_path);
                creat_obstacle(AllObstacle, obj, tar_interp_ros_path);
            }else{
                continue;
            }
        }
    }
}

/**
 * @brief 更新车辆基本信息、路径点和驾驶员意图
 */
void AutoDriver::uodate_vehicle_info() {
    ego_length = vehicle_carla_actor_->GetBoundingBox().extent.x * 2.0;
    ego_width = vehicle_carla_actor_->GetBoundingBox().extent.y * 2.0;
    ego_pose = local_planner_->get_current_pose_();
    ego_odmo = local_planner_->get_odom_();
    ego_transform = ros_pose_to_carla_transform(ego_pose);
    ego_forward = ego_transform.GetForwardVector().MakeUnitVector();
    current_speed_ = local_planner_->get_current_speed_();

    ego_corners = GetRectangleCorners(ego_transform, ego_length, ego_width);
    ego_polygon.outer().clear();

    for (const auto& pt : ego_corners) {
        bg::append(ego_polygon.outer(), Point(pt.x, pt.y));
    }
    bg::correct(ego_polygon);
    // if (role_name_ == "hero0" || role_name_ == "hero100" || role_name_ == "hero400") {

    //     DrawPolygon(ego_polygon, *debug_helper_, ego_transform.location.z+0.5, 0.3f, 1.0f, {255, 0, 0});

    // }
    if (vehicle_carla_actor_) {
        auto location = vehicle_carla_actor_->GetTransform().location;
        location.z += 2.5;
        std::string text = role_name_ + "[" + std::to_string(id_) + "]";
        debug_helper_->DrawString(
            location,
            text,
            false, 
            carla::client::DebugHelper::Color{0U, 0U, 255U},
            0.1f, 
            false 
        );
    }
}
        
/**
 * @brief 更新车辆所在路径点、局部路径和多边形以及车头位置在局部路径上的投影
 */
void AutoDriver::update_path_waypoint() {
    if(global_path_updated_ == true){
        path_points.clear();
        
        for(int i = 0; i < global_path_->waypoints.size(); ++i){
            path_points.emplace_back(ros_pose_to_carla_transform(global_path_->waypoints[i].pose), 
            static_cast<local_planner::RoadOption>(global_path_->waypoints[i].road_option));
        }
        global_path_updated_ = false;
    }
    const double cos_threshold = std::cos(20.0 * M_PI / 180.0);  // 合理方向差余弦阈值
    ego_wpt = find_match_waypoint(id_, ego_transform, cos_threshold, 1.0, last_valid_ego_wpts_);
    interp_path.clear();
    ego_path = get_ego_path(50);
    // 插值并存入结果
    interpolate_path(ego_path, 1.0, interp_path);
    // 将路径转换为多边形，用于碰撞检测
    ego_path_poly = PathToPolygon(interp_path, ego_width/2.0);

    if (!bg::is_valid(ego_path_poly)) {
        std::ostringstream oss;
        oss << "[" << role_name_ << "]: Invalid polygon, path points: ";
        const auto& ring = ego_path_poly.outer();
        for (size_t i = 0; i < ring.size(); ++i) {
            oss << "(" << std::fixed << std::setprecision(2) 
                << ring[i].x() << ", " << ring[i].y() << ")";
            if (i != ring.size() - 1) oss << ", ";
        }
        ROS_WARN("%s", oss.str().c_str());

        ROS_WARN("[%s] front_path: x,y,right_x,right_y", role_name_.c_str());

        for (const auto& tf : interp_path) {
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
void AutoDriver::update_intention() {
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
void AutoDriver::update_vehicle_light(bool is_active, carla::SharedPtr<carla::client::Vehicle> vehicle) {
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


/**
 * @brief 确定车辆的行驶选项
 * @return 返回路径点和行驶选项的pair，如果无效则返回nullptr和VOID
 */
std::shared_ptr<std::pair<carla::geom::Transform, local_planner::RoadOption>> AutoDriver::get_road_option() {
    // 检查是否在路口
    bool is_junction = ego_wpt && ego_wpt->IsJunction();
    if (is_junction) {
        // 在路口时，检查下一个路径点 
        auto next_trans = path_points[last_matched_point_idx_ + 3 < path_points.size() ? last_matched_point_idx_ + 3 : path_points.size() - 1].first;
        auto next_wpt = world_.GetMap()->GetWaypoint(next_trans.location);
        bool next_is_junction = next_wpt && next_wpt->IsJunction();
        
        if (!next_is_junction) {
            // 向前检查4个点之前的位置，防止越界
            size_t prev_idx0 = (last_matched_point_idx_ >= 4) 
                                ? last_matched_point_idx_ - 4 
                                : 0;
            // 返回一个 shared_ptr<pair>
            return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                path_points[prev_idx0].first, path_points[prev_idx0].second);
        }else {
            // 如果下一个点还是路口，看当前位置
            return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                path_points[last_matched_point_idx_].first, path_points[last_matched_point_idx_].second);
        }

    } else {
        // 不在路口时，检查前一个路径点
        size_t prev_idx1 = (last_matched_point_idx_ >= 4) 
                            ? last_matched_point_idx_ - 4 
                            : 0;
        auto last_trans = path_points[prev_idx1].first;
        auto last_wpt = world_.GetMap()->GetWaypoint(last_trans.location);
        bool last_is_junction = last_wpt && last_wpt->IsJunction();
        
        if (last_is_junction) {
            // 如果前一个点是路口，往回看
            size_t prev_idx1 = (last_matched_point_idx_ >= 4) 
                                ? last_matched_point_idx_ - 4 
                                : 0;
            // 返回一个 shared_ptr<pair>
            return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                path_points[prev_idx1].first, path_points[prev_idx1].second);
        }else {
            // 🚗 前一个点不是路口 -> 先判断是否处于 ego 的左右车道
            const auto &target_tf = path_points[last_matched_point_idx_].first;
            carla::geom::Location target_loc = target_tf.location;
            // if(role_name_ == "hero0"){
            //     // 🔹 绘制一个蓝色小球
            //     debug_helper_->DrawPoint(
            //         target_loc,                           // 坐标
            //         0.2f,                                 // 半径（米）
            //         {0, 0, 255},              // 蓝色
            //         0.1f,                                 // 持续时间（秒）
            //         true                                  // persistent_lines=true 表示持续可见
            //     );
            // }

            // 获取 path_points[last_matched_point_idx_] 对应的 waypoint
            auto target_wp = world_.GetMap()->GetWaypoint(target_loc);
            if (!target_wp) {
                ROS_WARN("未能从 path_points[%zu] 获取有效 waypoint，使用前向点。", last_matched_point_idx_);
            }

            bool is_left_lane = false;
            bool is_right_lane = false;

            if (ego_wpt && target_wp) {
                // 获取 ego 的左右车道 waypoint
                auto left_wp = ego_wpt->GetLeft();
                auto right_wp = ego_wpt->GetRight();

                // 判断是否相同道路段、相邻车道
                if (left_wp && (left_wp->GetLaneId() == target_wp->GetLaneId())) {
                    is_left_lane = true;
                } else if (right_wp && (right_wp->GetLaneId() == target_wp->GetLaneId())) {
                    is_right_lane = true;
                }
            }           
            if (is_left_lane) {
                ROS_INFO("[get_target_path] 目标点位于ego的车道 (idx=%zu) 左侧", last_matched_point_idx_);
                return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                    path_points[last_matched_point_idx_].first,
                    local_planner::RoadOption::CHANGELANELEFT);
            }else if(is_right_lane){
                ROS_INFO("[get_target_path] 目标点位于ego的车道 (idx=%zu) 右侧", last_matched_point_idx_);
                return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                    path_points[last_matched_point_idx_].first,
                    local_planner::RoadOption::CHANGELANERIGHT);
            }
            // 🚗 否则，按原逻辑往前找 20 个点
            size_t pro_idx = (last_matched_point_idx_ + 20 < path_points.size()) 
                                ? last_matched_point_idx_ + 20 
                                : path_points.size()-1;
            // 返回一个 shared_ptr<pair>
            return std::make_shared<std::pair<carla::geom::Transform, local_planner::RoadOption>>(
                path_points[pro_idx].first, path_points[pro_idx].second);
        }
    }
    return nullptr;
}

/**
 * @brief 获取自车路径
 * @param num_points 点数
 * @return 路径
 */

std::vector<carla::geom::Transform> AutoDriver::get_ego_path(int num_points) {
    std::vector<carla::geom::Transform> path;
    if (path_points.empty()) {
        ROS_WARN("[%s] path_points 为空，无法生成 ego 路径。", role_name_.c_str());
        return path;
    }

    const auto &E = ego_transform.location;  // 自车位置

    // λ: 计算两点间距离平方
    auto sq_dist = [](const carla::geom::Location &a, const carla::geom::Location &b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    };

    int best_idx = -1;

    // ======================================================
    //  🚗 Step 1️⃣：首次匹配 —— 使用“极小点 + 连续上升趋势检测”逻辑
    // ======================================================
    if (!has_first_matched_point_) {
        bool found_min = false;
        int increase_in_count = 0;
        int min_idx = -1;

        for (int i = 0; i < static_cast<int>(path_points.size()) - 1; ++i) {
            double l_prev = (i == 0) ? std::numeric_limits<double>::infinity()
                                     : sq_dist(path_points[i - 1].first.location, E);
            double l = sq_dist(path_points[i].first.location, E);
            double l_next = sq_dist(path_points[i + 1].first.location, E);

            // Step 1.1：检测局部极小点
            if (!found_min && l_prev > l && l_next > l) {
                found_min = true;
                min_idx = i;
                // ROS_INFO("[%s] [InitMatch] 局部极小点 idx=%d, dist2=%.4f", 
                //          role_name_.c_str(), i, l);

                // Step 1.2：向后检测连续上升趋势
                increase_in_count = 0;
                for (int j = i; j < static_cast<int>(path_points.size()) - 1; ++j) {
                    double lj = sq_dist(path_points[j].first.location, E);
                    double lj_next = sq_dist(path_points[j + 1].first.location, E);

                    if (lj_next > lj){
                        ++increase_in_count;
                    }
                    else{
                        increase_in_count = 0;
                        min_idx = std::min(j + 1, static_cast<int>(path_points.size()) - 1);
                    }
                    // 连续上升超过3次 → 确认该极小点为投影点
                    if (increase_in_count > 3) {
                        // ROS_INFO("[%s] [InitMatch] 连续上升次数 > 3，确认匹配点 idx=%d",
                        //          role_name_.c_str(), min_idx);
                        best_idx = min_idx;
                        break;
                    }
                }
            }
            // 提前退出优化：避免长路径全扫描
            if (best_idx != -1 || found_min == true) break;
        }

        // Step 1.3：如果没有检测到极小点，则使用全局最小点
        if (best_idx == -1) {
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < (int)path_points.size(); ++i) {
                double dist2 = sq_dist(path_points[i].first.location, E);
                if (dist2 < min_dist) {
                    min_dist = dist2;
                    best_idx = i;
                }
            }
            ROS_WARN("[%s] [InitMatch] 未检测到局部极小点，使用全局最小点 idx=%d",
                     role_name_.c_str(), best_idx);
        }

        last_matched_point_idx_ = best_idx;
        has_first_matched_point_ = true;
    }

    // ======================================================
    //  🧭 Step 2️⃣：后续帧匹配 —— 使用“方向感知 + 局部窗口搜索”
    // ======================================================
    else {
        int idx = last_matched_point_idx_;
        carla::geom::Vector3D path_dir(
            std::cos(path_points[idx].first.rotation.yaw * M_PI / 180.0),
            std::sin(path_points[idx].first.rotation.yaw * M_PI / 180.0),
            0.0);
        carla::geom::Vector3D ego_vec(
            E.x - path_points[idx].first.location.x,
            E.y - path_points[idx].first.location.y,
            0.0);

        double dot = path_dir.x * ego_vec.x + path_dir.y * ego_vec.y;
        bool search_forward = (dot > 0);
        if (search_forward) {
            bool found_min = false;
            int increase_in_count = 0;
            int min_idx = -1;

            for (int i = std::max(0, idx - 2); i < static_cast<int>(path_points.size()) - 1; ++i) {
                double l_prev = (i == 0) ? std::numeric_limits<double>::infinity()
                                        : sq_dist(path_points[i - 1].first.location, E);
                double l = sq_dist(path_points[i].first.location, E);
                double l_next = sq_dist(path_points[i + 1].first.location, E);

                // Step 1.1：检测局部极小点
                if (!found_min && l_prev > l && l_next > l) {
                    found_min = true;
                    min_idx = i;
                    // ROS_ERROR("[%s] [InitMatch] 局部极小点 idx=%d, dist2=%.4f", 
                    //         role_name_.c_str(), i, l);

                    // Step 1.2：向后检测连续上升趋势
                    increase_in_count = 0;
                    for (int j = i; j < static_cast<int>(path_points.size()) - 1; ++j) {
                        double lj = sq_dist(path_points[j].first.location, E);
                        double lj_next = sq_dist(path_points[j + 1].first.location, E);

                        if (lj_next > lj){
                            ++increase_in_count;
                        }
                        else{
                            increase_in_count = 0;
                            min_idx = std::min(j + 1, static_cast<int>(path_points.size()) - 1);
                        }
                        // 连续上升超过3次 → 确认该极小点为投影点
                        if (increase_in_count > 3) {
                            // ROS_INFO("[%s] [InitMatch] 连续上升次数 > 3，确认匹配点 idx=%d",
                            //         role_name_.c_str(), min_idx);
                            best_idx = min_idx;
                            break;
                        }
                    }
                }
                // 提前退出优化：避免长路径全扫描
                if (best_idx != -1 || found_min == true) break;
            }
            // if (best_idx == -1)
            // ROS_ERROR("[%s] [InitMatch] 未检测到局部极小点，使用全局最小点 idx=%d",
            //     role_name_.c_str(), best_idx);
        }else{
            bool found_min = false;
            int increase_in_count = 0;
            int min_idx = -1;

            for (int i = std::min((int)path_points.size() - 2, idx + 2); i >= 0; --i) {
                double l_prev = (i == 0) ? std::numeric_limits<double>::infinity()
                                        : sq_dist(path_points[i - 1].first.location, E);
                double l = sq_dist(path_points[i].first.location, E);
                double l_next = sq_dist(path_points[i + 1].first.location, E);

                // Step 1.1：检测局部极小点
                if (!found_min && l_prev > l && l_next > l) {
                    found_min = true;
                    min_idx = i;
                    // ROS_WARN("[%s] [InitMatch] 局部极小点 idx=%d, dist2=%.4f", 
                    //         role_name_.c_str(), i, l);

                    // Step 1.2：向前检测连续上升趋势
                    increase_in_count = 0;
                    for (int j = i; j >= 0; --j) {
                        double lj = sq_dist(path_points[j].first.location, E);
                        double lj_prev = (j == 0) ? std::numeric_limits<double>::infinity()
                                         : sq_dist(path_points[j - 1].first.location, E);
                        if (lj_prev > lj){
                            ++increase_in_count;
                        }
                        else{
                            increase_in_count = 0;
                            min_idx = std::max(j - 1, 0);
                        }
                        // 连续上升超过3次 → 确认该极小点为投影点
                        if (increase_in_count > 3 || j == 0) {
                            // ROS_INFO("[%s] [InitMatch] 连续上升次数 > 3，确认匹配点 idx=%d",
                            //         role_name_.c_str(), min_idx);
                            best_idx = min_idx;
                            break;
                        }
                    }
                }
                // 提前退出优化：避免长路径全扫描
                if (best_idx != -1 || found_min == true) break;
            }  
            // if (best_idx == -1)
            // ROS_WARN("[%s] [InitMatch] 未检测到局部极小点，使用全局最小点 idx=%d",
            //          role_name_.c_str(), best_idx);
        }
        if (best_idx == -1) {
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < (int)path_points.size(); ++i) {
                double dist2 = sq_dist(path_points[i].first.location, E);
                if (dist2 < min_dist) {
                    min_dist = dist2;
                    best_idx = i;
                }
            }
           ROS_WARN("[%s] [InitMatch] 未检测到局部极小点，使用全局最小点 idx=%d",
            role_name_.c_str(), best_idx);
        }
        last_matched_point_idx_ = best_idx;
        has_first_matched_point_ = true;
    }

    // ======================================================
    //  🧩 Step 3️⃣：构建 ego path
    // ======================================================
    int end_idx = std::min((int)path_points.size(), best_idx + num_points);
    for (int i = best_idx; i < end_idx; ++i)
        path.push_back(path_points[i].first);

    // ======================================================
    //  💾 Step 4️⃣：导出到文件（调试用）
    // ======================================================
    std::vector<carla::geom::Transform> path_transforms;
    path_transforms.reserve(path_points.size());
    for (const auto& p : path_points) {
        path_transforms.push_back(p.first);  // 提取 transform
    }
    if(Config_.role_name_ == "hero100" && 1){
        common::FileExporter::ExportEgoPath(
            path_transforms,
            path,
            "/home/bob/文档/备份/demo05/src/test/txt/ego_path/ego_path.txt",
            Config_.frame_id,
            false,   // append_mode
            true,    // split_mode
            300,     // 每300帧一个文件
            false    // 非CSV模式
        );
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
bool AutoDriver::is_box_intersect_path(
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

std::vector<carla::geom::Transform> AutoDriver::get_target_path(
    const carla::SharedPtr<carla::client::Waypoint>& tar_wp,
    const carla::geom::Transform& tar_transform,
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
    future_path.push_back(tar_transform);

    if ((direction == "left" || direction == "right") && !current_wp->IsJunction()) {

            auto lane_change = current_wp->GetLaneChange();  // 获取当前车道允许的变道方向

            if (direction == "left" && lane_change == carla::road::element::LaneMarking::LaneChange::Left||
                lane_change == carla::road::element::LaneMarking::LaneChange::Both) {

                auto left_wp = current_wp->GetLeft();
                if (left_wp && left_wp->GetType() == carla::road::Lane::LaneType::Driving) {
                    current_wp = left_wp;
                    future_path.push_back(current_wp->GetTransform());
                    ROS_INFO("[get_target_path] ID=%d 在非路口处左变道到 LaneID=%d", 
                            target_id, current_wp->GetLaneId());
                }

            } else if (direction == "right" && lane_change == carla::road::element::LaneMarking::LaneChange::Right ||
                    lane_change == carla::road::element::LaneMarking::LaneChange::Both) {

                auto right_wp = current_wp->GetRight();
                if (right_wp && right_wp->GetType() == carla::road::Lane::LaneType::Driving) {
                    current_wp = right_wp;
                    future_path.push_back(current_wp->GetTransform());
                    ROS_INFO("[get_target_path] ID=%d 在非路口处右变道到 LaneID=%d", 
                            target_id, current_wp->GetLaneId());
                }
            }
    }
        
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

carla::SharedPtr<carla::client::Waypoint> AutoDriver::find_match_waypoint(
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

void AutoDriver::MakeDecision() {
    // TODO: 实现决策逻辑
    // 1. 根据当前状态做出决策
    // 2. 更新控制命令
    ROS_DEBUG("MakeDecision: 决策功能待实现");
}

double AutoDriver::calcu_distance_to_entry_line(const carla::SharedPtr<carla::client::Waypoint>& wp,
                                           const carla::geom::Location& ego_loc) const {
    if (!wp) return -1.0;

    int road_id = wp->GetRoadId();
    int lane_id = wp->GetLaneId();

    auto it = road_lane_to_entry_stop_line_.find({road_id, lane_id});
    if (it == road_lane_to_entry_stop_line_.end()) return -1.0;

    const auto& stop_line = it->second.location;
    return carla::geom::Math::Distance(ego_loc, stop_line);
}

 
bool AutoDriver::yield_in_decision_area() {

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

    if (phase_state == "Red") {
    }

    if (phase_state == "Yellow") {
    }

    // 绿灯时判断模糊组合
    if (phase_state == "Green") {
    }

    // 其它未知灯 → 保守等待
    ROS_DEBUG("%s: 未知灯相位，默认等待", role_name_.c_str());
    return true;
}


bool AutoDriver::is_crossed_exit_line(
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

bool AutoDriver::is_crossed_entry_line(
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

void AutoDriver::interpolate_path(
    const std::vector<carla::geom::Transform>& path,
    double spacing,
    std::vector<carla::geom::Transform>& interp_path)
{
    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;

    interp_path.clear();
    if (path.size() < 2 || spacing <= 0.0) {
        interp_path = path;
        return;
    }

    const size_t N = path.size();
    std::vector<double> s_list(N, 0.0);
    std::vector<double> yaw_list(N), pitch_list(N), roll_list(N);

    // === 1. 累计弧长 ===
    for (size_t i = 1; i < N; ++i) {
        const auto& p0 = path[i - 1].location;
        const auto& p1 = path[i].location;
        s_list[i] = s_list[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    // === 2. 角度转弧度 ===
    for (size_t i = 0; i < N; ++i) {
        yaw_list[i]   = path[i].rotation.yaw   * DEG2RAD;
        pitch_list[i] = path[i].rotation.pitch * DEG2RAD;
        roll_list[i]  = path[i].rotation.roll  * DEG2RAD;
    }

    const double s_max = s_list.back();

    // === 3. 定义角度 slerp（短弧插值）===
    auto slerp_angle = [](double a0, double s0, double a1, double s1, double s) {
        double da = std::fmod(a1 - a0, 2.0 * M_PI);
        if (da > M_PI) da -= 2.0 * M_PI;
        else if (da < -M_PI) da += 2.0 * M_PI;
        double r = (s - s0) / (s1 - s0);
        return a0 + da * r;
    };

    // === 4. 按弧长插值 ===
    size_t i = 0;
    for (double s = 0.0; s <= s_max + 1e-6; s += spacing) {
        // 找到当前 s 所在区间
        while (i + 1 < N && s_list[i + 1] < s) ++i;
        if (i + 1 >= N) break;

        const double s0 = s_list[i];
        const double s1 = s_list[i + 1];
        const double ratio = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;

        const auto& tf0 = path[i];
        const auto& tf1 = path[i + 1];

        // --- 插值位置 ---
        carla::geom::Location loc;
        loc.x = tf0.location.x + ratio * (tf1.location.x - tf0.location.x);
        loc.y = tf0.location.y + ratio * (tf1.location.y - tf0.location.y);
        loc.z = tf0.location.z + ratio * (tf1.location.z - tf0.location.z);

        // --- 插值姿态 ---
        double yaw   = slerp_angle(yaw_list[i], s0, yaw_list[i + 1], s1, s);
        double pitch = slerp_angle(pitch_list[i], s0, pitch_list[i + 1], s1, s);
        double roll  = slerp_angle(roll_list[i], s0, roll_list[i + 1], s1, s);

        carla::geom::Rotation rot;
        rot.yaw   = yaw   * RAD2DEG;
        rot.pitch = pitch * RAD2DEG;
        rot.roll  = roll  * RAD2DEG;

        interp_path.emplace_back(loc, rot);
    }

    // === 5. 确保终点插入 ===
    if (interp_path.empty() || 
        std::hypot(interp_path.back().location.x - path.back().location.x,
                   interp_path.back().location.y - path.back().location.y) > 1e-3) {
        interp_path.push_back(path.back());
    }
}


std::optional<std::pair<double, size_t>> AutoDriver::project_point_to_path(
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

void AutoDriver::get_forward_path(
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

void AutoDriver::reset() {
    // 重置状态变量
    driver_state_ = DriverState::INIT;  // 重置为初始化状态
    is_destroyed_ = false;              // 标记为非销毁状态

    // 清空路径相关信息
    ego_path.clear();                   // 清空自车路径
    interp_path.clear();                // 清空插值路径

    // 清空感知相关的信息
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

    // 清空交通灯信息
    traffic_light_map_.clear();         // 清空交通灯相位信息
    affecting_traffic_light = nullptr;  // 清空交通灯对象

    // 清空周围物体信息
    carla_objects_.clear();             // 清空物体信息
    target_stationary_counter_.clear(); // 清空目标静止计数器
}

void AutoDriver::destroy() {
    std::lock_guard<std::mutex> objects_lock(data_mutex);
    if (is_destroyed_) {
        ROS_WARN("%s(%d): destroy() called, but already destroyed.", role_name_.c_str(), vehicle_config_.carla_id);
        return;
    }

    is_destroyed_ = true;

    try {
        ROS_INFO("%s(%d): Destroying AutoDriver...", role_name_.c_str(), vehicle_config_.carla_id);

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
        ego_corners.clear();
        ego_polygon.outer().clear();
        ego_path_poly.outer().clear();

        // 4. 清空动态对象、规划器、感知数据
        carla_objects_.clear();               // 清空目标障碍物集合
        last_valid_tar_wpts_.clear();
        last_valid_ego_wpts_.clear();
        traffic_light_map_.clear();
        road_lane_to_entry_stop_line_.clear();
        road_lane_to_exit_reference_.clear();
        road_to_traffic_light_.clear();


        ROS_INFO("%s(%d): AutoDriver successfully destroyed.", role_name_.c_str(), vehicle_config_.carla_id);

    } catch (const std::exception& e) {
        ROS_ERROR("%s(%d): Exception during destroy(): %s", role_name_.c_str(), vehicle_config_.carla_id, e.what());
    } catch (...) {
        ROS_ERROR("%s(%d): Unknown exception occurred during destroy().", role_name_.c_str(), vehicle_config_.carla_id);
    }
}

void AutoDriver::creat_obstacle(
    std::vector<planner::Obstacle>& AllObstacle,
    const derived_object_msgs::Object& obj,
    const geometry_msgs::PoseArray& tar_path)
{
    planner::Obstacle obs(Config_);

    obs.timestamp_ = obj.header.stamp;
    obs.centerpoint = obj.pose;
    // 尺寸
    obs.obstacle_length = obj.shape.dimensions[0];
    obs.obstacle_width  = obj.shape.dimensions[1];
    obs.obstacle_height = obj.shape.dimensions[2];

    // 基本属性
    obs.obstacle_id    = std::to_string(obj.id);
    obs.obstacle_type  = 1;// 1-车辆，2-行人，3-自行车，4-摩托车，5-静止物体，6-未知
    obs.is_virtual = false;

    obs.obstacle_shape = obj.shape.BOX;

    // 姿态
    tf::Quaternion q;
    tf::quaternionMsgToTF(obj.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    obs.obstacle_threa = yaw;

    // 速度 & 加速度
    obs.obstacle_velocity = std::sqrt(
        obj.twist.linear.x * obj.twist.linear.x + 
        obj.twist.linear.y * obj.twist.linear.y +
        obj.twist.linear.z * obj.twist.linear.z);
    obs.obstacle_acc = 0.0; //障碍物轨迹预测不考虑加速度

    // ===== 动态障碍物预测 =====
    if (obs.obstacle_velocity > 1.0) {
        obs.is_static = false;
        oba.Generater_Trajectory_FromPath(obs.trajectory_prediction, 
                                 obs.centerpoint,
                                 Config_.dynamic_obs_predict_time, 
                                 tar_path,                  
                                 obs.obstacle_velocity, 
                                 obs.obstacle_acc);
    }else{
        obs.start_time = 0.0;
        obs.end_time = 5.0;
    }

    // ===== 计算几何边界点 =====
    PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
    PPoint lf, lb, rf, rb;
    oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, 
                              center, obs.obstacle_threa, lf, lb, rb, rf);

    std::vector<PPoint> corners = {lf, rf, rb, lb};
    
    if (obs.obstacle_shape == obj.shape.BOX) 
    {
        obs.pinnacle = geometry_msgs::PoseArray();
        obs.pinnacle.header.stamp = ros::Time::now();
        obs.pinnacle.header.frame_id = "map";
        obs.pinnacle.poses.clear();
        for (const auto& c : corners) {
            geometry_msgs::Pose sds;
            sds.position.x = c.x;
            sds.position.y = c.y;
            sds.position.z = 0.0;
            obs.pinnacle.poses.push_back(sds);
            obs.polygon_points.push_back(Vec2d(c.x, c.y));
        }
        obs.obstacle_radius = 0;
    }
    // ✅ 存入 AllObstacle
    AllObstacle.push_back(obs);
}

void AutoDriver::create_schedule_obstacles(
    std::vector<planner::Obstacle>& AllObstacle,
    std::vector<ScheduleCommand>& cmds,  // 注意这里不再是 const
    const ReferenceLine &reference_line) 
{
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    ros::Time now = ros::Time::now();
    auto build_one = [&](const ScheduleCommand& cmd, ReferencePoint matched_min, ReferencePoint matched_max, double time_start, double time_end) {
        if (cmd.ConflictpPoint_corners.empty()) {
            ROS_WARN("[Scheduler] 区域无角点信息，跳过障碍构建");
            return;
        }

        // ===== 2. 构建两个虚拟障碍物 =====
        auto create_obs = [&](const ReferencePoint& matched_point,
                            const std::string& suffix,
                            double start_t, double end_t,
                            uint8_t color_r, uint8_t color_g, uint8_t color_b) {
            planner::Obstacle obs(Config_);
            obs.timestamp_ = now;
            obs.centerpoint.position.x = matched_point.x_;
            obs.centerpoint.position.y = matched_point.y_;
            obs.centerpoint.position.z = 0.0;
            obs.centerpoint.orientation = tf::createQuaternionMsgFromYaw(matched_point.heading_);

            obs.obstacle_length = 1.0;
            obs.obstacle_width  = 4.0;
            obs.obstacle_height = 1.0;

            obs.obstacle_id = "schedule_" + std::to_string(cmd.id) + suffix;
            obs.is_virtual = true;
            obs.obstacle_type = (suffix == "_before") ? 9 : 10;
            obs.obstacle_shape = derived_object_msgs::Object::_shape_type::BOX;
            obs.obstacle_threa = matched_point.heading_;
            obs.obstacle_velocity = 0.0;
            obs.obstacle_acc = 0.0;

            obs.start_time = start_t;
            obs.end_time   = end_t;

            // 计算包围盒四角
            PPoint center(matched_point.x_, matched_point.y_);
            PPoint lf, lb, rf, rb;
            oba.CalculateCarBoundaryPoint(
                obs.obstacle_length, obs.obstacle_width,
                center, obs.obstacle_threa, lf, lb, rf, rb);

            std::vector<PPoint> corners = {lf, rf, rb, lb};
            obs.pinnacle.header.stamp = now;
            obs.pinnacle.header.frame_id = "map";
            obs.pinnacle.poses.clear();
            for (const auto& c : corners) {
                geometry_msgs::Pose sds;
                sds.position.x = c.x;
                sds.position.y = c.y;
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(c.x, c.y));
            }

            obs.obstacle_radius = 0.5;
            AllObstacle.push_back(obs);

            // ===== 可视化（仅首次构建时） =====
            carla::geom::Transform transform = ros_pose_to_carla_transform(obs.centerpoint);
            carla::geom::Vector3D extent(obs.obstacle_length * 0.5,
                                        obs.obstacle_width * 0.5,
                                        obs.obstacle_height * 0.5);
            debug_helper_->DrawBox(
                carla::geom::BoundingBox(transform.location, extent),
                transform.rotation,
                0.1f,
                {color_r, color_g, color_b},
                0.2,
                true);

            debug_helper_->DrawString(
                transform.location + carla::geom::Location(0, 0, 2.0f),
                obs.obstacle_id,
                false, {255u, 255u, 0u}, 0.1f, true);
        };

        // ===== 3. 构建 “进入前” 和 “离开后” 障碍物 =====
        if(time_start > 0.0){
            create_obs(matched_min, "_before", 0.0, time_start, 255, 0, 0);         // 红色
        }
        if(time_end > 0.0){
            create_obs(matched_max, "_after", time_end, time_end + 10.0, 255, 0, 0); // 绿色
        }
    };

    // 新的 cmds 容器，用于保存未完成的调度
    std::vector<ScheduleCommand> remaining_cmds;

    for (const auto& cmd : cmds) {
        // ===== 1. 匹配所有角点到参考线，找出 s_min 和 s_max =====
        double s_min = std::numeric_limits<double>::max();
        double s_max = std::numeric_limits<double>::lowest();
        ReferencePoint matched_min, matched_max;

        for (const auto& corner_pose : cmd.ConflictpPoint_corners)
        {
            ReferencePoint matched_point = PathMatcher::MatchToPath(
                reference_line.path_reference(),
                corner_pose.position.x,
                corner_pose.position.y);

            if (matched_point.s() < s_min) {
                s_min = matched_point.s();
                matched_min = matched_point;
            }
            if (matched_point.s() > s_max) {
                s_max = matched_point.s();
                matched_max = matched_point;
            }
        }

        // 如果已经通过目标点，跳过该调度
        if (matched_max.s() <= 1.0) {
            ROS_ERROR("ScheduleCommand id=%d finished, removing.", cmd.id);
            continue;
        }

        // 时间逻辑
        double t_start_rel = (cmd.time_start - now).toSec();
        double t_end_rel   = (cmd.time_end   - now).toSec();

        if (t_start_rel >= t_end_rel) {
            ROS_WARN("ScheduleCommand id=%d invalid time window: start=%.2f, end=%.2f",
                     cmd.id, t_start_rel, t_end_rel);
            continue;
        }
    
        build_one(cmd, matched_min, matched_max, t_start_rel, t_end_rel);

        // 保留该调度
        remaining_cmds.push_back(cmd);
    }
    // 更新 cmds（去掉已完成的指令）
    cmds.swap(remaining_cmds);
}

void AutoDriver::create_traffic_light_obstacle(std::vector<planner::Obstacle>& AllObstacle) 
{
    ros::Time now = ros::Time::now();
    int road_id = ego_wpt->GetRoadId();
    int lane_id = ego_wpt->GetLaneId();
    auto key = std::make_pair(road_id, lane_id);

    // 找 entry stop line
    auto entry_line = road_lane_to_entry_stop_line_.find(key);
    if (entry_line == road_lane_to_entry_stop_line_.end()) {
        ROS_DEBUG("[%s] 未找到 road_id=%d lane_id=%d 的 entry_stop_line", 
                  role_name_.c_str(), road_id, lane_id);
        return;
    }

    const auto& entry_line_loc = entry_line->second.location;
    int light_id = -1;  // 默认无效值

    // road_id → traffic light 映射
    auto road_it = road_to_traffic_light_.find(road_id);
    if (road_it != road_to_traffic_light_.end()) {
        light_id = road_it->second;
    } else { 
        ROS_WARN("[%s] 未找到 road_id=%d 的交通灯信息，无法构造交通灯障碍物", 
                 role_name_.c_str(), road_id);
        return;
    }

    // traffic_light_map 查询
    auto it = traffic_light_map_.find(light_id);
    if (traffic_light_map_.empty() || it == traffic_light_map_.end()) {
        ROS_WARN("[%s] traffic_light_map 中没有找到 light_id=%d", 
                 role_name_.c_str(), light_id);
        return;
    }

    const auto& tl_info = it->second;
    planner::Obstacle obs(Config_);
    obs.is_virtual = true;

    if (tl_info.state == "Red") {
        // ============ 构造红灯障碍物 ============
        obs.timestamp_ = now;
        // 位姿
        geometry_msgs::Pose pose;
        pose.position.x = entry_line_loc.x;
        pose.position.y = -entry_line_loc.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(entry_line->second.rotation.yaw * M_PI / 180.0);
        obs.centerpoint = pose;

        // 基础属性
        obs.obstacle_length = 1.0;
        obs.obstacle_width  = 4.0;
        obs.obstacle_height = 1.0;

        obs.obstacle_id    = "traffic_light_" + std::to_string(light_id);
        obs.obstacle_type  = 8; // 交通灯
        obs.obstacle_shape = derived_object_msgs::Object::_shape_type::BOX;

        obs.obstacle_threa     = entry_line->second.rotation.yaw * M_PI / 180.0;
        obs.obstacle_velocity  = 0.0;
        obs.obstacle_acc       = 0.0;

        obs.start_time = 0.0;
        obs.end_time   = tl_info.remaining_time;  // 红灯剩余时间
        if(tl_info.remaining_time < 0.001 && role_name_ == "hero0"){
            ROS_ERROR("%s: 红灯【%d】时间过短，默认等待", role_name_.c_str(), light_id);
            for (const auto& kv : traffic_light_map_) {
                const auto& phase = kv.second;
                ROS_INFO("[TrafficLightMap] road_id=%d id=%d lane_id=%d state=%s "
                        "remain=%.2f red=%.2f yellow=%.2f green=%.2f loc=(%.2f, %.2f, %.2f)",
                        phase.road_id,
                        phase.id,
                        phase.lane_id,
                        phase.state.c_str(),
                        phase.remaining_time,
                        phase.red_duration,
                        phase.yellow_duration,
                        phase.green_duration,
                        phase.location.x,
                        phase.location.y,
                        phase.location.z);
            }
        }

        // 计算四个角点
        PPoint center(pose.position.x, pose.position.y);
        PPoint lf, lb, rf, rb;
        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width,
                                    center, obs.obstacle_threa, lf, lb, rf, rb);

        std::vector<PPoint> corners = {lf, rf, rb, lb};

        // 清空并赋值
        obs.pinnacle = geometry_msgs::PoseArray();
        obs.pinnacle.header.stamp = now;
        obs.pinnacle.header.frame_id = "map";
        obs.pinnacle.poses.clear();
        obs.polygon_points.clear();

        for (const auto& c : corners) {
            geometry_msgs::Pose sds;
            sds.position.x = c.x;
            sds.position.y = c.y;
            sds.position.z = 0.0;
            obs.pinnacle.poses.push_back(sds);
            obs.polygon_points.push_back(Vec2d(c.x, c.y));
        }

        obs.obstacle_radius = 0.5;
    }else if (tl_info.state == "Green")
    {
        obs.timestamp_ = now;
        // 位姿
        geometry_msgs::Pose pose;
        pose.position.x = entry_line_loc.x;
        pose.position.y = -entry_line_loc.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(entry_line->second.rotation.yaw * M_PI / 180.0);
        obs.centerpoint = pose;

        // 基础属性
        obs.obstacle_length = 1.0;
        obs.obstacle_width  = 4.0;
        obs.obstacle_height = 1.0;

        obs.obstacle_id    = "traffic_light_" + std::to_string(light_id);
        obs.obstacle_type  = 8; // 交通灯
        obs.obstacle_shape = derived_object_msgs::Object::_shape_type::BOX;

        obs.obstacle_threa     = entry_line->second.rotation.yaw * M_PI / 180.0;
        obs.obstacle_velocity  = 0.0;
        obs.obstacle_acc       = 0.0;

        obs.start_time = tl_info.remaining_time;
        obs.end_time   = tl_info.remaining_time + 30;  // 红灯剩余时间

        // 计算四个角点
        PPoint center(pose.position.x, pose.position.y);
        PPoint lf, lb, rf, rb;
        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width,
                                    center, obs.obstacle_threa, lf, lb, rf, rb);

        std::vector<PPoint> corners = {lf, rf, rb, lb};

        // 清空并赋值
        obs.pinnacle = geometry_msgs::PoseArray();
        obs.pinnacle.header.stamp = now;
        obs.pinnacle.header.frame_id = "map";
        obs.pinnacle.poses.clear();
        obs.polygon_points.clear();

        for (const auto& c : corners) {
            geometry_msgs::Pose sds;
            sds.position.x = c.x;
            sds.position.y = c.y;
            sds.position.z = 0.0;
            obs.pinnacle.poses.push_back(sds);
            obs.polygon_points.push_back(Vec2d(c.x, c.y));
        }

        obs.obstacle_radius = 0.5;
    }else if (tl_info.state == "Yellow"){
        obs.timestamp_ = now;
        // 位姿
        geometry_msgs::Pose pose;
        pose.position.x = entry_line_loc.x;
        pose.position.y = -entry_line_loc.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(entry_line->second.rotation.yaw * M_PI / 180.0);
        obs.centerpoint = pose;

        // 基础属性
        obs.obstacle_length = 1.0;
        obs.obstacle_width  = 4.0;
        obs.obstacle_height = 1.0;

        obs.obstacle_id    = "traffic_light_" + std::to_string(light_id);
        obs.obstacle_type  = 8; // 交通灯
        obs.obstacle_shape = derived_object_msgs::Object::_shape_type::BOX;

        obs.obstacle_threa     = entry_line->second.rotation.yaw * M_PI / 180.0;
        obs.obstacle_velocity  = 0.0;
        obs.obstacle_acc       = 0.0;

        obs.start_time = 0.0;
        obs.end_time   = tl_info.remaining_time + 30;

        // 计算四个角点
        PPoint center(pose.position.x, pose.position.y);
        PPoint lf, lb, rf, rb;
        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width,
                                    center, obs.obstacle_threa, lf, lb, rf, rb);

        std::vector<PPoint> corners = {lf, rf, rb, lb};

        // 清空并赋值
        obs.pinnacle = geometry_msgs::PoseArray();
        obs.pinnacle.header.stamp = now;
        obs.pinnacle.header.frame_id = "map";
        obs.pinnacle.poses.clear();
        obs.polygon_points.clear();

        for (const auto& c : corners) {
            geometry_msgs::Pose sds;
            sds.position.x = c.x;
            sds.position.y = c.y;
            sds.position.z = 0.0;
            obs.pinnacle.poses.push_back(sds);
            obs.polygon_points.push_back(Vec2d(c.x, c.y));
        }

        obs.obstacle_radius = 0.5;
    }
    
    // 加入障碍物集合
    // if(role_name_ != "hero000"){
    //     AllObstacle.push_back(obs);
    // }
        
    AllObstacle.push_back(obs);

    // 可视化绘制
    carla::geom::Transform transform = ros_pose_to_carla_transform(obs.centerpoint);
    carla::geom::Vector3D extent(obs.obstacle_length * 0.5,
                                obs.obstacle_width * 0.5,
                                obs.obstacle_height * 0.5);

    // 根据灯色选择颜色
    carla::client::DebugHelper::Color box_color;
    carla::client::DebugHelper::Color text_color;
    if (tl_info.state == "Red") {
        box_color  = {255u, 0u, 0u, 100u};     // 红色
        text_color = {255u, 0u, 0u};           // 红色文字
    } else if (tl_info.state == "Green") {
        box_color  = {0u, 255u, 0u, 100u};     // 绿色
        text_color = {0u, 255u, 0u};           // 绿色文字
    } else { // Yellow
        box_color  = {255u, 255u, 0u, 100u};   // 黄色
        text_color = {255u, 255u, 0u};         // 黄色文字
    }

    // 绘制 Box
    debug_helper_->DrawBox(
        carla::geom::BoundingBox(transform.location, extent),
        transform.rotation,
        0.1f,
        box_color,
        0.2f,
        true
    );

    // 绘制剩余时间字符串
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << tl_info.remaining_time << "s";
    carla::geom::Location text_location = transform.location;
    text_location.z += obs.obstacle_height + 1.0f; // 抬高文字，避免重叠
    debug_helper_->DrawString(
        text_location,
        ss.str(),
        false,   // 不要阴影
        text_color,
        0.2f
    );
}

void driver_model::AutoDriver::generate_test_schedule_commands(std::vector<ScheduleCommand>& cmds) {
    static bool initialized = false;
    static std::vector<ScheduleCommand> cached_cmds;

    if (!initialized) {
        ros::Time now = ros::Time::now();

        auto make_pose = [](double x, double y) {
            geometry_msgs::Pose p;
            p.position.x = x;
            p.position.y = y;
            p.position.z = 0.0;
            p.orientation.w = 1.0;
            return p;
        };

        ScheduleCommand cmd2;
        cmd2.id = 2;
        cmd2.pose = make_pose(-10.0, -2.0);
        cmd2.time_start = now + ros::Duration(130.0);
        cmd2.time_end = now + ros::Duration(18.0);
        cached_cmds.push_back(cmd2);

        initialized = true;
    }

    cmds = cached_cmds;
}

// void AutoDriver::generate_test_schedule_commands(std::vector<ScheduleCommand>& cmds) {
//     // 用静态变量，保证只生成一次
//     static bool initialized = false;
//     static std::vector<ScheduleCommand> cached_cmds;

//     if (!initialized) {
//         ros::Time now = ros::Time::now();

//         cached_cmds.push_back({1, -30.0, -2.0, now + ros::Duration(5.0),  now + ros::Duration(10.0)});
//         cached_cmds.push_back({2, -10.0, -2.0, now + ros::Duration(13.0), now + ros::Duration(18.0)});
//         cached_cmds.push_back({3,  20.0, -2.0, now + ros::Duration(17.0), now + ros::Duration(25.0)});

//         initialized = true;
//     }

//     // 拷贝到传入的 cmds
//     cmds = cached_cmds;
// }

bool AutoDriver::dynamic_routing(){
    double header_time = ros::Time::now().toSec();
    best_path.Clear();
    // =============== 1. 轨迹拼接，获取规划起点 =================
    // 更新车辆状态
    UpdateVehicleStateFromOdom();
    VehicleState ego_vehicle_state = vehicle_state_provider_.vehicle_state();

    TrajectoryPoint planning_init_point;
    std::string replan_reason;

    auto stitching_traj = planner::TrajectoryStitcher::ComputeStitchingTrajectory(
        ego_vehicle_state,                          // 当前车辆状态
        header_time,                               // 当前时间
        0.1,                                         // 规划周期
        &pb_planned_trajectory_,                        // 上一条轨迹
        &replan_reason,
        &Config_);

    if (!stitching_traj.empty()) {
        planning_init_point = stitching_traj.back();   // ✅ 拼接轨迹的最后个点作为规划起点
    } else {
        ROS_ERROR("[%s]:Trajectory stitching failed. Cannot find planning init point!", Config_.role_name_.c_str());
    }
    if (!replan_reason.empty()) {
        ROS_WARN("[%s]:Replan reason: %s", Config_.role_name_.c_str(), replan_reason.c_str());
    }

    // =============== 2. 构造参考线 =================
    reference_path.first.clear();
    reference_path.second.clear();
    reference_points.clear();
    accumulated_s.clear();

    geometry_msgs::PoseArray ego_interp_ros_path = carla_path_to_ros_posearray(interp_path);
    
    if (!ego_interp_ros_path.poses.empty())
    {
        std::vector<double> xs, ys;
        for (const auto& pose : ego_interp_ros_path.poses) {
            xs.push_back(pose.position.x);
            ys.push_back(pose.position.y);
        }

        // 1) 用 CubicSpline2D 光滑
        CubicSpline2D spline;
        std::vector<double> s = spline.calc_s(xs, ys);
        csp = new CubicSpline2D(xs, ys, s);

        // 2) 在样条上均匀采样，生成光滑后的 xy_points
        std::vector<std::pair<double,double>> xy_points;
        double step = 0.5;  // 采样间隔，可调
        for (double si = s.front(); si <= s.back(); si += step) {
            double x = csp->calc_x(si);
            double y = csp->calc_y(si);
            xy_points.emplace_back(x, y);
        }

        // 3) 调用 PathMatcher::ComputePathProfile 计算 heading/kappa
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;

        if (!PathMatcher::ComputePathProfile(xy_points, &headings,
                                            &accumulated_s, &kappas, &dkappas))
        {
            ROS_WARN("❌ ReferenceLine generate failed in ComputePathProfile!");
        }
        else
        {
            for (size_t i = 0; i < xy_points.size(); i++) { 
                reference_path.first.push_back(xy_points[i].first);
                reference_path.second.push_back(xy_points[i].second);
                ReferencePoint reference_point(
                    kappas[i], dkappas[i],
                    xy_points[i].first, xy_points[i].second,
                    headings[i], accumulated_s[i]);
                reference_points.emplace_back(reference_point);
            }
            // ROS_INFO("✅ ReferenceLine generated successfully with smoothing! size=%zu", reference_points.size());
        }
    } else {
        ROS_WARN("⚠️ ReferenceLine generation failed: empty input path!");
    }
    double lon_decision_horizon = 80; 
    ReferenceLine reference_line(csp, reference_points, accumulated_s, reference_path, Config_);
    ReferenceLineInfo reference_line_info(ego_vehicle_state, reference_line);

    // =============== 3. 构造调度障碍物 =================
    // generate_test_schedule_commands(schedule_cmds_); //测试
    create_schedule_obstacles(AllObstacle, schedule_cmds_, reference_line);
    if(driver_intention_ != DrivingIntention::TURNING_RIGHT && scene_type_ == "tl_intersection"){
        create_traffic_light_obstacle(AllObstacle);
    }

    // =============== 4. 轨迹规划 =================
    // 规划起点在参考线上的投影点
    ReferencePoint matched_point = PathMatcher::MatchToPath(reference_line.path_reference(), planning_init_point.path_point().x,
                                                            planning_init_point.path_point().y);
    // 笛卡尔坐标系转Frenet坐标系
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    EM.ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
    planning_init_point.set_s(init_s[0]);
    if(std::abs(init_d[0]) <= 0.2 && Config_.IsChangeLanePath){
        Config_.IsChangeLanePath = false;
        Config_.safe_distance = 5.0;
        Config_.default_cruise_speed=Config_.last_cruise_speed;
        Config_.planning_upper_speed_limit=Config_.last_planning_upper_speed_limit;
        ROS_WARN("[%s] 🚗 已完成换道！| init_d=%.3f | 恢复安全距离=%.1f",
                role_name_.c_str(), init_d[0], Config_.safe_distance);
    }
    best_path = EM.Plan(planning_init_point, reference_line, reference_line_info, AllObstacle, ego_odmo, 
                        planning_init_point.theta, lon_decision_horizon, init_s, init_d);

    // ==================== 5. 结果处理 ====================
    if (!best_path.empty()) {
        // ExportSimulationData(reference_line, ego_odmo, planning_init_point, pb_planned_trajectory_, best_path, "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/frame_data.txt", header_time);

        std::vector<TrajectoryPoint> combined_traj;
        if (!stitching_traj.empty()) {
            combined_traj.assign(stitching_traj.begin(), stitching_traj.end() - 1);
        }
        combined_traj.insert(combined_traj.end(), best_path.begin(), best_path.end());
                
        DiscretizedTrajectory final_traj= DiscretizedTrajectory(combined_traj);
        // 更新轨迹记录
        pb_planned_trajectory_ = planner::PublishableTrajectory(header_time, final_traj);
        local_planner_-> set_local_trajectory(pb_planned_trajectory_);
        // local_planner_->set_reference_line(reference_path);
        has_last_trajectory_ = true;
        is_running_ = true;
        // ROS_INFO("Dynamic routing successfully!");
        return true;
    }else{
        pb_planned_trajectory_.set_header_time(header_time);
        ROS_ERROR("[%s]:Dynamic routing failed: empty best_path!", role_name_.c_str());
        // ExportSimulationData(reference_line, ego_odmo, planning_init_point, pb_planned_trajectory_, best_path, "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/frame_data.txt", header_time);
        return false;
    }
}

std::shared_ptr<local_planner::LocalPlanner> AutoDriver::get_local_planner() {
    return local_planner_;
}

int AutoDriver::get_vehicle_id() const {
    return id_;
}

void AutoDriver::UpdateVehicleStateFromOdom() {
    const auto& pos = ego_odmo.pose.pose.position;
    const auto& ori = ego_odmo.pose.pose.orientation;
    const auto& twist = ego_odmo.twist.twist;

    // 时间戳
    double ts = ego_odmo.header.stamp.toSec();

    // 线速度大小 = sqrt(vx^2 + vy^2 + vz^2)
    double v = std::sqrt(
        twist.linear.x * twist.linear.x +
        twist.linear.y * twist.linear.y +
        twist.linear.z * twist.linear.z);

    // 线加速度（Odometry 里可能没有，需要你用差分或者传进来，这里设为 0）
    double a = 0.0;

    // 偏航角速度
    double yaw_rate = twist.angular.z;

    // 调用 VehicleStateProvider 更新
    vehicle_state_provider_.Update(
        pos.x, pos.y, pos.z,
        ori.w, ori.x, ori.y, ori.z,
        v, a, yaw_rate, ts);
}

void AutoDriver::schedulerCallback(
    const road_side_system_type::PoseWithTimeWindowArray::ConstPtr& msg)
{
    ROS_INFO("[%s] 收到调度消息，共 %lu 个区域", role_name_.c_str(), msg->poses.size());

    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    schedule_cmds_.clear();

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        const auto& p = msg->poses[i];
        ScheduleCommand cmd;
        cmd.id = i;
        // 1. 保存中心点
        cmd.pose = p.pose;

        // 2. 保存角点
        cmd.ConflictpPoint_corners.clear();
        for (const auto& corner : p.ConflictpPoint_corners)
            cmd.ConflictpPoint_corners.push_back(corner);

        // 3. 保存时间窗
        cmd.time_start = p.time_window[0];
        cmd.time_end   = p.time_window[1];

        // 4. 存储
        schedule_cmds_.push_back(cmd);

        ROS_INFO("[%s] 区域[%lu] 保存完成: 角点=%lu, 时间窗=[%.2f, %.2f]",
                 role_name_.c_str(), i,
                 cmd.ConflictpPoint_corners.size(),
                 cmd.time_start.toSec(), cmd.time_end.toSec());
    }

    ROS_INFO("[%s] 已缓存 %lu 条调度命令", role_name_.c_str(), schedule_cmds_.size());
}

} // namespace driver_model 