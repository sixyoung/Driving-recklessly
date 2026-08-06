#include "road_side_system/traffic_light_utils.h"
#include <chrono>
#include <algorithm>

using namespace carla::client;
using namespace std::chrono_literals;

TrafficLightPublisher::TrafficLightPublisher()
    : client_("localhost", 2000), world_(client_.GetWorld()) {
    pub_ = nh_.advertise<road_side_system::TrafficLightPhaseArray>("/traffic_light/phases", 10);
}

void TrafficLightPublisher::RunLoop() {
    ros::Rate rate(10);
    ROS_ERROR("[TrafficLightPublisher] TrafficLightPublisher initialized successfully!");
    while (ros::ok()) {
        traffic_lights_ = world_.GetActors()->Filter("traffic.traffic_light");
        size_t light_count = traffic_lights_->size();

        // 如果找到了交通灯且还没设置过相位，就设置一次
        if (light_count > 0 && !phases_initialized_) {
            SetTrafficLightPhases(10.0f, 3.0f, 0.1f);  
            phases_initialized_ = true;
            ROS_INFO("[TrafficLightPublisher] Found %zu traffic lights, phases initialized!", light_count);
        }
        road_side_system::TrafficLightPhaseArray msg_array;

        for (const auto& light_actor : *traffic_lights_) {
            auto light = boost::static_pointer_cast<TrafficLight>(light_actor);
            road_side_system::TrafficLightPhase phase_msg = BuildTrafficLightPhaseMsg(light);
            msg_array.phases.push_back(phase_msg);
        }

        pub_.publish(msg_array);
        ros::spinOnce();
        rate.sleep();
    }
}

void TrafficLightPublisher::SetTrafficLightPhases(float green_time, float yellow_time, float red_time) {
    if (!traffic_lights_) {
        ROS_WARN("[TrafficLightPublisher] traffic_lights_ is null!");
        return;
    }
    size_t light_count = traffic_lights_->size();
    ROS_INFO("[TrafficLightPublisher] Found %zu traffic lights. Applying phase times...", light_count);

    for (const auto& actor : *traffic_lights_) {
        auto light = boost::static_pointer_cast<TrafficLight>(actor);
        light->ResetGroup();
        light->SetGreenTime(green_time);
        light->SetYellowTime(yellow_time);
        light->SetRedTime(red_time);
    }
}

road_side_system::TrafficLightPhase TrafficLightPublisher::BuildTrafficLightPhaseMsg(
    const boost::shared_ptr<TrafficLight>& light) {

    auto group = light->GetGroupTrafficLights();
    auto [active_light, elapsed] = GetActiveLight(group);
    if (!active_light) {
        ROS_WARN("[TrafficLightMap] No active light found for group");
    }   
    float remaining = EstimateRemainingTime(light, group, active_light, elapsed);

    road_side_system::TrafficLightPhase msg;
    msg.id = light->GetId();
    msg.state = TrafficLightStateToString(light->GetState());
    msg.remaining_time = remaining;

    auto loc = light->GetTransform().location;
    msg.location.x = loc.x;
    msg.location.y = loc.y;
    msg.location.z = loc.z;

    std::vector<carla::SharedPtr<carla::client::Waypoint>> stop_wps = light->GetStopWaypoints();
    if (!stop_wps.empty()) {
        msg.road_id = stop_wps[0]->GetRoadId();
        msg.lane_id = stop_wps[0]->GetLaneId();
        msg.stop_yaw = stop_wps[0]->GetTransform().rotation.yaw;
        msg.stop_location.x = stop_wps[0]->GetTransform().location.x;
        msg.stop_location.y = stop_wps[0]->GetTransform().location.y;
        msg.stop_location.z = stop_wps[0]->GetTransform().location.z;
    } else {
        msg.stop_yaw = light->GetTransform().rotation.yaw;
        msg.stop_location.x = loc.x;
        msg.stop_location.y = loc.y;
        msg.stop_location.z = loc.z;
    }
    msg.green_duration = light->GetGreenTime();
    msg.yellow_duration = light->GetYellowTime();
    msg.red_duration = (light->GetGreenTime() + light->GetYellowTime())*3;

    return msg;
}


std::pair<boost::shared_ptr<carla::client::TrafficLight>, float>
TrafficLightPublisher::GetActiveLight(
    const std::vector<boost::shared_ptr<TrafficLight>>& group) {

    // 1. 先找 elapsed > 0 的灯
    for (auto& l : group) {
        float elapsed = l->GetElapsedTime();
        if (elapsed > 0.0f) {
            return {l, elapsed};
        }
    }

    // 2. 如果没找到，再找一个非 Red 的灯
    for (auto& l : group) {
        auto state = l->GetState();
        if (state == carla::rpc::TrafficLightState::Green) {
            return {l, l->GetGreenTime()};
        } else if (state == carla::rpc::TrafficLightState::Yellow) {
            return {l, l->GetYellowTime()};
        }
    }

    // 3. group 为空，返回空指针
    return {nullptr, 0.0f};
}


float TrafficLightPublisher::EstimateRemainingTime(
    const boost::shared_ptr<TrafficLight>& light,
    const std::vector<boost::shared_ptr<TrafficLight>>& group,
    const boost::shared_ptr<TrafficLight>& active_light,
    float elapsed) {

    if (!light) return 0.0f;
    if (!active_light) {
        return 30.0f;
    }
    auto state = active_light->GetState();
    float duration = 0.0f;
    float wait_time = 0.0f;
    if (light->GetId() == active_light->GetId()) {
        if (state == carla::rpc::TrafficLightState::Green) {
            duration = active_light->GetGreenTime() - elapsed;
        }else if (state == carla::rpc::TrafficLightState::Yellow) {
            duration = active_light->GetYellowTime() - elapsed;
        }else if (state == carla::rpc::TrafficLightState::Red) {
            duration = active_light->GetRedTime() - elapsed;
        }

        if (state == carla::rpc::TrafficLightState::Red) {
            // std::cout << "[🟥] 当前灯为红灯，开始计算等待时间..." << std::endl;

            // 找到当前灯在 group 中的索引
            int current_index = -1;
            for (size_t i = 0; i < group.size(); ++i) {
                if (group[i]->GetId() == light->GetId()) {
                    current_index = static_cast<int>(i);
                    break;
                }
            }
            if (current_index != -1) {
                int index = (current_index + 1) % group.size();
                while (index != current_index) {
                    auto l = group[index];
                    wait_time += l->GetRedTime() + l->GetGreenTime() + l->GetYellowTime();
                    index = (index + 1) % group.size();
                }
            }

            // 红灯状态时的总等待时间 = 当前灯剩余红灯 + 其他灯轮转时间
            return std::max(0.0f, duration + wait_time);
        } else {
            return std::max(0.0f, duration);
        }
    }else{
        // 如果当前灯不是活动灯，计算从活动灯到当前灯的等待时间
        if (state == carla::rpc::TrafficLightState::Green) {
            duration = active_light->GetGreenTime() + active_light->GetYellowTime() + active_light->GetRedTime() - elapsed;
        }else if (state == carla::rpc::TrafficLightState::Yellow) {
            duration = active_light->GetYellowTime() + active_light->GetRedTime() - elapsed;
        }else if (state == carla::rpc::TrafficLightState::Red) {
            duration = active_light->GetRedTime() - elapsed;
        }

        int active_index = -1;
        int current_index = -1;
        for (size_t i = 0; i < group.size(); ++i) {
            if (group[i]->GetId() == active_light->GetId()) {
                active_index = static_cast<int>(i);
            }
            if (group[i]->GetId() == light->GetId()) {
                current_index = static_cast<int>(i);
            }
        }

        if (active_index == -1 || current_index == -1) {
            return 0.0f;  // 未找到，直接返回
        }

        int index = (active_index + 1) % group.size();
        while (index != current_index) {
            const auto& l = group[index];
            wait_time += l->GetGreenTime() + l->GetYellowTime() + l->GetRedTime();
            index = (index + 1) % group.size();
        }
        return std::max(0.0f, duration + wait_time);
    }
}


std::string TrafficLightPublisher::TrafficLightStateToString(carla::rpc::TrafficLightState state) const {
    switch (state) {
        case carla::rpc::TrafficLightState::Red: return "Red";
        case carla::rpc::TrafficLightState::Yellow: return "Yellow";
        case carla::rpc::TrafficLightState::Green: return "Green";
        default: return "Unknown";
    }
}
