#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <road_side_system/TrafficLightPhase.h>
#include <road_side_system/TrafficLightPhaseArray.h>
#include <carla/client/Client.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/World.h>
#include <carla/client/ActorList.h>
#include <carla_msgs/CarlaWorldInfo.h>

using carla::client::TrafficLight;
class TrafficLightPublisher {
public:
    TrafficLightPublisher();
    void RunLoop();

private:
    void SetTrafficLightPhases(float green_time = 30.0f, float yellow_time = 3.0f, float red_time = 0.0);
    road_side_system::TrafficLightPhase BuildTrafficLightPhaseMsg(
    const boost::shared_ptr<TrafficLight>& light);
    std::pair<boost::shared_ptr<carla::client::TrafficLight>, float> GetActiveLight(const std::vector<boost::shared_ptr<carla::client::TrafficLight>>& group);
    float EstimateRemainingTime(const boost::shared_ptr<carla::client::TrafficLight>& light,
                                const std::vector<boost::shared_ptr<carla::client::TrafficLight>>& group,
                                const boost::shared_ptr<carla::client::TrafficLight>& active_light,
                                float elapsed = -1.0f);
    std::string TrafficLightStateToString(carla::rpc::TrafficLightState state) const;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    carla::client::Client client_;
    carla::client::World world_;
    boost::shared_ptr<carla::client::ActorList> traffic_lights_;
    std::unordered_map<int, ros::Publisher> road_id_publishers_;
    bool phases_initialized_ = false; 
};
