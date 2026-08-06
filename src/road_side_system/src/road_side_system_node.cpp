#include <ros/ros.h>
#include <thread>
#include <atomic>
#include "road_side_system/scheduler_system.h"
#include "road_side_system/traffic_light_utils.h"

using namespace road_side_system;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_side_system_node");
    setlocale(LC_ALL, "C.UTF-8");

    ros::NodeHandle nh;

    ROS_INFO("[road_side_system_node] 🚀 Initializing RoadSideSystem...");

    // =============================================
    // 1️⃣ 等待 CARLA 世界信息 (防止 TrafficLightPublisher 连接失败)
    // =============================================
    ROS_INFO("[road_side_system_node] ⏳ Waiting for /carla/world_info...");
    auto msg = ros::topic::waitForMessage<carla_msgs::CarlaWorldInfo>(
        "/carla/world_info", nh, ros::Duration(20.0));

    if (!msg) {
        ROS_ERROR("[road_side_system_node] ❌ Timeout: No /carla/world_info received within 10 seconds.");
        return 1;
    }

    ROS_INFO("[road_side_system_node] ✅ /carla/world_info received.");

    // =============================================
    // 2️⃣ 创建系统对象
    // =============================================
    auto scheduler = std::make_shared<SchedulerSystem>();
    auto tl_publisher = std::make_shared<TrafficLightPublisher>();

    ROS_INFO("[road_side_system_node] ✅ SchedulerSystem and TrafficLightPublisher initialized.");

    // =============================================
    // 3️⃣ 线程1：运行交通灯发布循环
    // =============================================
    std::thread tl_thread([&]() {
        try {
            ROS_INFO("[road_side_system_node] ▶️ TrafficLightPublisher thread started.");
            tl_publisher->RunLoop();  // 内部有 ros::Rate(10)
        } catch (const std::exception &e) {
            ROS_ERROR("[road_side_system_node] TrafficLightPublisher crashed: %s", e.what());
        }
    });

    // =============================================
    // 4️⃣ 线程2：定时发布车辆信息数组
    // =============================================
    // std::thread scheduler_thread([&]() {
    //     ros::Rate loop_rate(10);  // 每秒发布 5 次
    //     ROS_INFO("[road_side_system_node] ▶️ SchedulerSystem thread started.");
    //     while (ros::ok()) {
    //         scheduler->PublishOnce();
    //         ros::spinOnce();
    //         loop_rate.sleep();
    //     }
    // });

    // =============================================
    // 5️⃣ 主线程：ROS 回调处理
    // =============================================
    ROS_INFO("[road_side_system_node] ✅ All systems running. Spinning main thread...");
    ros::spin();

    // =============================================
    // 6️⃣ 退出处理
    // =============================================
    if (tl_thread.joinable())
        tl_thread.join();
    // if (scheduler_thread.joinable())
    //     scheduler_thread.join();

    ROS_INFO("[road_side_system_node] 🛑 Shutdown complete.");
    return 0;
}
