#include "driver_models/driver_model_manager.h"
#include <ros/ros.h>
#include <string>
#include <exception>
#include <thread>  // std::thread::hardware_concurrency

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_model_manager_node");
  setlocale(LC_ALL, "C.UTF-8");

  ros::NodeHandle nh("~");

  // ==== 参数配置 ====
  std::string host;
  int port;
  nh.param<std::string>("carla_host", host, "localhost");
  nh.param<int>("carla_port", port, 2000);

  // ==== 等待关键服务 (可选) ====
  const std::string required_service = "/carla_waypoint_publisher/get_path";
  if (!ros::service::waitForService(required_service, ros::Duration(10.0))) {
    ROS_ERROR("Required service [%s] not available. Exiting.", required_service.c_str());
    return 1;
  }

  // ==== 连接 CARLA ====
  carla::client::Client client(host, port);
  client.SetTimeout(std::chrono::duration<double>(10.0));
  carla::client::World world = client.GetWorld();

  // ==== 创建 DriverModelManager ====
  unsigned int hw_threads = std::thread::hardware_concurrency();
  unsigned int physical_cores = hw_threads / 2;   // 简单估算：逻辑 / 2

  // TaskScheduler = 物理核心数
  size_t pool_size = std::max(4u, physical_cores);
  DriverModelManager manager(nh, world, pool_size);
  ROS_INFO("✅ DriverModelManager node started with pool_size=%zu", pool_size);

  // AsyncSpinner = 2~4 个线程
  ros::AsyncSpinner spinner(4);
  spinner.start();

  try {
    ros::waitForShutdown(); 
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in driver_model_manager_node: %s", e.what());
  } catch (...) {
    ROS_ERROR("Unknown exception in driver_model_manager_node");
  }

  return 0;
}
