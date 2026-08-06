#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <cmath> 
#include <carla/geom/Location.h> 
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>  
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/Pose.h>  
#include <geometry_msgs/Vector3.h>  
#include <geometry_msgs/Transform.h>  
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense> 
#include <eigen3/Eigen/Core> 
#include <geometry_msgs/PoseArray.h>    

namespace common {

    // 将 ROS 中的 Point 转换为 CARLA 的 Location，y 轴反向转换
    carla::geom::Location ros_point_to_carla_location(const geometry_msgs::Point& ros_point);

    // 将弧度转换为角度，并调整 pitch 和 yaw 的符号
    carla::geom::Rotation RPY_to_carla_rotation(double roll, double pitch, double yaw);

    carla::geom::Rotation ros_quaternion_to_carla_rotation(const geometry_msgs::Quaternion& ros_quaternion);

    carla::geom::Transform ros_pose_to_carla_transform(const geometry_msgs::Pose& ros_pose); 

    geometry_msgs::Point carla_location_to_ros_point(const carla::geom::Location& carla_location);

    geometry_msgs::Pose carla_transform_to_ros_pose(const carla::geom::Transform& carla_transform);

    geometry_msgs::PoseArray carla_path_to_ros_posearray(const std::vector<carla::geom::Transform>& interp_path,
                                                        const std_msgs::Header& header = std_msgs::Header());
    geometry_msgs::Vector3 carla_location_to_ros_vector3(const carla::geom::Location& carla_location);

    std::tuple<double, double, double> carla_rotation_to_RPY(const carla::geom::Rotation& carla_rotation);

    geometry_msgs::Quaternion carla_rotation_to_ros_quaternion(const carla::geom::Rotation& carla_rotation);

    geometry_msgs::Transform carla_transform_to_ros_transform(const carla::geom::Transform& carla_transform);

    geometry_msgs::Twist rotate_velocity_to_vehicle_heading_ros(const geometry_msgs::Twist& twist, const geometry_msgs::Pose& ros_pose);


}// namespace common

#endif // TRANSFORMS_H