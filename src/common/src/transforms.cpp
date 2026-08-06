#include <common/transforms.h>

namespace common {

    carla::geom::Location ros_point_to_carla_location(const geometry_msgs::Point& ros_point) {
        // 将 ROS 中的 Point 转换为 CARLA 的 Location，y 轴反向转换
        return carla::geom::Location(ros_point.x, -ros_point.y, ros_point.z);
    }

    carla::geom::Rotation RPY_to_carla_rotation(double roll, double pitch, double yaw) {
        // 将弧度转换为角度，并调整 pitch 和 yaw 的符号
        return carla::geom::Rotation(
            -pitch * 180.0 / M_PI,
            -yaw * 180.0 / M_PI,
            roll * 180.0 / M_PI);
    }

    carla::geom::Rotation ros_quaternion_to_carla_rotation(const geometry_msgs::Quaternion& ros_quaternion) {
        // 将 ROS 四元数转换为 tf2::Quaternion 类型
        tf2::Quaternion tf_quat(ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w);

        // 使用 tf2 的 Matrix3x3 来从四元数提取欧拉角（roll, pitch, yaw）
        tf_quat.normalize();
        tf2::Matrix3x3 tf_mat(tf_quat);
        double roll, pitch, yaw;
        tf_mat.getRPY(roll, pitch, yaw);

        // 返回通过 RPY 转换得到的 CARLA 旋转
        return RPY_to_carla_rotation(roll, pitch, yaw);
    }

    carla::geom::Transform ros_pose_to_carla_transform(const geometry_msgs::Pose& ros_pose){
        // 将 ROS 的位置和方向转换为 CARLA 的 Location 和 Rotation
        carla::geom::Location carla_location = ros_point_to_carla_location(ros_pose.position);
        carla::geom::Rotation carla_rotation = ros_quaternion_to_carla_rotation(ros_pose.orientation);

        // 返回一个 CARLA Transform 对象，包含位置和旋转
        return carla::geom::Transform(carla_location, carla_rotation);
    }

    geometry_msgs::Point carla_location_to_ros_point(const carla::geom::Location& carla_location){
        // 创建 ROS 的点对象
        geometry_msgs::Point ros_point;

        // 将 Carla 的 Location 转换为 ROS 的 Point（注意反转 y 坐标）
        ros_point.x = carla_location.x;
        ros_point.y = -carla_location.y;  // 反转 y 坐标
        ros_point.z = carla_location.z;

        return ros_point;
    }

    std::tuple<double, double, double> carla_rotation_to_RPY(const carla::geom::Rotation& carla_rotation) {
        // 将 CARLA 的旋转角度从度转换为弧度，并考虑左手坐标系到右手坐标系的转换

        double roll = carla_rotation.roll * M_PI / 180.0;   // 度转弧度
        double pitch = -carla_rotation.pitch * M_PI / 180.0; // 注意方向反转
        double yaw = -carla_rotation.yaw * M_PI / 180.0;     // 注意方向反转

        return std::make_tuple(roll, pitch, yaw);
    }

    geometry_msgs::Vector3 carla_location_to_ros_vector3(const carla::geom::Location& carla_location){
        // 创建 ROS 的 Vector3 消息
        geometry_msgs::Vector3 ros_translation;

        // 将 CARLA 的坐标转换为 ROS 坐标
        ros_translation.x = carla_location.x;
        ros_translation.y = -carla_location.y;  // 转换 y 轴方向
        ros_translation.z = carla_location.z;

        return ros_translation;
    }

    geometry_msgs::Quaternion carla_rotation_to_ros_quaternion(const carla::geom::Rotation& carla_rotation) {
         geometry_msgs::Quaternion ros_quaternion;
        // 将 CARLA 旋转（度）转换为 ROS 使用的欧拉角（弧度）
        double roll, pitch, yaw;
        std::tie(roll, pitch, yaw) = carla_rotation_to_RPY(carla_rotation);

        // 将欧拉角（roll, pitch, yaw）转换为四元数
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        if (quat.length2() < 1e-12) {
            quat.setValue(0.0, 0.0, 0.0, 1.0);  // 防止 NaN
        } else {
            quat.normalize();
        }
        // 填充 ROS 四元数消息
        ros_quaternion.x = quat.x();
        ros_quaternion.y = quat.y();
        ros_quaternion.z = quat.z();
        ros_quaternion.w = quat.w();

        return ros_quaternion;
    }

    geometry_msgs::Transform carla_transform_to_ros_transform(const carla::geom::Transform& carla_transform){
        geometry_msgs::Transform ros_transform;

        // 转换位置（Location -> Vector3）
        ros_transform.translation = carla_location_to_ros_vector3(carla_transform.location);

        // 转换旋转（Rotation -> Quaternion）
        ros_transform.rotation = carla_rotation_to_ros_quaternion(carla_transform.rotation);

        return ros_transform;

    }

    // 将 carla::geom::Transform 转换为 ROS 的 Pose
    geometry_msgs::Pose carla_transform_to_ros_pose(const carla::geom::Transform& carla_transform) {
        // 从 CARLA 的 Transform 获取位置和旋转
        carla::geom::Location carla_location = carla_transform.location;
        carla::geom::Rotation carla_rotation = carla_transform.rotation;

        // 转换为 ROS 的 Pose
        geometry_msgs::Pose ros_pose;
        ros_pose.position = carla_location_to_ros_point(carla_location);
        ros_pose.orientation = carla_rotation_to_ros_quaternion(carla_rotation);

        return ros_pose;
    }

    geometry_msgs::PoseArray carla_path_to_ros_posearray(const std::vector<carla::geom::Transform>& interp_path,
                                                        const std_msgs::Header& header) {
        geometry_msgs::PoseArray pose_array;
        pose_array.header = header;  // 可选：传入 frame_id 和 stamp

        for (const auto& tf : interp_path) {
            pose_array.poses.push_back(carla_transform_to_ros_pose(tf));
        }

        return pose_array;
    }

    geometry_msgs::Twist rotate_velocity_to_vehicle_heading_ros(
        const geometry_msgs::Twist& twist, 
        const geometry_msgs::Pose& ros_pose) {

        // 获取车辆的旋转信息 (四元数)
        tf2::Quaternion quaternion;
        tf2::fromMsg(ros_pose.orientation, quaternion);

        // 将四元数转换为旋转矩阵（3x3矩阵）
        Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
        Eigen::Quaternionf eigen_quaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
        rotation_matrix = eigen_quaternion.toRotationMatrix();

        // 将ROS Twist中的线速度转为Eigen向量
        Eigen::Vector3f linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);

        // 旋转线速度
        Eigen::Vector3f rotated_linear_velocity = rotation_matrix * linear_velocity;

        // 创建一个新的ROS Twist并赋值
        geometry_msgs::Twist rotated_twist;
        rotated_twist.linear.x = rotated_linear_velocity(0);
        rotated_twist.linear.y = rotated_linear_velocity(1);
        rotated_twist.linear.z = rotated_linear_velocity(2);

        // 保持角速度不变（如果需要旋转角速度，也可以类似进行旋转）
        rotated_twist.angular.x = twist.angular.x;
        rotated_twist.angular.y = twist.angular.y;
        rotated_twist.angular.z = twist.angular.z;

        return rotated_twist;
    }
}