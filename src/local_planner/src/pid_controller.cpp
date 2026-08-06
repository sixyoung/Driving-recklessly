// This file is used to implement the PID controller algorithm. 
#include"local_planner/pid_controller.h" 
#define M_PI 3.14159265358979323846

double quat2yaw(const geometry_msgs::Quaternion& quat) {
    // 使用 tf2 库将四元数转换为欧拉角
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    if (q.length() == 0) {
        ROS_DEBUG("Invalid quaternion: w = %f, x = %f, y = %f, z = %f", quat.w, quat.x, quat.y, quat.z);
        return 0.0;  // 返回一个默认值或处理逻辑
    }
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // 获取 Roll, Pitch, Yaw
    return yaw;
}

// PID控制器的实现
carla_msgs::CarlaEgoVehicleControl PIDController::run_step(double target_speed, double current_speed, const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &waypoint) {
    carla_msgs::CarlaEgoVehicleControl control;
    auto [throttle, brake] = lon_controller.run_step(target_speed, current_speed);
    control.throttle = throttle;
    control.brake = brake;
    // control.throttle = 0; 
    control.steer = lat_controller.run_step(current_pose, waypoint, current_speed);
    // control.steer = -lat_controller.run_step(current_pose, waypoint);
    control.hand_brake = false;
    control.manual_gear_shift = false;
    return control; 
}

// 纵向PID控制器的实现  
std::pair<double, double> PIDLongitudinalController::run_step(double target_speed, double current_speed) {
        double previous_error = error;
        error = target_speed - current_speed;

        // std::cout << "[DEBUG] target_speed=" << target_speed
        //           << " current_speed=" << current_speed
        //           << " prev_error=" << previous_error
        //           << " error=" << error << std::endl;

        // ✅ 死区处理
        if (std::abs(error) < 0.01) {
            double effective_output = std::max(last_output.first, last_output.second);
            // std::cout << "[DEBUG] In dead zone, error=" << error 
            //           << " error_integral=" << error_integral
            //           << " -> return last_output: throttle=" << last_output.first
            //           << " brake=" << last_output.second << std::endl;
            debug_info_lo = {error, error_integral, error_derivative, 0.0, 0.0, 0.0, effective_output};
            return last_output;
        }

        error_integral = std::clamp(error_integral + error, -10.0, 10.0);
        error_derivative = (error - previous_error) / 0.02;

        double p_term = K_P * error;
        double i_term = K_I * error_integral;
        double d_term = K_D * error_derivative;
        double output = p_term + i_term + d_term;

        // 保存调试信息
        debug_info_lo = {target_speed, current_speed, error_derivative, p_term, i_term, d_term, output};

        // ✅ throttle / brake 分离，并保存 last_output
        if (output >= 0.0) {
            last_output = {std::clamp(output, 0.0, 1.0), 0.0};
            // std::cout << "[DEBUG] throttle=" << last_output.first 
            //           << " brake=" << last_output.second << std::endl;
            return last_output;
        } else if (output <= -0.05) {
            last_output = {0.0, std::clamp(-output, 0.0, 1.0)};
            // std::cout << "[DEBUG] throttle=" << last_output.first 
            //           << " brake=" << last_output.second << std::endl;
            return last_output;
        } else {
            last_output = {0.0, 0.0};
            // std::cout << "[DEBUG] throttle=0.0 brake=0.0 (deadband)" << std::endl;
            return last_output;
        }
    }

double PIDLateralController::run_step(const geometry_msgs::Pose& current_pose,
                                   const geometry_msgs::Pose& target_pose,
                                   double current_speed) {
    // ========== 1. 提取自车位置与航向 ==========
    geometry_msgs::Point ego_pos = current_pose.position;
    double ego_yaw = tf2::getYaw(current_pose.orientation);

    // 参考点位置与航向
    geometry_msgs::Point ref_pos = target_pose.position;
    double ref_yaw = tf2::getYaw(target_pose.orientation);

    // ========== 2. 航向误差 θ_e ==========
    double heading_error = ego_yaw - ref_yaw;
    // wrap 到 [-pi, pi]
    while (heading_error > M_PI)  heading_error -= 2*M_PI;
    while (heading_error < -M_PI) heading_error += 2*M_PI;

    // ========== 3. 横向误差 e_y ==========
    // 参考线切向单位向量
    Eigen::Vector2d ref_dir(cos(ref_yaw), sin(ref_yaw));
    // 从参考点指向自车的向量
    Eigen::Vector2d diff(ego_pos.x - ref_pos.x, ego_pos.y - ref_pos.y);

    // 横向误差带符号：diff 在 ref_dir 左侧为正，右侧为负
    double e_y = ref_dir[0] * diff[1] - ref_dir[1] * diff[0];

    // ========== 4. Stanley 控制律 ==========
    double k_ = 6.0;      // Stanley 增益，可调
    double epsilon_ = 0.1; // 软化项，防止除零
    double stanley_term = atan2(k_ * e_y, current_speed + epsilon_);
    double delta_raw = 1.8*heading_error + stanley_term;

    // ========== 5. 输出限幅 ==========
    double output = std::clamp(delta_raw, -1.0, 1.0);
    debug_info_la = {
        heading_error,   // heading_error
        e_y,             // 横向误差
        k_ * e_y,        // cross_term
        stanley_term,    // stanley_term
        delta_raw,       // delta_raw
        output           // 限幅后的输出
    };       
    return output; // 控制量范围 [-1,1]
}

// double PIDLateralController::run_step(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose, double current_speed) {
//     // 获取当前车辆的位置（车辆的位置是一个几何点，包含x, y, z坐标）
//     geometry_msgs::Point v_begin = current_pose.position;
    
//     // 将四元数转换为欧拉角（偏航角yaw）
//     double yaw = quat2yaw(current_pose.orientation);
    
//     // 计算车辆前进方向的向量
//     geometry_msgs::Point v_end;
//     v_end.x = v_begin.x + cos(yaw);  // 根据yaw角计算车辆朝向的x方向增量
//     v_end.y = v_begin.y + sin(yaw);  // 根据yaw角计算车辆朝向的y方向增量

//     // 将当前位置和目标位置向量转换为Eigen的三维向量
//     Eigen::Vector3d v_vec(v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0);  // 车辆前进的向量
//     Eigen::Vector3d w_vec(target_pose.position.x - v_begin.x, target_pose.position.y - v_begin.y, 0.0);  // 从当前位置到目标点的向量

//     // 计算两个向量之间的夹角（dot product 和 norm）
//     double dot_product = v_vec.dot(w_vec);  // 计算两个向量的点积
//     double norm_v = v_vec.norm();  // 计算车辆前进向量的长度
//     double norm_w = w_vec.norm();  // 计算目标向量的长度
//     double _dot = 0.0;
//     if (norm_v > 1e-6 && norm_w > 1e-6) {
//         // atan2(y, x) = atan2(|v×w|, v·w)，能直接得到带符号角度
//         Eigen::Vector3d cross_product = v_vec.cross(w_vec);
//         _dot = atan2(cross_product.z(), dot_product);
//     }

//     // PID控制计算
//     double previous_error = error;  // 保存之前的误差值
//     error = _dot;  // 更新当前误差为计算得到的夹角

//     error_integral = std::clamp(error_integral + error, -10.0, 10.0);  // 积分项，避免积分风up，限定在[-400, 400]范围
//     error_derivative = error - previous_error;  // 微分项，计算当前误差与上次误差的差值

//     // 根据PID公式计算输出控制值
//     double output = K_P * error + K_I * error_integral + K_D * error_derivative;  // PID控制计算
//     return std::clamp(output, -1.0, 1.0);  // 将输出限制在[-1, 1]范围内，确保输出不超过控制范围
// }

