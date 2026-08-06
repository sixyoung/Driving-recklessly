#pragma once

#include "vehicle_state.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

class VehicleStateProvider {
public:
  VehicleStateProvider() = default;

  /**
   * @brief 从 ROS odometry 更新车辆状态
   * @param x, y, z      位置
   * @param qw, qx, qy, qz 四元数 (姿态)
   * @param v            线速度 (m/s)
   * @param a            线加速度 (m/s^2)
   * @param yaw_rate     横摆角速度 (rad/s)
   * @param ts           时间戳 (秒)
   */
  void Update(double x, double y, double z,
              double qw, double qx, double qy, double qz,
              double v, double a, double yaw_rate, double ts);

  // 预测未来位置
  Eigen::Vector2d EstimateFuturePosition(double t) const;

  // 计算质心位置
  Eigen::Vector2d ComputeCOMPosition(double rear_to_com_distance) const;

  // Getter 接口
  double x() const { return state_.x(); }
  double y() const { return state_.y(); }
  double z() const { return state_.z(); }
  double roll() const { return state_.roll(); }
  double pitch() const { return state_.pitch(); }
  double yaw() const { return state_.yaw(); }
  double heading() const { return state_.heading(); }
  double kappa() const { return state_.kappa(); }
  double linear_velocity() const { return state_.linear_velocity(); }
  double angular_velocity() const { return state_.angular_velocity(); }
  double linear_acceleration() const { return state_.linear_acceleration(); }
  double timestamp() const { return state_.timestamp(); }

  const VehicleState& vehicle_state() const { return state_; }

private:
  VehicleState state_;
  mutable std::mutex mtx_;

  bool ConstructExceptLinearVelocity(double x, double y, double z,
                                     double qw, double qx, double qy, double qz,
                                     double yaw_rate, double a, double ts);
};