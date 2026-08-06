#include "vehicle_state_provider.h"
#include <cmath>

void VehicleStateProvider::Update(double x, double y, double z,
                                  double qw, double qx, double qy, double qz,
                                  double v, double a, double yaw_rate, double ts) {
  std::lock_guard<std::mutex> lock(mtx_);

  // 位置和姿态
  if (!ConstructExceptLinearVelocity(x, y, z, qw, qx, qy, qz, yaw_rate, a, ts)) {
    throw std::runtime_error("Failed to construct vehicle state");
  }

  // 速度 & 加速度
  state_.set_linear_velocity(v);
  state_.set_linear_acceleration(a);
  state_.set_angular_velocity(yaw_rate);
  state_.set_timestamp(ts);

  // 曲率 kappa = yaw_rate / v
  constexpr double kEpsilon = 1e-6;
  if (std::fabs(v) < kEpsilon) {
    state_.set_kappa(0.0);
  } else {
    state_.set_kappa(yaw_rate / v);
  }
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    double x, double y, double z,
    double qw, double qx, double qy, double qz,
    double yaw_rate, double a, double ts) {
  state_.set_x(x);
  state_.set_y(y);
  state_.set_z(z);

  // 四元数 → 欧拉角（手动计算，更稳定）
  Eigen::Quaterniond q(qw, qx, qy, qz);

  // roll (绕 x)
  double roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  // pitch (绕 y)
  double pitch = std::asin(std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
  // yaw (绕 z)
  double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  // 归一化到 [-π, π]
  auto normalize_angle = [](double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  };

  roll = normalize_angle(roll);
  pitch = normalize_angle(pitch);
  yaw = normalize_angle(yaw);

  state_.set_roll(roll);
  state_.set_pitch(pitch);
  state_.set_yaw(yaw);

  // 航向角 (heading = yaw)
  state_.set_heading(yaw);

  return true;
}


Eigen::Vector2d VehicleStateProvider::EstimateFuturePosition(double t) const {
  std::lock_guard<std::mutex> lock(mtx_);

  double v = state_.linear_velocity();
  double yaw_rate = state_.angular_velocity();

  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  if (std::fabs(yaw_rate) < 1e-4) {
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / yaw_rate * (1.0 - std::cos(yaw_rate * t));
    vec_distance[1] = std::sin(yaw_rate * t) * v / yaw_rate;
  }

  Eigen::Quaterniond q(std::cos(state_.heading() / 2),
                       0, 0,
                       std::sin(state_.heading() / 2));
  Eigen::Vector3d pos_vec(state_.x(), state_.y(), state_.z());
  Eigen::Vector3d future_pos = q.toRotationMatrix() * vec_distance + pos_vec;

  return {future_pos[0], future_pos[1]};
}

Eigen::Vector2d VehicleStateProvider::ComputeCOMPosition(
    double rear_to_com_distance) const {
  std::lock_guard<std::mutex> lock(mtx_);

  Eigen::Vector3d offset(0.0, rear_to_com_distance, 0.0);
  Eigen::Vector3d pos_vec(state_.x(), state_.y(), state_.z());

  Eigen::Quaterniond q(std::cos(state_.heading() / 2),
                       0, 0,
                       std::sin(state_.heading() / 2));
  Eigen::Vector3d com_pos = q.toRotationMatrix() * offset + pos_vec;

  return {com_pos[0], com_pos[1]};
}
