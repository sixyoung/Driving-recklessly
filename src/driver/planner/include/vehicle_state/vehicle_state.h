#pragma once

#include <mutex>
#include <cmath>
/**
 * @class VehicleState
 * @brief 存储车辆的基本状态 (位置、姿态、速度等)
 */
class VehicleState {
public:
  VehicleState() = default;

  // --- getters ---
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double heading() const { return heading_; }
  double roll() const { return roll_; }
  double pitch() const { return pitch_; }
  double yaw() const { return yaw_; }
  double kappa() const { return kappa_; }
  double linear_velocity() const { return linear_velocity_; }
  double angular_velocity() const { return angular_velocity_; }
  double linear_acceleration() const { return linear_acceleration_; }
  double timestamp() const { return timestamp_; }

  // --- setters ---
  void set_x(double v) { x_ = v; }
  void set_y(double v) { y_ = v; }
  void set_z(double v) { z_ = v; }
  void set_heading(double v) { heading_ = v; }
  void set_roll(double v) { roll_ = v; }
  void set_pitch(double v) { pitch_ = v; }
  void set_yaw(double v) { yaw_ = v; }
  void set_kappa(double v) { kappa_ = v; }
  void set_linear_velocity(double v) { linear_velocity_ = v; }
  void set_angular_velocity(double v) { angular_velocity_ = v; }
  void set_linear_acceleration(double v) { linear_acceleration_ = v; }
  void set_timestamp(double v) { timestamp_ = v; }

private:
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  double heading_ = 0.0;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  double kappa_ = 0.0;                 // 曲率 = yaw_rate / v
  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;      // yaw_rate
  double linear_acceleration_ = 0.0;
  double timestamp_ = 0.0;
};
