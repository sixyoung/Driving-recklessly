#include "trajectory_stitcher.h"
#include "vehicle_model.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace planner {

// 重新初始化 stitching（从当前车辆状态出发）
TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.set_s(0.0);
  init_point.set_x(vehicle_state.x());
  init_point.set_y(vehicle_state.y());
  init_point.set_z(vehicle_state.z());
  init_point.set_theta(vehicle_state.heading());
  init_point.set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(planning_cycle_time);

  return init_point;
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// 对上次的轨迹进行坐标变换（坐标系更新）
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](TrajectoryPoint& p) {
                  auto x = p.x;
                  auto y = p.y;
                  auto theta = p.theta;

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = theta - theta_diff;

                  p.set_x(x_new);
                  p.set_y(y_new);
                  p.set_theta(theta_new);
                });
}

// stitching 逻辑
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
    const Param_Configs* Config_) {
  if (!prev_trajectory) {
    if (replan_reason) *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();
  if (prev_trajectory_size == 0) {
    if (replan_reason) *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();
  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 && veh_rel_time < 0.0) {
    if (replan_reason) {
      *replan_reason = "replan for current time < first trajectory point.";
    }
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    if (replan_reason) {
      *replan_reason = "replan for current time > last trajectory point.";
    }
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  auto time_matched_point =
      prev_trajectory->TrajectoryPointAt(time_matched_index);

  size_t position_matched_index =
      prev_trajectory->QueryNearestPointWithBuffer(
          {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(position_matched_index));

  auto lon_diff = time_matched_point.s - frenet_sd.first;

  auto lat_diff = frenet_sd.second;

  if (std::fabs(lat_diff) > Config_->FLAGS_replan_lateral_distance_threshold) {  // 阈值可调
    if (replan_reason) {
      *replan_reason = "replan for large lateral deviation.";
    }
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (std::fabs(lon_diff) > Config_->FLAGS_replan_longitudinal_distance_threshold) {
      if (replan_reason) {
          if (lon_diff < 0) {
              *replan_reason = "replan: vehicle ahead of trajectory (超前).";
          } else {
              *replan_reason = "replan: vehicle behind trajectory (滞后).";
          }
      }
      ROS_ERROR("[Stitching][%s] |Δlon|=%.3f > %.3f [触发重规划原因: %s]",
                Config_->role_name_.c_str(),
                lon_diff,
                Config_->FLAGS_replan_longitudinal_distance_threshold,
                replan_reason->c_str());
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
double tmi_rel_time = prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time;
double forward_rel_time = tmi_rel_time + planning_cycle_time;;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);


  auto matched_index = std::min(time_matched_index, position_matched_index);

  // 限制最多保存过去3s的轨迹点
  size_t past_index = prev_trajectory->QueryLowerBoundPoint(-0.5);

  constexpr size_t kNumPreCyclePoint = 0; // 或者 20

  size_t start_index = std::max(
      past_index,
      matched_index >= kNumPreCyclePoint ? matched_index - kNumPreCyclePoint : 0UL
  );

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() + start_index,
      prev_trajectory->begin() + forward_time_index + 1);

  const double zero_s = stitching_trajectory.back().s;
  for (auto& tp : stitching_trajectory) {
    tp.set_relative_time(tp.relative_time + prev_trajectory->header_time() - current_timestamp);
    tp.set_s(tp.s - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  double dx = x - p.x;
  double dy = y - p.y;
  double cos_theta = std::cos(p.theta);
  double sin_theta = std::sin(p.theta);

  double ds = dx * cos_theta + dy * sin_theta;
  double dl = -dx * sin_theta + dy * cos_theta;
  return {p.s + ds, dl};
}

}  // namespace planning
