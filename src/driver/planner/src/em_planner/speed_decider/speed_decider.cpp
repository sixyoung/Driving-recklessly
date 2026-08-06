#include "speed_decider/speed_decider.h"
#include "Configs.h"
#include <iostream>
#include <cmath>

SpeedDecider::SpeedDecider(const TrajectoryPoint &planning_init_point, const SL_Boundary& adc_sl_boundary, 
                            const ReferenceLine* reference_line, const std::string &role_name, const Param_Configs &cfg)
    : init_point_(planning_init_point), adc_sl_boundary_(adc_sl_boundary),reference_line_(reference_line),role_name_(role_name), Config_(&cfg) {
  // std::cout << "[SpeedDecider] initialized." << std::endl;
}
// ================= 主入口 =================
bool SpeedDecider::Execute(const SpeedData& speed_data,
                           std::vector<Obstacle*>& obstacles) {
  if (!MakeObjectDecision(speed_data, obstacles)) {
    std::cerr << "[SpeedDecider] Get object decision by speed profile failed."
              << std::endl;
    return false;
  }

  // === 3. 成功返回 ===
  return true;
}


bool SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile,
    std::vector<Obstacle*>& obstacles) const {
  if (speed_profile.size() < 2) {
    std::cerr << "[SpeedDecider] dp_st_graph failed to get speed profile."
              << std::endl;
    return false;
  }

  for (auto* obstacle : obstacles) {
    if (!obstacle) continue;
    if (obstacle->obstacle_type == 9 || obstacle->obstacle_type == 10) {
      ROS_INFO("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=type(%d)",
              role_name_.c_str(), obstacle->obstacle_id.c_str(), obstacle->obstacle_type);
      AppendIgnoreDecision(*obstacle);
      continue;
    }

    auto& boundary = obstacle->path_st_boundary();

    if (boundary.IsEmpty()) {
      // ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=boundary is empty",
      //         role_name_.c_str(), obstacle->obstacle_id.c_str());
      AppendIgnoreDecision(*obstacle);
      continue;
    }

    if (boundary.max_s_ < 0.0) {
      // ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=boundary.max_s_ < 0.0 (%.2f)",
      //         role_name_.c_str(), obstacle->obstacle_id.c_str(), boundary.max_s_);
      AppendIgnoreDecision(*obstacle);
      continue;
    }

    if (boundary.max_t_ < 0.0) {
      // ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=boundary.max_t_ < 0.0 (%.2f)",
      //         role_name_.c_str(), obstacle->obstacle_id.c_str(), boundary.max_t_);
      AppendIgnoreDecision(*obstacle);
      continue;
    }

    if (boundary.min_t_ >= speed_profile.back().t) {
      ROS_ERROR_THROTTLE(1.0,
          "[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=boundary.min_t_ >= speed_profile.back().t (%.2f >= %.2f)",
          role_name_.c_str(), obstacle->obstacle_id.c_str(),
          boundary.min_t_, speed_profile.back().t);

      AppendIgnoreDecision(*obstacle);
      continue;
    }

    if (obstacle->HasLongitudinalDecision()) {
      // ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=already has longitudinal decision",
      //         role_name_.c_str(), obstacle->obstacle_id.c_str());
      AppendIgnoreDecision(*obstacle);
      continue;
    }

    auto position = GetStPosition(speed_profile, boundary);

    if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
      if (CheckKeepClearBlocked(obstacles, *obstacle)) {
        position = StPosition::BELOW;
      }
    }

    auto box = obstacle->PerceptionBoundingBox();
    SLPoint start_sl_point;
    reference_line_->XYToSL({box.center_x(), box.center_y()}, &start_sl_point);
    double start_abs_l = std::abs(start_sl_point.l);

    // if(role_name_ == "hero3" || role_name_ == "hero10"){
    //     ROS_INFO("[SpeedDecider][%s] obstacle_id[%s] max_t=%.2f min_t=%.2f lateral=%.2f",
    //             role_name_.c_str(), obstacle->obstacle_id.c_str(),
    //             boundary.max_t_, boundary.min_t_, start_abs_l);
    // }
    switch (position) {
      case StPosition::BELOW:
        if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
          if (CreateStopDecision(*obstacle, &obstacle->longitudinal_decision_, 0.0, 1e9)) {
            obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
          }
        } else if (CheckIsFollowByT(boundary) &&
                   (boundary.max_t_ - boundary.min_t_ > 2.0 /* follow_min_time_sec */) &&
                   start_abs_l < 2.0 /* follow_min_obs_lateral_distance */) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*obstacle)) {
            if (CreateStopDecision(*obstacle, &obstacle->longitudinal_decision_,
                                   -Config_->FLAGS_min_stop_distance_obstacle,
                                   1e9)) {
              obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
            }
          } else {
            // FOLLOW decision
            if (CreateFollowDecision(*obstacle, &obstacle->longitudinal_decision_, 1e9)) {
              obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
            }
          }
        } else {
          // YIELD decision
          if (CreateYieldDecision(*obstacle, &obstacle->longitudinal_decision_, 1e9)) {
            obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
          }
        }
        break;

      case StPosition::ABOVE:
        if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
          ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=KEEP_CLEAR ABOVE",
                  role_name_.c_str(), obstacle->obstacle_id.c_str());
          AppendIgnoreDecision(*obstacle);
        } else {
          // OVERTAKE decision
          if (CreateOvertakeDecision(*obstacle, &obstacle->longitudinal_decision_, 1e9)) {
            obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
          }
        }
        break;

      case StPosition::CROSS:
        if (obstacle->path_st_boundary().boundary_type() != ST_Boundary::BoundaryType::KEEP_CLEAR &&
            obstacle->IsStatic()) {
          if (CreateStopDecision(*obstacle, &obstacle->longitudinal_decision_,
                                 -Config_->FLAGS_min_stop_distance_obstacle, 1e9)) {
            obstacle->SetLongitudinalDecision(obstacle->longitudinal_decision_);
          }
          std::cerr << "[SpeedDecider]Failed to find solution for crossing obstacle: "
                    << obstacle->obstacle_id << std::endl;
          return false;
        }
        break;

      default:
        std::cerr << "[SpeedDecider] Unknown position" << std::endl;
    }
    // 只有当障碍物还没有决策时，才设置 IGNORE
    if (!obstacle->HasLongitudinalDecision()) {
        ROS_ERROR("[SpeedDecider][%s] IGNORE obstacle_id[%s] reason=no decision after evaluation",
                role_name_.c_str(), obstacle->obstacle_id.c_str());
        AppendIgnoreDecision(*obstacle);
    }
  }

  return true;
}

// ================= 工具函数 =================
SpeedDecider::StPosition SpeedDecider::GetStPosition(
    const SpeedData& speed_profile,
    const ST_Boundary& st_boundary) const {
  StPosition st_position = StPosition::BELOW;
  if (st_boundary.IsEmpty()) {
    return st_position;
  }

  bool st_position_set = false;
  const double start_t = st_boundary.min_t_;
  const double end_t = st_boundary.max_t_;

  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const STPoint curr_st(speed_profile[i].s, speed_profile[i].t);
    const STPoint next_st(speed_profile[i + 1].s, speed_profile[i + 1].t);

    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    LineSegment2d speed_line(curr_st, next_st);
    if (st_boundary.HasOverlap(speed_line)) {
      std::cout << "[SpeedDecider] speed profile cross st_boundaries."
                << std::endl;
      st_position = StPosition::CROSS;

      if (st_boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
        if (!CheckKeepClearCrossable(speed_profile, st_boundary)) {
          st_position = StPosition::BELOW;
        }
      }
      break;
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        STPoint bd_point_front = st_boundary.getUpper_points().front();
        double side = math::CrossProd(bd_point_front, curr_st, next_st);
        st_position = side < 0.0 ? StPosition::ABOVE : StPosition::BELOW;
        st_position_set = true;
      }
    }
  }
  return st_position;
}

bool SpeedDecider::CheckKeepClearCrossable(
    const SpeedData& speed_profile,
    const ST_Boundary& keep_clear_st_boundary) const {
  bool keep_clear_crossable = true;

  if (speed_profile.empty()) {
    return true;  // 没有速度点，直接返回可通行
  }

  const auto& last_speed_point = speed_profile.back();

  // 获取最后一个点的速度
  double last_speed_point_v = 0.0;
  if (last_speed_point.v > 1e-6) {  
    // 如果结构体直接存速度
    last_speed_point_v = last_speed_point.v;
  } else {
    // 没有速度时，用最后两个点估算
    const size_t len = speed_profile.size();
    if (len > 1) {
      const auto& last_2nd_speed_point = speed_profile[len - 2];
      double ds = last_speed_point.s - last_2nd_speed_point.s;
      double dt = last_speed_point.t - last_2nd_speed_point.t;
      if (dt > 1e-3) {
        last_speed_point_v = ds / dt;
      }
    }
  }

  constexpr double kKeepClearSlowSpeed = 2.5;  // m/s

  std::cout << "[SpeedDecider] last_speed_point_s[" << last_speed_point.s
            << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s_
            << "] last_speed_point_v[" << last_speed_point_v << "]"
            << std::endl;

  // 规则：如果自车的终点还在 keep_clear 内，且速度小于阈值，认为不能通过
  if (last_speed_point.s <= keep_clear_st_boundary.max_s_ &&
      last_speed_point_v < kKeepClearSlowSpeed) {
    keep_clear_crossable = false;
  }

  return keep_clear_crossable;
}

bool SpeedDecider::CheckKeepClearBlocked(
    const std::vector<Obstacle*>& obstacles,
    const Obstacle& keep_clear_obstacle) const {
  bool keep_clear_blocked = false;

  for (auto* obstacle : obstacles) {   // 注意：这里不再是 const auto*
    if (!obstacle) continue;
    if (obstacle->obstacle_id == keep_clear_obstacle.obstacle_id) {
      continue;
    }

    double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s_;
    double keep_clear_end_s = keep_clear_obstacle.PerceptionSLBoundary().end_s_;
    double distance = obstacle_start_s - keep_clear_end_s;

    if ((obstacle->path_st_boundary().boundary_type() != ST_Boundary::BoundaryType::KEEP_CLEAR) 
        && distance > 0.0
        && distance < (Config_->FLAGS_vehicle_length / 2.0)) {
      keep_clear_blocked = true;
      break;
    }
  }

  return keep_clear_blocked;
}

bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  if (obstacle.path_st_boundary().boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
    return false;
  }

  if (obstacle.path_st_boundary().min_t_ > 0.0) {
    return false;
  }
  const double obs_speed = obstacle.obstacle_velocity;
  const double ego_speed = init_point_.v;
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      obstacle.path_st_boundary().min_s_ - Config_->FLAGS_min_stop_distance_obstacle;
  constexpr double decel = 1.0;
  return distance < std::pow((ego_speed - obs_speed), 2) * 0.5 / decel;
}

bool SpeedDecider::CheckIsFollowByT(const ST_Boundary& boundary) const {
  if (boundary.bottom_left_point_.s() > boundary.bottom_right_point_.s()) {
    return false;
  }
  constexpr double kFollowTimeEpsilon = 1e-3;
  constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t_ > kFollowCutOffTime ||
      boundary.max_t_ < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}


// ================= 决策生成 =================
bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* stop_decision,
                                      double stop_distance,
                                      double main_stop_s) const {
  if (!stop_decision) return false;

  const auto& boundary = obstacle.path_st_boundary();

  // 默认停车位置：自车车尾 + 障碍物前沿 + 安全距离
  double fence_s = adc_sl_boundary_.end_s_ + boundary.min_s_ + stop_distance;

  // KEEP_CLEAR 特殊情况：在 KEEP_CLEAR 起点停车
  if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s_;
  }

  // 如果已有更近的 STOP fence，忽略
  if (main_stop_s < fence_s) {
    ROS_INFO("[SpeedDecider] Stop fence is further away, ignore.");
    return false;
  }

  // === 设置 STOP 决策 ===
  stop_decision->tag = DecisionTag::STOP;
  stop_decision->stop.distance_s = stop_distance;

  // 在没有 reference_line 的情况下，可以直接保存 s 坐标
  stop_decision->stop.stop_heading = 0.0;  // 这里缺 heading，可以后接 reference line

  if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR) {
    stop_decision->stop.reason_code = StopReasonCode::STOP_REASON_CLEAR_ZONE;
  }

  // ROS_INFO("[SpeedDecider] STOP: obstacle_id[%s] at fence_s=%.2f",
  //         obstacle.obstacle_id.c_str(),
  //         fence_s);

  return true;
}


bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* follow_decision,
    double main_stop_s
) const {
  if (!follow_decision) return false;

  const auto& boundary = obstacle.path_st_boundary();
  const double follow_speed = init_point_.v;
  // 跟车安全距离：取 max(v * t_buffer, min_dist)，再取负号（表示落后）
  const double follow_distance_s =
      -std::fmax(follow_speed * 1.5, 4.0);

  // 参考 s 位置
  const double reference_s = adc_sl_boundary_.end_s_  + boundary.min_s_ + follow_distance_s;

  // 如果已有更近的 STOP fence，就忽略
  if (main_stop_s < reference_s) {
     ROS_INFO("[SpeedDecider] Follow reference_s is further away, ignore.");

    return false;
  }

  // === 设置 FOLLOW 决策 ===
  follow_decision->tag = DecisionTag::FOLLOW;
  follow_decision->follow.distance_s = follow_distance_s;

  // 没有 reference_line，这里只存纵向位置
  follow_decision->follow.fence_point = reference_s;
  follow_decision->follow.fence_heading = 0.0;

  // ROS_INFO("[SpeedDecider] FOLLOW: obstacle_id[%s] at reference_s=%.2f follow_distance=%.2f",
  //         obstacle.obstacle_id.c_str(),
  //         reference_s,
  //         follow_distance_s);

  return true;
}


bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* yield_decision,
    double main_stop_s,
    double yield_distance_vehicle,    // 车辆默认让行距离
    double yield_distance_ped_bike   // 行人/自行车更大让行距离
) const {
  if (!yield_decision) return false;

  // 根据障碍物类型选择让行距离
  double yield_distance = yield_distance_vehicle;
  if (obstacle.obstacle_type == 6 ||
      obstacle.obstacle_type == 4) {
    yield_distance = yield_distance_ped_bike;
  }

  const auto& boundary = obstacle.path_st_boundary();

  // 保证在障碍物 min_s 之前 yield
  const double yield_distance_s =
      std::max(-boundary.min_s_, -yield_distance);

  // 计算参考 fence 位置
  const double reference_s = adc_sl_boundary_.end_s_ + boundary.min_s_ + yield_distance_s;

  // 如果已有更近的 STOP fence，则忽略
  if (main_stop_s < reference_s) {
    ROS_INFO("[SpeedDecider] Yield reference_s is further away, ignore.");
    return false;
  }

  // === 设置 YIELD 决策 ===
  yield_decision->tag = DecisionTag::YIELD;
  yield_decision->yield.distance_s = yield_distance_s;

  // 只存纵向位置
  yield_decision->yield.fence_point = reference_s;
  yield_decision->yield.fence_heading = 0.0;
  // ROS_INFO("[SpeedDecider] YIELD: obstacle_id[%s] type=%d reference_s=%.2f yield_distance=%.2f",
  //         obstacle.obstacle_id.c_str(),
  //         obstacle.obstacle_type,
  //         reference_s,
  //         yield_distance);

  return true;
}


bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* overtake_decision,
    double main_stop_s
) const {
  if (!overtake_decision) return false;

  constexpr double kOvertakeTimeBuffer = 3.0;    // s
  constexpr double kMinOvertakeDistance = 10.0;  // m

  // 计算障碍物在自车方向上的速度分量
  double obs_speed = obstacle.obstacle_velocity;
  double obs_theta = obstacle.obstacle_threa;

  double dir_x = std::cos(init_point_.v);
  double dir_y = std::sin(init_point_.v);
  double obs_vx = obs_speed * std::cos(obs_theta);
  double obs_vy = obs_speed * std::sin(obs_theta);

  double obstacle_speed = dir_x * obs_vx + dir_y * obs_vy;

  // 计算超车所需纵向距离
  double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v, obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);

  // 超车 fence 位置
  const auto& boundary = obstacle.path_st_boundary();
  double reference_s = adc_sl_boundary_.end_s_ + boundary.min_s_ + overtake_distance_s;

  // 已有更近的 STOP fence，则忽略
  if (main_stop_s < reference_s) {
    ROS_INFO("[SpeedDecider] Overtake reference_s is further away, ignore.");
    return false;
  }

  // === 设置 OVERTAKE 决策 ===
  overtake_decision->tag = DecisionTag::OVERTAKE;
  overtake_decision->overtake.distance_s = overtake_distance_s;
  overtake_decision->overtake.fence_point = reference_s;
  overtake_decision->overtake.fence_heading = 0.0;

  // ROS_INFO("[SpeedDecider] OVERTAKE: obstacle_id[%s] reference_s=%.2f overtake_distance=%.2f",
  //         obstacle.obstacle_id.c_str(),
  //         reference_s,
  //         overtake_distance_s);

  return true;
}


void SpeedDecider::AppendIgnoreDecision(Obstacle& obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.tag = DecisionTag::IGNORE;
  obstacle.SetLongitudinalDecision(ignore_decision);
}
