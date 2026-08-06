/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
  Modification: Only some functions are referenced
**/
#include "dp_st_cost.h"
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>

namespace
{
  static constexpr double kDoubleEpsilon = 1.0e-3;
  constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const double total_t, const double total_s, const std::vector<const Obstacle *> &obstacles,
                   const TrajectoryPoint &init_point, const Param_Configs& cfg)
    : obstacles_(obstacles), init_point_(init_point), total_s_(total_s), Config_(&cfg), unit_t_(cfg.unit_t)
{
  int index = 0;
  for (const auto &obstacle : obstacles)
  {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  const auto dimension_t = static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) + 1;
  boundary_cost_.resize(obstacles_.size());
  for (auto &vec : boundary_cost_)
  {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(const std::vector<const Obstacle *> &obstacles)
{
  for (const auto &obstacle : obstacles)
  {
    if (obstacle->path_st_boundary().boundary_type() != ST_Boundary::BoundaryType::KEEP_CLEAR)
    {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s_;
    double end_s = obstacle->path_st_boundary().max_s_;
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(std::vector<std::pair<double, double>> *keep_clear_range)
{
  if (!keep_clear_range || keep_clear_range->empty())
  {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size())
  {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first)
    {
      ++i;
      ++j;
    }
    else
    {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second, keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const
{
  for (const auto &p : keep_clear_range_)
  {
    if (p.first <= s && p.second >= s)
    {
      return true;
    }
  }
  return false;
}

//当前节点(相对时间t，累积距离s)和障碍物运动轨迹的开销
//每个障碍物在未来的时间间隔内(例如5s，每0.1s就有一个采样的位置s)都有它的运动轨迹，也就是运动位置s_x。
//那么在相对时间t和累积距离s时刻，无人车会不会和障碍物该时刻相撞呢？这部分就需要计算无人车和障碍物在t时刻的位置信息，也就是位置cost。
//障碍物位置开销思路比较简单，循环每个障碍物，计算t时刻障碍物st边界框的上界和下届，只需要无人车的位置(t,s)与边界框不重合即可。
double DpStCost::GetObstacleCost(const StGraphPoint &st_graph_point)
{
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;


  for (const auto *obstacle : obstacles_)
  {
    // Not applying obstacle approaching cost to virtual obstacle like created
    // stop fences
    // if (obstacle->IsVirtual())
    // {
    //   continue;
    // }

    auto boundary = obstacle->path_st_boundary();
    if (boundary.min_s_ > Config_->FLAGS_speed_lon_decision_horizon) //50m外的障碍物不考虑
    {
      continue;
    }

    if (t < boundary.min_t_- 1e-3 || t > boundary.max_t_ + 1e-3)
    {
        continue;
    }
    if(boundary.obstacle_type == 9 || boundary.obstacle_type == 10){
       if(boundary.obstacle_type == 9){
          double s_upper = 0.0;
          double s_lower = 0.0;

          int boundary_index = boundary_map_[boundary.id()];
          if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0)
          {
            boundary.GetBoundarySRange(t, &s_upper, &s_lower);
            boundary_cost_[boundary_index][st_graph_point.index_t()] = std::make_pair(s_upper, s_lower);
          }
          else
          {
            s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
            s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
          }
          if (s <= s_lower)
          {
            const double follow_distance_s = Config_->safe_distance;
            if (s + follow_distance_s < s_lower)
            {
              continue;
            }
            else
            {
              auto s_diff = follow_distance_s - s_lower + s;
              cost += Config_->obstacle_weight * 100 * s_diff * s_diff;
            }
          }
       }else{
          double s_upper = 0.0;
          double s_lower = 0.0;

          int boundary_index = boundary_map_[boundary.id()];
          if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0)
          {
            boundary.GetBoundarySRange(t, &s_upper, &s_lower);
            boundary_cost_[boundary_index][st_graph_point.index_t()] = std::make_pair(s_upper, s_lower);
          }
          else
          {
            s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
            s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
          }
          if (s <= s_lower)
          {
            auto s_diff = s_lower - s;
            cost += Config_->obstacle_weight * 100 * s_diff * s_diff;
          }
       }
    }else{
      if (boundary.IsPointInBoundary(st_graph_point.point()))
      {
        ///////////////////////////////////////////////////////////
        // {
        //   if (ofs.is_open()) {
        //       ofs << std::fixed << std::setprecision(6)
        //           << "Collision at obstacle=" << boundary.id()
        //           << " t=" << st_graph_point.point().t()
        //           << " s=" << st_graph_point.point().s()
        //           << std::endl;
        //       ofs.close();
        //   } else {
        //       std::cerr << "❌ Failed to open st_collisions.txt" << std::endl;
        //   }
        // }
        ///////////////////////////////////////////////////////////
        return kInf;
      }

      double s_upper = 0.0;
      double s_lower = 0.0;

      int boundary_index = boundary_map_[boundary.id()];
      if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0)
      {
        boundary.GetBoundarySRange(t, &s_upper, &s_lower);
        boundary_cost_[boundary_index][st_graph_point.index_t()] = std::make_pair(s_upper, s_lower);
      }
      else
      {
        s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
        s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
      }
      if (s < s_lower)
      {
        double follow_distance_s = Config_->safe_distance;
        if(boundary.obstacle_type == 8){
            follow_distance_s = 3.0; 
        }
        if (s + follow_distance_s < s_lower)
        {
          continue;
        }
        else
        {
          auto s_diff = follow_distance_s - s_lower + s;
          cost += Config_->obstacle_weight * Config_->default_obstacle_cost * s_diff * s_diff;
        }
      }
      else if (s > s_upper)
      {

        double overtake_distance_s = Config_->overtake_distance_s; // kDpSafetyDistance
        if(boundary.obstacle_type == 8){
            overtake_distance_s = 30.0; 
        }
        if (s > s_upper + overtake_distance_s)
        { // or calculated from velocity
          continue;
        }
        else
        {
          auto s_diff = overtake_distance_s + s_upper - s;
          cost += Config_->obstacle_weight * Config_->default_obstacle_cost * s_diff * s_diff;
        }
      }
    }
  }

  return cost * unit_t_;
}

double DpStCost::GetSpatialPotentialCost(const StGraphPoint &point)
{
  return (total_s_ - point.point().s()) * Config_->spatial_potential_penalty;
}

double DpStCost::GetReferenceCost(const STPoint &point, const STPoint &reference_point) const
{
  return Config_->reference_weight * (point.s() - reference_point.s()) * (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint &first, const STPoint &second, const double speed_limit,
                              const double cruise_speed) const
{
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0)
  {
    return kInf;
  }

  const double max_adc_stop_speed = Config_->max_abs_speed_when_stopped;
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s()))
  {
    // first.s in range
    cost += Config_->keep_clear_low_speed_penalty * unit_t_ * Config_->default_speed_cost;
  }

  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0)
  {
    cost += Config_->exceed_speed_penalty * Config_->default_speed_cost * (det_speed * det_speed) * unit_t_;
  }
  else if (det_speed < 0)
  {
    cost += Config_->low_speed_penalty * Config_->default_speed_cost * -det_speed * unit_t_;
  }

  if (Config_->FLAGS_enable_dp_reference_speed)
  {
    double diff_speed = speed - cruise_speed;
    cost += Config_->reference_speed_penalty * Config_->default_speed_cost * fabs(diff_speed) * unit_t_;
  }

  return cost;
}

double DpStCost::GetAccelCost(const double accel)
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  //   DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size())
  {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0)
  {
    const double accel_sq = accel * accel;
    double max_acc = Config_->max_acceleration;
    double max_dec = Config_->max_deceleration;
    double accel_penalty = Config_->accel_penalty;
    double decel_penalty = Config_->decel_penalty;
    if (accel > 0.0)
    {
      cost = accel_penalty * accel_sq;
    }
    else
    {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty / (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty / (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  }
  else
  {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint &first, const STPoint &second, const STPoint &third)
{
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed, const STPoint &pre_point, const STPoint &curr_point)
{
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk)
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size())
  {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0)
  {
    double jerk_sq = jerk * jerk;
    if (jerk > 0)
    {
      cost = Config_->positive_jerk_coeff * jerk_sq * unit_t_;
    }
    else
    {
      cost = Config_->negative_jerk_coeff * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  }
  else
  {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint &first, const STPoint &second, const STPoint &third,
                                         const STPoint &fourth)
{
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) / (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetSchedulingCrossCost(const STPoint &pre_point,
                                  const STPoint &curr_point) {
  double cost = 0.0;

  // 调度惩罚权重（内部固定）
  const double w_early = 1.0;
  const double w_late  = 1.0;

  for (const auto *obstacles : obstacles_) {
    auto boundary = obstacles->path_st_boundary();
    // 只处理调度指令障碍物
    if (boundary.obstacle_type == 9 || boundary.obstacle_type == 10) {
      if(boundary.obstacle_type == 9){
        // 当前时间范围
        double t_curr = curr_point.t();
        double s_upper = 0.0, s_lower = 0.0;
        if (!boundary.GetBoundarySRange(t_curr, &s_upper, &s_lower)) {
          continue;  // 当前时间没定义 s 范围
        }

        double s_star = s_lower;
        double t_a = boundary.max_t_;

        // 判断是否跨越
        if (pre_point.s() <= s_star && curr_point.s() >= s_star) {
          // 线性插值计算跨越时刻
          double t_cross = pre_point.t() +
                          (s_star - pre_point.s()) /
                              (curr_point.s() - pre_point.s()) *
                              (curr_point.t() - pre_point.t());

          // 计算代价
          if (t_cross < t_a) {
            cost += w_early * (t_a - t_cross)*10000;
          } else {
            // [t_a, t_b] 内穿越 → 不加代价
          }
        }
      }else{
        // 当前时间范围
        double t_curr = curr_point.t();
        double s_upper = 0.0, s_lower = 0.0;
        if (!boundary.GetBoundarySRange(t_curr, &s_upper, &s_lower)) {
          continue;  // 当前时间没定义 s 范围
        }

        double s_star = s_lower;
        double t_b = boundary.min_t_;

        // 判断是否跨越
        if (pre_point.s() <= s_star && curr_point.s() >= s_star) {
          // 线性插值计算跨越时刻
          double t_cross = pre_point.t() +
                          (s_star - pre_point.s()) /
                              (curr_point.s() - pre_point.s()) *
                              (curr_point.t() - pre_point.t());

          // 计算代价
          if (t_cross > t_b) {
            cost += w_early * (t_cross-t_b)*10000;
          } else {
            // [t_a, t_b] 内穿越 → 不加代价
          }
        }
      }
    }
  }
  return cost;
}

double DpStCost::ComputeNearBoundaryCost(const STPoint &pre_point,
                                         const STPoint &curr_point)  
{
    double t_edge_min = std::min(pre_point.t(), curr_point.t());
    double t_edge_max = std::max(pre_point.t(), curr_point.t());

    double cost = 0.0;

    for (const auto *obstacles : obstacles_) {
        auto boundary = obstacles->path_st_boundary();
        // 只处理非调度指令障碍物
        if (boundary.obstacle_type == 9 || boundary.obstacle_type == 10) {
            continue;
        }

        double t_obs_min = boundary.min_t_;
        double t_obs_max = boundary.max_t_;
        // ✅ 时间区间无交集 → 不考虑这个障碍物
        if (t_edge_max < t_obs_min || t_edge_min > t_obs_max) {
            continue;
        }

        // === 计算 signed 最近距离 ===
        double min_signed_dist = std::numeric_limits<double>::infinity();
        const auto& vertices = boundary.GetAllVertices(); // (s,t) 顶点

        for (size_t i = 0; i < vertices.size(); ++i) {
            double s_obs = vertices[i].y();
            double t_obs = vertices[i].x();
            if (t_obs >= t_edge_min && t_obs <= t_edge_max) {
                double ratio = (t_obs - t_edge_min) / 
                              (t_edge_max - t_edge_min + 1e-6);
                double s_edge = pre_point.s() + ratio * (curr_point.s() - pre_point.s());
                double signed_dist = s_edge - s_obs;  // 🔑 保留正负号
                if (std::abs(signed_dist) < std::abs(min_signed_dist)) {
                    min_signed_dist = signed_dist;
                }
            }
        }

        // === 代价函数设计（二次型） ===
        if (min_signed_dist < 0) {  
            // 🚗 在障碍物后方
            double follow_distance_s = Config_->safe_distance; // kDpSafetyDistance
            if(boundary.obstacle_type == 8){
                follow_distance_s = 3.0; 
            }
            if (std::abs(min_signed_dist) < follow_distance_s) {
                double s_diff = follow_distance_s - std::abs(min_signed_dist);
                cost += Config_->obstacle_weight *
                        Config_->default_obstacle_cost *
                        s_diff * s_diff;
            }
        } else if (min_signed_dist > 0) {  
            // 🚗 在障碍物前方
            double overtake_distance_s = Config_->overtake_distance_s; // kDpSafetyDistance
            if(boundary.obstacle_type == 8){
                overtake_distance_s = 30.0; 
            }
            if (min_signed_dist < overtake_distance_s) {
                double s_diff = overtake_distance_s - min_signed_dist;
                cost += Config_->obstacle_weight *
                        Config_->default_obstacle_cost *
                        s_diff * s_diff;
            }
        }
    }

    return cost;
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc, const STPoint &pre_point,
                                        const STPoint &curr_point)
{
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed, const STPoint &first, const STPoint &second,
                                          const STPoint &third)
{
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}