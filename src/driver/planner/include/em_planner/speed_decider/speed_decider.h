#pragma once

#include <vector>
#include <string>
#include "Obstacle.h"
#include "ObjectDecision.h"
#include "speed_data.h"
#include "boundarys.h"
#include "reference_line.h"

// 🚗 SpeedDecider: 负责给障碍物打上决策 (STOP/FOLLOW/YIELD/OVERTAKE/IGNORE)
class SpeedDecider {
public:
  explicit SpeedDecider() = default;
  SpeedDecider(const TrajectoryPoint &planning_init_point,
                const SL_Boundary& adc_sl_boundary,
                const ReferenceLine* reference_line, const std::string &role_name,
                const Param_Configs &cfg);
  // 主入口：根据 DP 粗解速度曲线和障碍物，生成决策
  bool Execute(const SpeedData& speed_data,
                           std::vector<Obstacle*>& obstacles);

private:
  // 自车与障碍物在 ST 图的关系
  enum class StPosition { ABOVE = 1, BELOW = 2, CROSS = 3 };

  bool MakeObjectDecision(
    const SpeedData& speed_profile,
    std::vector<Obstacle*>& obstacles) const;

  // === 工具函数 ===
  StPosition GetStPosition(const SpeedData& speed_profile,
                           const ST_Boundary& st_boundary) const;

  bool CheckKeepClearCrossable(const SpeedData& speed_profile,
                               const ST_Boundary& keep_clear_st_boundary) const;

  bool CheckKeepClearBlocked(const std::vector<Obstacle*>& obstacles,
                           const Obstacle& keep_clear_obstacle) const;

  bool CheckIsFollowByT(const ST_Boundary& boundary) const;

  bool IsFollowTooClose(const Obstacle& obstacle) const;

  // === 决策生成 ===
  bool CreateStopDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* stop_decision,
    double stop_distance,
    double main_stop_s) const;

  bool CreateFollowDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* follow_decision,
    double main_stop_s
  ) const;

  bool CreateYieldDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* yield_decision,
    double main_stop_s,
    double yield_distance_vehicle = 8.0,    // 车辆默认让行距离
    double yield_distance_ped_bike = 12.0   // 行人/自行车更大让行距离
  ) const;

  bool CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* overtake_decision,
    double main_stop_s
  ) const;

  void AppendIgnoreDecision(Obstacle& obstacle) const;
private:
  TrajectoryPoint init_point_;
  SL_Boundary adc_sl_boundary_;
  const ReferenceLine* reference_line_;
  std::string role_name_;
private:
  const Param_Configs* Config_;
};
