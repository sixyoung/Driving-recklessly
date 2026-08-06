/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "publishable_trajectory.h"
#include "reference_line.h"
#include "vehicle_state_provider.h"

namespace planner {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher(const Param_Configs& cfg ) : Config_(&cfg) {};

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
      const VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      std::string* replan_reason,
      const Param_Configs* Config);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const TrajectoryPoint& matched_trajectory_point);

  static TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time, const VehicleState& vehicle_state);
      
  static std::vector<TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const VehicleState& vehicle_state);
private:
  const Param_Configs* Config_;
};

}  // namespace planning