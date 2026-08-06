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
 * @file publishable_trajectory.h
 **/

#pragma once
#include "trajectoryPoint.h"

namespace planner {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  double header_time() const;
  // ✅ 新增：设置 header_time
  void set_header_time(const double t) { header_time_ = t; }
 private:
  double header_time_ = 0.0;
};

}  // namespace planning
