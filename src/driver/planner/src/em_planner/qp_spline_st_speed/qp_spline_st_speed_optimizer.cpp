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
#include "qp_spline_st_speed_optimizer.h"
#include <utility>
#include <vector>
#include "qp_piecewise_st_graph.h"
#include "qp_spline_st_graph.h"
#include "st_graph_data.h"

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer(const Param_Configs &cfg): Config_(&cfg)
{
    std::vector<double> init_knots;

    if (Config_->FLAGS_use_osqp_optimizer_for_qp_st)
    {
        spline_solver_.reset(new OsqpSpline1dSolver(init_knots, 5));
    }
    else
    {
        spline_solver_.reset(new ActiveSetSpline1dSolver(init_knots, 5, *Config_));
    }
}

bool QpSplineStSpeedOptimizer::Process(const SL_Boundary &adc_sl_boundary,
                                       const StGraphData &st_graph_data,
                                       const PathData &path_data,
                                       const TrajectoryPoint &init_point,
                                       ReferenceLineInfo &reference_line_info,
                                       SpeedData *const speed_data)
{

    if (path_data.discretized_path().empty())
    {
        std::cout << "Empty path data";
        return false;
    }

    QpSplineStGraph st_graph(spline_solver_.get(), Config_->IsChangeLanePath, reference_line_info, *Config_);

    std::pair<double, double> accel_bound = {
        Config_->preferred_min_deceleration,
        Config_->preferred_max_acceleration};

    auto ret = st_graph.Search(st_graph_data, accel_bound, speed_data);
    if (ret == false)
    {
        ROS_ERROR("[speed QP][%s] Failed to solve with ideal acceleration conditions！ Use secondary choice instead.",
                Config_->role_name_.c_str());

        accel_bound.first = Config_->max_deceleration;
        accel_bound.second = Config_->max_acceleration;
        ret = st_graph.Search(st_graph_data, accel_bound, speed_data);

        // backup plan: use piecewise_st_graph
        if (ret == false)
        {

            ROS_ERROR("[speed QP][%s] Spline QP speed solver Failed！ Using finite difference method.",Config_->role_name_.c_str());
            QpPiecewiseStGraph piecewise_st_graph(*Config_);
            ret = piecewise_st_graph.Search(st_graph_data, speed_data, accel_bound);

            if (ret == false)
            {
                ROS_ERROR("[speed QP][%s] Failed to search graph with quadratic programming!",Config_->role_name_.c_str());
                return false;
            }
        }
    }

    return true;
}