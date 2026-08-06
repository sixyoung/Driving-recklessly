#ifndef JERK_SPEED_PLANNING_JERK_SPEED_PLANNER_H
#define JERK_SPEED_PLANNING_JERK_SPEED_PLANNER_H

#include "data_pool.h"
// #include "scenario_generator.h"
#include "max_velocity_filter.h"
#include "optimizer.h"
#include "utils.h"
namespace speed_planner{
class JerkSpeedPlanner {
public:
    explicit JerkSpeedPlanner(DataPool& data_pool);
    
    std::tuple<bool, BaseSolver::OutputInfo> plan(bool should_output_results);
    void setOptimizerParams(const BaseSolver::OptimizerParam& param);
    void setConstraints(bool is_hard, double margin_s1, double margin_s2);
    
    // 删除setScenario函数，因为不再需要

private:
    bool velocityPlanning();
    bool trajectoryOptimization();
    void outputResults();

    DataPool& data_pool_;
    double low_vel_threshold_;
    double margin_s1_;
    double margin_s2_;
    BaseSolver::OptimizerParam param_;
    bool is_hard_;
    // 删除scenario_num_成员
};
}
#endif //JERK_SPEED_PLANNING_JERK_SPEED_PLANNER_H
