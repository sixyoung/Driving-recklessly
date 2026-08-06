#ifndef JERK_SPEED_PLANNING_DATA_POOL_H
#define JERK_SPEED_PLANNING_DATA_POOL_H

#include <vector>
#include <string>
#include <iostream>
#include "max_velocity_filter.h"
#include "optimizer.h"
#include "obstacle.h"
namespace speed_planner{
struct DataPool {
    // 场景数据
    std::vector<double> positions_;
    std::vector<double> max_velocities_;
    std::vector<Obstacle> obs_;
    double ds_{0.1};
    double v0_{0.0};
    double a0_{0.0};
    double max_acc_{1.0};
    double max_jerk_{0.8};
    double min_acc_{-1.0};
    double min_jerk_{-0.8};

    // 中间数据
    MaximumVelocityFilter::OutputInfo modified_data;
    MaximumVelocityFilter::OutputInfo obs_filtered_data;
    std::vector<double> jerk_filtered_vels;
    std::vector<double> jerk_filtered_accs;

    // 优化结果
    BaseSolver::OutputInfo lp_output;
    BaseSolver::OutputInfo qp_output;
    // BaseSolver::OutputInfo nc_output;  

    // 其他
    std::string current_dir;

    // 数据验证函数
    bool validate() const {
        if (positions_.empty() || max_velocities_.empty()) {
            std::cerr << "Empty trajectory data!" << std::endl;
            return false;
        }
        if (positions_.size() != max_velocities_.size()) {
            std::cerr << "Inconsistent data size!" << std::endl;
            return false;
        }
        // Add more validation as needed
        return true;
    }

    // 重置函数
    void reset() {
        positions_.clear();
        max_velocities_.clear();
        obs_.clear();
        modified_data = MaximumVelocityFilter::OutputInfo();
        obs_filtered_data = MaximumVelocityFilter::OutputInfo();
        jerk_filtered_vels.clear();
        jerk_filtered_accs.clear();
        lp_output = BaseSolver::OutputInfo();
        qp_output = BaseSolver::OutputInfo();
    }
};
}
#endif //JERK_SPEED_PLANNING_DATA_POOL_H