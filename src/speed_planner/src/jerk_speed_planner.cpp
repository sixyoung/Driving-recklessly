#include "jeker_speed_planner.h"
#include <chrono>
#include <iostream>
namespace speed_planner{
JerkSpeedPlanner::JerkSpeedPlanner(DataPool& data_pool) 
    : data_pool_(data_pool),
      low_vel_threshold_(5*1e-1),
      margin_s1_(6.0),
      margin_s2_(2.0){
      // 优化器参数
      param_.max_accel = 3.0;
      param_.min_decel = -5.0;
      param_.max_jerk = 2.0;
      param_.min_jerk = -2.0;
      param_.over_j_weight = 200;
      param_.over_a_weight = 100;
      param_.over_v_weight = 120;
      param_.smooth_weight = 20.0;
      is_hard_ = false;  // 默认使用硬约束
}

void JerkSpeedPlanner::setOptimizerParams(const BaseSolver::OptimizerParam& param) {
    param_ = param;
}

void JerkSpeedPlanner::setConstraints(bool is_hard, double margin_s1, double margin_s2) {
    is_hard_ = is_hard;
    margin_s1_ = margin_s1;
    margin_s2_ = margin_s2;
}


std::tuple<bool, BaseSolver::OutputInfo> JerkSpeedPlanner::plan(bool should_output_results) {
    if (!data_pool_.validate()) {
        std::cerr << "Invalid input data!" << std::endl;
        return std::make_pair(false, data_pool_.lp_output);
    }
    param_.max_accel = data_pool_.max_acc_;
    param_.min_decel = data_pool_.min_acc_;
    param_.max_jerk = data_pool_.max_jerk_;
    param_.min_jerk = data_pool_.min_jerk_;

    if (!velocityPlanning()) {
        return std::make_pair(false, data_pool_.lp_output);
    }

    if (!trajectoryOptimization()) {
        if(should_output_results) {
            outputResults(); 
        }
        return std::make_pair(false, data_pool_.lp_output); 
    }

    if (should_output_results) {
        outputResults(); 
    }
    return std::make_pair(true, data_pool_.lp_output); 
}

bool JerkSpeedPlanner::velocityPlanning() {
    // 1. 最大速度限制过滤器初始化
    MaximumVelocityFilter vel_filter(low_vel_threshold_);

    // 2. 基于道路限制的最大速度修正
    vel_filter.modifyMaximumVelocity(data_pool_.positions_, data_pool_.max_velocities_, data_pool_.modified_data);

    // 3. 初始化 obs_filtered_data 为 modified_data
    data_pool_.obs_filtered_data = data_pool_.modified_data;
    if (!data_pool_.obs_.empty()) {
        for (const auto& obs : data_pool_.obs_) {
            // 1. 分类
            Category obs_category = vel_filter.categorizeObstacles(
                data_pool_.obs_filtered_data.time,
                data_pool_.obs_filtered_data.position,
                obs
            );
            Trajectory next_traj;  // 每轮新建，不累加
            // 2. 按类型计算限制
            if (obs_category == Category::ZeroVelObstacle) {
                // std::cout << "Obstacle with zero velocity" << std::endl;
                vel_filter.calcZeroVelObsVelocity(
                    data_pool_.obs_filtered_data.time, data_pool_.obs_filtered_data.position, data_pool_.v0_, data_pool_.obs_filtered_data.velocity, obs, margin_s1_,
                    margin_s2_, next_traj.time, next_traj.position, next_traj.velocity);
            } else if (obs_category == Category::PosInterceptionObstacle) {
                // std::cout << "Obstacle with positive interception" << std::endl;
                vel_filter.calcPosInterceptObsVelocity(
                    data_pool_.obs_filtered_data.time, data_pool_.obs_filtered_data.position, data_pool_.v0_, data_pool_.obs_filtered_data.velocity, obs, margin_s1_,
                    margin_s2_, next_traj.time, next_traj.position, next_traj.velocity);
            } else if (obs_category == Category::NegRightInterceptionObstacle) {
                // std::cout << "Obstacle with right negative interception" << std::endl;
                vel_filter.calcNegRightInterceptObsVelocity(
                    data_pool_.obs_filtered_data.time, data_pool_.obs_filtered_data.position, data_pool_.v0_, data_pool_.obs_filtered_data.velocity, obs, margin_s1_,
                    margin_s2_, next_traj.time, next_traj.position, next_traj.velocity);
            } else if (obs_category == Category::NegLeftInterceptionObstacle) {
                // std::cout << "Obstacle with left negative interception" << std::endl;
                vel_filter.calcNegLeftInterceptObsVelocity(
                    data_pool_.obs_filtered_data.time, data_pool_.obs_filtered_data.position, data_pool_.v0_, data_pool_.obs_filtered_data.velocity, obs, margin_s1_,
                    margin_s2_, next_traj.time, next_traj.position, next_traj.velocity);
            } else {
                // std::cout << "Safe Obstacle" << std::endl;
                continue;
            }


            // 一致性检查
            if (data_pool_.positions_.size() != next_traj.velocity.size()) {
                std::string category_str;
                switch (obs_category) {
                    case Category::ZeroVelObstacle:
                        category_str = "ZeroVelObstacle"; break;
                    case Category::PosInterceptionObstacle:
                        category_str = "PosInterceptionObstacle"; break;
                    case Category::NegRightInterceptionObstacle:
                        category_str = "NegRightInterceptionObstacle"; break;
                    case Category::NegLeftInterceptionObstacle:
                        category_str = "NegLeftInterceptionObstacle"; break;
                    default:
                        category_str = "UnknownCategory"; break;
                }

                std::cerr << "        [ERROR] Obstacle category: " << category_str
                        << " caused trajectory length mismatch. position.size() = "
                        << data_pool_.positions_.size()
                        << ", velocity.size() = "
                        << data_pool_.obs_filtered_data.velocity.size()
                        << std::endl;
                std::cerr << "        Obstacle Time (t_)  : [" << obs.t_.first << ", " << obs.t_.second << "]\n"
                        << "        Obstacle S-Range (s_): [" << obs.s_.first << ", " << obs.s_.second << "]\n"
                        << "        Obstacle Velocity    : " << obs.vel_ << " m/s\n";

                std::cerr << "        obs_filtered_data.velocity (" 
                        << data_pool_.obs_filtered_data.velocity.size() << "): [";
                for (size_t i = 0; i < data_pool_.obs_filtered_data.velocity.size(); ++i) {
                    std::cerr << data_pool_.obs_filtered_data.velocity[i];
                    if (i != data_pool_.obs_filtered_data.velocity.size() - 1) {
                        std::cerr << ", ";
                    }
                }
                std::cerr << "]" << std::endl;

                // 打印 next_traj.velocity 内容
                std::cerr << "        next_traj.velocity (" 
                        << next_traj.velocity.size() << "): [";
                for (size_t i = 0; i < next_traj.velocity.size(); ++i) {
                    std::cerr << next_traj.velocity[i];
                    if (i != next_traj.velocity.size() - 1) {
                        std::cerr << ", ";
                    }
                }
                std::cerr << "]" << std::endl;
            }

            data_pool_.obs_filtered_data.time     = std::move(next_traj.time);
            data_pool_.obs_filtered_data.position = std::move(next_traj.position);
            data_pool_.obs_filtered_data.velocity = std::move(next_traj.velocity);
        }
    }else{
        // std::cout << "[JerkSpeedPlanner] No obstacles detected." << std::endl;
    }

    // 5. 基于加加速度的速度平滑
    vel_filter.smoothVelocity(
        data_pool_.ds_, data_pool_.v0_, data_pool_.a0_,
        data_pool_.max_acc_, data_pool_.max_jerk_,
        data_pool_.min_acc_, data_pool_.min_jerk_,
        data_pool_.obs_filtered_data.velocity,
        data_pool_.jerk_filtered_vels,
        data_pool_.jerk_filtered_accs
    );
    return true;
}

bool JerkSpeedPlanner::trajectoryOptimization() {

    // 2. 线性规划(LP)优化
    Optimizer lp_optimizer(Optimizer::OptimizerSolver::OSQP_LP, param_);
    data_pool_.lp_output.position = data_pool_.positions_;

    bool lp_result = lp_optimizer.solve(
        is_hard_, data_pool_.v0_, data_pool_.a0_, data_pool_.ds_,
        data_pool_.jerk_filtered_vels, data_pool_.jerk_filtered_vels,
        data_pool_.lp_output
    );

    if (!lp_result) {
        std::cerr << "LP Solver has Error" << std::endl;
        return false;
    }
    return true;
}

void JerkSpeedPlanner::outputResults() {
    if (!data_pool_.obs_.empty()) {
        // std::string obs_filename = data_pool_.current_dir + "/result/obs.csv";
        std::string obs_filename = data_pool_.current_dir + "/home/bob/文档/备份/demo05/src/speed_planner/result/obs.csv";
        for (size_t i = 0; i < data_pool_.obs_.size(); ++i) {
            std::string obs_filename = data_pool_.current_dir + "/home/bob/文档/备份/demo05/src/speed_planner/result/obs_" + std::to_string(i) + ".csv";
            Utils::outputObsToFile(obs_filename, data_pool_.obs_[i]);
        }
    }

    std::string filename = data_pool_.current_dir + "/home/bob/文档/备份/demo05/src/speed_planner/result/optimization_result.csv";
    // std::string filename = data_pool_.current_dir + "/result/optimization_result.csv";
    Utils::outputToFile(
        filename,
        data_pool_.positions_,
        data_pool_.modified_data,
        data_pool_.obs_filtered_data,
        data_pool_.jerk_filtered_vels,
        data_pool_.lp_output,
        data_pool_.qp_output
    );
}
}