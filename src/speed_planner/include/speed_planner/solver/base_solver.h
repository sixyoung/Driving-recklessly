#ifndef FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H
#define FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H

#include <Eigen/Eigen> // 用于线性代数、矩阵和向量运算、数值解算
#include <vector>  // C++标准模板库（STL）中的vector容器
#include <cmath> // sin、cos、sqrt等，用于执行基本的数学运算
#include <cassert> // 断言宏
#include <iostream> // 输入输出流
#include <chrono> // 用于计时
// #include "nlopt.hpp"
#include "../interpolate.h" // 用于插值
// #include "gurobi_c++.h"
#include "osqp_interface/osqp_interface.h" // 用于求解QP问题
#include <ros/ros.h>


class BaseSolver
{
public:
    struct OptimizerParam
    {
        double max_accel;
        double min_decel;
        double max_jerk;
        double min_jerk;
        double smooth_weight;
        double over_j_weight;
        double over_v_weight;
        double over_a_weight;
    };

    struct OutputInfo
    {
        std::vector<double> time;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> jerk;

        void reserve(const unsigned int& N)
        {
            time.reserve(N);
            velocity.reserve(N);
            acceleration.reserve(N);
            jerk.reserve(N);
        }

        void resize(const unsigned int& N)
        {
            time.resize(N);
            velocity.resize(N);
            acceleration.resize(N);
            jerk.resize(N);
        }
    };

    BaseSolver(const OptimizerParam& param) : param_(param) {};

    void setParam(const OptimizerParam& param)
    {
        param_ = param;
    }

    virtual bool solveSoft(const double& initial_vel,
                           const double& initial_acc,
                           const double& ds,
                           const std::vector<double>& ref_vels,
                           const std::vector<double>& max_vels,
                           OutputInfo& output) = 0;

    virtual bool solveHard(const double& initial_vel,
                           const double& initial_acc,
                           const double& ds,
                           const std::vector<double>& ref_vels,
                           const std::vector<double>& max_vels,
                           OutputInfo& output) = 0;

    virtual bool solveSoftPseudo(const double& initial_vel,
                                 const double& initial_acc,
                                 const double& ds,
                                 const std::vector<double>& ref_vels,
                                 const std::vector<double>& max_vels,
                                 OutputInfo& output) = 0;

    virtual bool solveHardPseudo(const double& initial_vel,
                                 const double& initial_acc,
                                 const double& ds,
                                 const std::vector<double>& ref_vels,
                                 const std::vector<double>& max_vels,
                                 OutputInfo& output) = 0;

protected:
    OptimizerParam param_;
};

#endif //FILTER_POSITION_OPTIMIZATION_BASE_SOLVER_H
