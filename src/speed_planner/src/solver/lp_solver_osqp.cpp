#include "solver/lp_solver_osqp.h"

namespace osqp
{
    bool LPSolver::solveSoft(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N]
         *      | pdelta[0], pdelta[1], ..., pdelta[N]
         *      | mdelta[0], mdelta[1], ..., mdelta[N]
         *      | psigma[0], psigma[1], ..., psigma[N]
         *      | msigma[0], msigma[1], ..., msigma[N]
         *      | pgamma[0], pgamma[1], ..., pgamma[N]
         *      | mgamma[0], mgamma[1], ..., mgamma[N]
         *      delta[i] = pdelta[i] - mdelta[i]
         *      sigma[i] = psigma[i] - msigma[i]
         *      gamma[i] = pgamma[i] - mgamma[i]
         *      |delta[i]| = pdelta[i] + mdelta[i]
         *      |sigma[i]| = psigma[i] + msigma[i]
         *      |gamma[i]| = pgamma[i] + mgamma[i]
         * b[i]: velocity^2
         * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
         * sigma: amin < a[i] - sigma[i] < amax
         * gamma: jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
         */

        // Create Solver
        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 8*N;
        const long constraint_size = 10*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;
        const double over_j_weight = param_.over_j_weight;
        const double over_v_weight = param_.over_v_weight;
        const double over_a_weight = param_.over_a_weight;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            const double v_max = std::max(max_vels.at(i), 0.1);
            gradient[i] = -1.0/(v_max*v_max);
        }
        for(long i=2*N; i<4*N; ++i)
            gradient[i] = over_v_weight;
        for(long i=4*N; i<6*N; ++i)
            gradient[i] = over_a_weight;
        for(long i=6*N; i<8*N; ++i)
            gradient[i] = over_j_weight;

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double>lowerBound(constraint_size, 0.0);
        std::vector<double>upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 0. Soft Constraint Variable
        for(long i=0; i<6*N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+2*N) = 1.0;
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 1e100;
        }

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            constraint_matrix(constraint_num, i+2*N) = -1.0; // -pdelta[i]
            constraint_matrix(constraint_num, i+3*N) =  1.0; // mdelta[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            constraint_matrix(constraint_num, i+4*N) = -1.0; // -psigma[i]
            constraint_matrix(constraint_num, i+5*N) =  1.0; // msigma[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Jerk Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N)   = -ref_vels[i]; // -a[i] * ref_vels[i]
            constraint_matrix(constraint_num, i+1+N) =  ref_vels[i]; // a[i+1] * ref_vels[i]
            constraint_matrix(constraint_num, i+6*N) = -ds; // -pgamma[i]
            constraint_matrix(constraint_num, i+7*N) =  ds; // mgamma[i]
            lowerBound[constraint_num] = jmin*ds;
            upperBound[constraint_num] = jmax*ds;
        }

        //4. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

        const std::vector<double> optval = std::get<0>(result);

        const auto& status = std::get<3>(result);  // 状态码（1 表示成功）
        const auto& msg    = std::get<4>(result);  // 状态信息
        const double throttle_time = 2.0;  // 控制打印频率为每 2 秒一次
        // // 判断并打印失败信息
        // if (status != 1) {
        //     ROS_ERROR("[LPSolverSoft] ❌ QP solve failed!");
        //     ROS_ERROR("[LPSolverSoft] Status Code : %d", status);
        //     ROS_ERROR("[LPSolverSoft] Status Msg  : %s", msg.c_str());

        //     // 打印传入参数（这些将每次都打印）
        //     ROS_INFO("[LPSolverSoft] initial_vel: %f", initial_vel);
        //     ROS_INFO("[LPSolverSoft] initial_acc: %f", initial_acc);
        //     ROS_INFO("[LPSolverSoft] ds: %f", ds);
        //     ROS_INFO("[LPSolverSoft] N: %zu", N);
        //     ROS_INFO("[LPSolverSoft] var_size: %zu", var_size);
        //     ROS_INFO("[LPSolverSoft] constraint_size: %zu", constraint_size);
        //     ROS_INFO("[LPSolverSoft] amax: %f", amax);
        //     ROS_INFO("[LPSolverSoft] amin: %f", amin);
        //     ROS_INFO("[LPSolverSoft] jmax: %f", jmax);
        //     ROS_INFO("[LPSolverSoft] jmin: %f", jmin);

        //     // 打印 max_vels
        //     std::ostringstream oss;
        //     oss << "[LPSolverSoft] max_vels (" << max_vels.size() << "): [";
        //     for (size_t i = 0; i < max_vels.size(); ++i) {
        //         oss << max_vels[i];
        //         if (i != max_vels.size() - 1) {
        //             oss << ", ";
        //         }
        //     }
        //     oss << "]";

        //     ROS_INFO("%s", oss.str().c_str());
            
        // output.resize(N);
        // for(unsigned int i=0; i<N; ++i)
        // {
        //     output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
        //     output.acceleration[i] = optval[i+N];
        // }

        // for(unsigned int i=0; i<N-1; ++i)
        // {
        //     double a_current = optval[i+N];
        //     double a_next    = optval[i+N+1];
        //     output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        // }
        // output.jerk[N-1] = output.jerk[N-2];

        //     return false;
        // } else {
        //     // 成功时仅打印一次
        //     ROS_INFO_THROTTLE(0.0, "[LPSolverSoft] ✅ QP solve success!");
        //     ROS_INFO_THROTTLE(0.0, "[LPSolverSoft] initial_vel: %f", initial_vel);
        //     ROS_INFO_THROTTLE(0.0, "[LPSolverSoft] initial_acc: %f", initial_acc);
        //     double v_violation_sum = 0.0, a_violation_sum = 0.0, j_violation_sum = 0.0;
        //     for (size_t i = 0; i < N; ++i) {
        //         v_violation_sum += optval[2*N + i] + optval[3*N + i];
        //         a_violation_sum += optval[4*N + i] + optval[5*N + i];
        //         j_violation_sum += optval[6*N + i] + optval[7*N + i];
        //     }
        //     ROS_INFO("[ViolationSum] velocity: %.4f, acc: %.4f, jerk: %.4f", v_violation_sum, a_violation_sum, j_violation_sum);
        // }

        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];

        return true;
    }

    bool LPSolver::solveHard(const double& initial_vel,
                             const double& initial_acc,
                             const double& ds,
                             const std::vector<double>& ref_vels,
                             const std::vector<double>& max_vels,
                             OutputInfo& output)
    {
        /*
         * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N]]
         * b[i]: velocity^2
         * 0 < b[i] < max_vel[i]*max_vel[i]
         * amin < a[i] < amax
         * jerk_min/ref_vel[i] < pseudo_jerk[i] < jerk_max/ref_vel[i]
         */

        osqp::OSQPInterface qp_solver_;
        qp_solver_.updateMaxIter(20000);
        qp_solver_.updateRhoInterval(0);  // 0 means automatic
        qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateEpsAbs(1.0e-4);  // def: 1.0e-4
        qp_solver_.updateVerbose(false);

        long N = static_cast<long>(ref_vels.size());
        const long var_size = 2*N;
        const long constraint_size = 4*N;
        const double amax = param_.max_accel;
        const double amin = param_.min_decel;
        const double jmax = param_.max_jerk;
        const double jmin = param_.min_jerk;

        // Hessian
        Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);

        // Gradient
        std::vector<double> gradient(var_size, 0.0);
        for(long i=0; i<N; ++i)
        {
            const double v_max = std::max(max_vels.at(i), 0.1);
            gradient[i] = -1.0/(v_max*v_max);
        }

        // Constraint Matrix
        Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
        std::vector<double> lowerBound(constraint_size, 0.0);
        std::vector<double> upperBound(constraint_size, 0.0);
        long constraint_num = 0;

        // 1. Velocity Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i) = 1.0; //b[i]
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = max_vels[i] * max_vels[i];
        }

        // 2. Acceleration Constraint
        for(long i=0; i<N; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
            lowerBound[constraint_num] = amin;
            upperBound[constraint_num] = amax;
        }

        //3. Jerk Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+N)   = -ref_vels[i]; // -a[i] * ref_vels[i]
            constraint_matrix(constraint_num, i+1+N) =  ref_vels[i]; // a[i+1] * ref_vels[i]
            lowerBound[constraint_num] = jmin*ds;
            upperBound[constraint_num] = jmax*ds;
        }

        //4. Dynamic Constraint
        for(long i=0; i<N-1; ++i, ++constraint_num)
        {
            constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
            constraint_matrix(constraint_num, i) = -1.0; // -b[i]
            constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
            lowerBound[constraint_num] = 0.0;
            upperBound[constraint_num] = 0.0;
        }

        //5. Initial Condition
        constraint_matrix(constraint_num, 0) = 1.0;
        lowerBound[constraint_num] = initial_vel*initial_vel;
        upperBound[constraint_num] = initial_vel*initial_vel;
        ++constraint_num;
        constraint_matrix(constraint_num, N) = 1.0;
        lowerBound[constraint_num] = initial_acc;
        upperBound[constraint_num] = initial_acc;
        ++constraint_num;
        assert(constraint_num == constraint_size);

        // solve the QP problem
        const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);
        const std::vector<double> optval = std::get<0>(result);

        const auto& status = std::get<3>(result);  // 状态码（1 表示成功）
        const auto& msg    = std::get<4>(result);  // 状态信息
        const double throttle_time = 2.0;  // 控制打印频率为每 2 秒一次
        // 判断并打印失败信息
        // if (status != 1) {
        //     // 使用 ROS_ERROR 打印错误信息，每次都会打印
        //     ROS_ERROR("[LPSolverHard] ❌❌ QP solve failed!");
        //     ROS_ERROR("[LPSolverHard] Status Code : %d", status);
        //     ROS_ERROR("[LPSolverHard] Status Msg  : %s", msg.c_str());

        //     // 打印传入参数（每次都会打印）
        //     ROS_INFO("[LPSolverHard] initial_vel: %f", initial_vel);
        //     ROS_INFO("[LPSolverHard] initial_acc: %f", initial_acc);
        //     ROS_INFO("[LPSolverHard] ds: %f", ds);
        //     ROS_INFO("[LPSolverHard] N: %zu", N);
        //     ROS_INFO("[LPSolverHard] var_size: %zu", var_size);
        //     ROS_INFO("[LPSolverHard] constraint_size: %zu", constraint_size);
        //     ROS_INFO("[LPSolverHard] amax: %f", amax);
        //     ROS_INFO("[LPSolverHard] amin: %f", amin);
        //     ROS_INFO("[LPSolverHard] jmax: %f", jmax);
        //     ROS_INFO("[LPSolverHard] jmin: %f", jmin);

        //     // 打印 max_vels（每次都会打印）
        //     ROS_INFO("[LPSolverHard] max_vels (%zu): [", max_vels.size());
        //     for (size_t i = 0; i < max_vels.size(); ++i) {
        //         ROS_INFO("%f", max_vels[i]);
        //         if (i != max_vels.size() - 1) {
        //             ROS_INFO(", ");
        //         }
        //     }
        //     ROS_INFO("]");
        //     return false;
        // } else {
        //     // 成功时仅打印一次
        //     ROS_INFO_THROTTLE(throttle_time, "[LPSolverHard] ✅✅ QP solve success!");
        // }
        output.resize(N);
        for(unsigned int i=0; i<N; ++i)
        {
            output.velocity[i] = std::sqrt(std::max(optval[i], 0.0));
            output.acceleration[i] = optval[i+N];
        }

        for(unsigned int i=0; i<N-1; ++i)
        {
            double a_current = optval[i+N];
            double a_next    = optval[i+N+1];
            output.jerk[i] = (a_next - a_current) * output.velocity[i] / ds;
        }
        output.jerk[N-1] = output.jerk[N-2];
        return true;
    }

    bool LPSolver::solveSoftPseudo(const double &initial_vel,
                                   const double &initial_acc,
                                   const double &ds,
                                   const std::vector<double> &ref_vels,
                                   const std::vector<double> &max_vels,
                                   OutputInfo &output)
    {
        std::cerr << "[Solver Error]: LP Solver cannot be applied to the pseudo-jerk problem" << std::endl;
        return false;
    }

    bool LPSolver::solveHardPseudo(const double &initial_vel,
                                   const double &initial_acc,
                                   const double &ds,
                                   const std::vector<double> &ref_vels,
                                   const std::vector<double> &max_vels,
                                   OutputInfo &output)

    {
        return solveSoftPseudo(initial_vel, initial_acc, ds, ref_vels, max_vels, output);
    }
}
