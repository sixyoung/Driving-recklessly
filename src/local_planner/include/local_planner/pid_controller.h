#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath>  
#include <fstream>
#include <ros/ros.h>
#include <tf/tf.h>  
#include <Eigen/Dense> 
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 引入tf2库
#include <tf2/utils.h>
struct PIDDebugInfoLO {
    double error;
    double error_integral;
    double error_derivative;
    double p_term;
    double i_term;
    double d_term;
    double output;
};

struct PIDDebugInfoLA {
    double heading_error;
    double e_y;
    double cross_term;
    double stanley_term;
    double delta_raw;
    double output;
};

class PIDLongitudinalController {
public:
    PIDLongitudinalController(double K_P = 0.0, double K_I = 0, double K_D = 0)
        : K_P(K_P), K_I(K_I), K_D(K_D), error(0.0), error_integral(0.0), error_derivative(0.0) {}
    ~PIDLongitudinalController() = default;
    std::pair<double, double> run_step(double target_speed, double current_speed);
    PIDDebugInfoLO get_debug_info() const { return debug_info_lo; }
private:
    double K_P, K_I, K_D;
    double error, error_integral, error_derivative;
    std::pair<double, double> last_output = {0.0, 0.0};
    PIDDebugInfoLO debug_info_lo;
};

class PIDLateralController {
public:
    PIDLateralController(double K_P = 0.0, double K_I = 0.0, double K_D = 0.0)
        : K_P(K_P), K_I(K_I), K_D(K_D), error(0.0), error_integral(0.0), error_derivative(0.0) {}
    ~PIDLateralController() = default;
    // double run_step(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &waypoint);
    double run_step(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose, double current_speed);
    PIDDebugInfoLA get_debug_info() const { return debug_info_la; }

private:
    double K_P, K_I, K_D;
    double error, error_integral, error_derivative;
    std::pair<double, double> last_output = {0.0, 0.0};
    PIDDebugInfoLA debug_info_la;
};

class PIDController {
public:
    PIDController(double lon_KP = 0.0, double lon_KI = 0.0, double lon_KD = 0.0, double lat_KP = 0.0, double lat_KI = 0.0, double lat_KD = 0.0){
            // 初始化控制器
            lon_controller = PIDLongitudinalController(lon_KP, lon_KI, lon_KD);
            lat_controller = PIDLateralController(lat_KP, lat_KI, lat_KD);
    }
        
    ~PIDController() = default;
    carla_msgs::CarlaEgoVehicleControl run_step(double target_speed, double current_speed, const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &waypoint);
    std::pair<PIDDebugInfoLO, PIDDebugInfoLA> get_debug_info() const {
        return {lon_controller.get_debug_info(), lat_controller.get_debug_info()};
    }

protected:
    PIDLongitudinalController lon_controller;
    PIDLateralController lat_controller;
};

#endif // PID_CONTROLLER_H
