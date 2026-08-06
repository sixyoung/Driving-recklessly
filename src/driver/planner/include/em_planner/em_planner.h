#pragma once
#include "quintic_polynomial_curve1d.h"
#include "dp_poly_path_optimizer.h"
#include "path_time_heuristic_optimizer.h"
#include "qp_spline_path_optimizer.h"
#include "qp_spline_st_speed_optimizer.h"
#include "speed_limit_decider.h"
#include "path_time_graph.h"
#include "speed_decider.h"
#include "trajectory_stitcher.h"
#include "Configs.h"
#include "file_exporter.h"

class EMPlanner
{
public:
  EMPlanner() = default;
  EMPlanner(const Param_Configs& cfg);

  ~EMPlanner() = default;

  DiscretizedTrajectory Plan(const TrajectoryPoint &planning_init_point, const ReferenceLine &reference_line,
                                      ReferenceLineInfo &reference_line_info,
                                      std::vector<Obstacle> AllObstacle,                       
                                      const nav_msgs::Odometry &vehicle_odom,
                                      const double &vehicle_heading, const double &lon_decision_horizon, 
                                      const std::array<double, 3> &init_s,
                                      const std::array<double, 3> &init_d);
                             
  void CopyFrom(TrajectoryPoint &traj, const InitialConditions &Initial);
  void ComputeInitFrenetState(const ReferencePoint &matched_point,
                              const TrajectoryPoint &cartesian_state,
                              std::array<double, 3> *ptr_s,
                              std::array<double, 3> *ptr_d);
private:
  bool CombinePathAndSpeedProfile(const double relative_time, const double start_s,
                                  DiscretizedTrajectory *ptr_discretized_trajectory, PathData &path_data, SpeedData &speed_data);

  bool PlanOnReferenceLine(const TrajectoryPoint &planning_start_point, const std::vector<const Obstacle *> &obstacles,
                           ReferenceLineInfo *reference_line_info, const SL_Boundary &adc_sl_boundary, PathData &path_data, SpeedData &speed_data);

  void GenerateFallbackPathProfile(const TrajectoryPoint &planning_init_point, const SL_Boundary &adc_sl_boundary,
                                   ReferenceLineInfo *reference_line_info, PathData &path_data);

  void GenerateFallbackSpeedProfile(const TrajectoryPoint &planning_init_point, SpeedData &speed_data);

  SpeedData GenerateStopProfile(const double init_speed, const double init_acc) const;

  SpeedData GenerateStopProfileFromPolynomial(const double init_speed, const double init_acc) const;

  bool IsValidProfile(const QuinticPolynomialCurve1d &curve) const;

  PathPoint MakePathPoint(const double x, const double y, const double z, const double theta, const double kappa,
                          const double dkappa, const double ddkappa);

  geometry_msgs::PoseArray traj_points_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  const Param_Configs* Config_;
};