#include "qp_spline_path_optimizer.h"

QpSplinePathOptimizer::QpSplinePathOptimizer(const Param_Configs& cfg) : Config_(&cfg)
{
  spline_solver_.reset(new ActiveSetSpline1dSolver(init_knots_, Config_->spline_order, *Config_)); // 样条曲线次数
}

bool QpSplinePathOptimizer::Process(const SpeedData &speed_data, const ReferenceLine &reference_line,
                                    const TrajectoryPoint &init_point, PathData *const path_data,
                                    const std::vector<const Obstacle *> &obstacles)
{
  QpSplinePathGenerator path_generator(spline_solver_.get(), reference_line, *Config_);
  if(Config_->IsChangeLanePath){
    path_generator.SetChangeLane(true);
  }else
  {
    path_generator.SetChangeLane(false);
  }
  
  // 第一次调用Generate()时，boundary_extension=0.0
  double boundary_extension = 0.0;
  bool is_final_attempt = false;

  bool ret = path_generator.Generate(obstacles, speed_data, init_point, boundary_extension, is_final_attempt, path_data);

  if (!ret)
  {
    // std::cout << "failed to generate spline path with boundary_extension = 0.0";
    boundary_extension = 1.2;
    is_final_attempt = true;
    // 若生成轨迹失败，则将boundary_extension增大为cross_lane_lateral_extension()（默认值1.2m)
    // 以放松优化过程中的限制条件，使得更易求解，第二次调用Generate()
    ret = path_generator.Generate(obstacles, speed_data, init_point, boundary_extension, is_final_attempt, path_data);
    if (!ret)
    {
      const std::string msg = "failed to generate spline path at final attempt.";
      std::cout << msg;
      return false;
    }
  }
  return true;
}