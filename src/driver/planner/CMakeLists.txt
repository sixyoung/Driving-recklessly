cmake_minimum_required(VERSION 3.0.2)
project(planner)

# 使用 C++17 标准
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g -O0)
## 查找 catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  object_msgs
)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen QUIET)
find_package(qpOASES QUIET)
find_package(yaml-cpp REQUIRED)

## Declare catkin 包配置
catkin_package(
  INCLUDE_DIRS include 
  include/common
  include/em_planner
  include/em_planner/road_graph
  include/em_planner/path_time_heuristic
  include/em_planner/qp_spline_path
  include/em_planner/qp_spline_st_speed
  include/em_planner/speed_decider
  include/frenet
  include/obstacle
  include/path
  include/polynomial
  include/qp
  include/refrence_line
  include/spline
  include/vehicle_state
  LIBRARIES planner
  CATKIN_DEPENDS roscpp rospy std_msgs object_msgs
  DEPENDS Eigen3
)

# 包含头文件路径
include_directories(
  include
  include/common
  include/em_planner
  include/em_planner/road_graph
  include/em_planner/path_time_heuristic
  include/em_planner/qp_spline_path
  include/em_planner/qp_spline_st_speed
  include/em_planner/speed_decider
  include/frenet
  include/obstacle
  include/path
  include/polynomial
  include/qp
  include/refrence_line
  include/spline
  include/vehicle_state
  ${osqp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

# 核心库
add_library(planner
    src/frenet/cartesian_frenet_conversion.cpp

    src/polynomial/QuinticPolynomial.cpp
    src/polynomial/QuarticPolynomial.cpp
    src/polynomial/quartic_polynomial_curve1d.cpp
    src/polynomial/quintic_polynomial_curve1d.cpp
    src/polynomial/polynomial_xd.cpp

    src/spline/CubicSpline1D.cpp
    src/spline/CubicSpline2D.cpp
    src/spline/spline_1d_constraint.cpp
    src/spline/spline_1d_kernel.cpp
    src/spline/spline_seg_kernel.cpp
    src/spline/spline_1d_seg.cpp
    src/spline/spline_1d_solver.cpp
    src/spline/spline_1d.cpp
 
    src/obstacle/Obstacle.cpp
    src/obstacle/Obstacle_test.cpp
    src/obstacle/Obstacle_avoid.cpp
    src/obstacle/boundarys.cpp
    src/obstacle/Ob_prediction_trajectory.cpp

    src/em_planner/road_graph/dp_poly_path_optimizer.cpp
    src/em_planner/road_graph/dp_road_graph.cpp
    src/em_planner/road_graph/planning_context.cpp
    src/em_planner/road_graph/trajectory_cost.cpp
    src/em_planner/road_graph/waypoint_sampler.cpp

    src/em_planner/qp_spline_path/qp_frenet_frame.cpp
    src/em_planner/qp_spline_path/qp_spline_path_generator.cpp
    src/em_planner/qp_spline_path/qp_spline_path_optimizer.cpp
    src/em_planner/qp_spline_st_speed/qp_piecewise_st_graph.cpp
    src/em_planner/qp_spline_st_speed/qp_spline_st_graph.cpp
    src/em_planner/qp_spline_st_speed/qp_spline_st_speed_optimizer.cpp
    src/em_planner/path_time_heuristic/st_graph_point.cpp
    src/em_planner/path_time_heuristic/st_graph_data.cpp
    src/em_planner/path_time_heuristic/dp_st_cost.cpp
    src/em_planner/path_time_heuristic/gridded_path_time_graph.cpp
    src/em_planner/path_time_heuristic/dp_st_graph.cpp
    src/em_planner/path_time_heuristic/path_time_heuristic_optimizer.cpp
    src/em_planner/speed_decider/speed_decider.cpp
    src/em_planner/em_planner.cpp

    src/common/angle.cpp
    src/common/auxiliary_function.cpp
    src/common/box2d.cpp
    src/common/Config.cpp
    src/common/line_segment2d.cpp
    src/common/math_utils.cpp
    src/common/path_time_graph.cpp
    src/common/polygon2d.cpp
    src/common/sin_table.cpp
    src/common/vec2d.cpp
    src/common/speed_limit.cpp
    src/common/speed_data.cpp
    src/common/speed_limit_decider.cpp
    src/common/PlanningTarget.cpp
    src/common/vehicle_model.cpp
    src/common/file_exporter.cpp
    
    src/path/path_data.cpp
    src/path/path_boundary.cpp
    src/path/path_bounds_decider.cpp
    src/path/path_matcher.cpp
    src/path/FrenetPath.cpp
    src/path/trajectoryPoint.cpp
    src/path/path_points.cpp
    src/path/publishable_trajectory.cpp
    src/path/trajectory_stitcher.cpp

    src/refrence_line/reference_point.cpp
    src/refrence_line/reference_line.cpp
    src/refrence_line/reference_line_info.cpp

    src/vehicle_state/vehicle_state_provider.cpp

    src/qp/constant_jerk_trajectory1d.cpp
    src/qp/lateral_osqp_optimizer.cpp
    src/qp/lateral_qp_optimizer.cpp
    src/qp/piecewise_jerk_trajectory1d.cpp
    src/qp/piecewise_jerk_path_problem.cpp
    src/qp/piecewise_jerk_problem.cpp
    src/qp/piecewise_jerk_speed_problem.cpp
    src/qp/osqp_spline_1d_solver.cpp
    src/qp/piecewise_linear_constraint.cpp
    src/qp/piecewise_linear_generator.cpp
    src/qp/piecewise_linear_kernel.cpp
    src/qp/qp_solver.cpp
    src/qp/fem_1d_qp_problem.cpp
    src/qp/fem_1d_expanded_jerk_qp_problem.cpp
    src/qp/active_set_spline_1d_solver.cpp
    src/qp/active_set_qp_solver.cpp
    src/qp/affine_constraint.cpp
    src/qp/fem_pos_deviation_ipopt_interface.cpp
    src/qp/fem_pos_deviation_osqp_interface.cpp
    src/qp/fem_pos_smoother.cpp
    src/qp/fem_pos_deviation_sqp_osqp_interface.cpp
    # src/test/test_qp_speed.cpp
)

# 添加头文件目录
target_include_directories(planner PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(planner 
  ${catkin_LIBRARIES} 
  ${Python3_LIBRARIES} 
  osqp qpOASES adolc 
  ipopt yaml-cpp cxsparse 
  )


  add_dependencies(planner ${catkin_EXPORTED_TARGETS})
# 创建测试可执行文件
# add_executable(test_dp_path src/test/test_dp_path.cpp)
# add_executable(test_dp_speed src/test/test_dp_speed.cpp)
# add_executable(test_qp_speed src/test/test_qp_speed.cpp)
# add_executable(test_cubic_spline src/test/test/scripts/test_cubic_spline.cpp)

# 将 test_dp_path 库与测试程序链接
# target_link_libraries(test_dp_path 
#   ${catkin_LIBRARIES}
#   planner
# )
# target_link_libraries( test_dp_speed
#   ${catkin_LIBRARIES}
#   planner
# )
# target_link_libraries( test_qp_speed
#   ${catkin_LIBRARIES}
#   planner
# )
# target_link_libraries( test_cubic_spline
#   ${catkin_LIBRARIES}
#   planner
# )

# 确保测试程序依赖于 cartesian_frenet_converter 库
# add_dependencies(test_dp_path planner)
# add_dependencies(test_dp_speed planner)
# add_dependencies(test_qp_speed planner)
# add_dependencies(test_cubic_spline planner)

