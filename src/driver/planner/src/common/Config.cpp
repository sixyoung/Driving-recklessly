#include "Configs.h"
#include <iostream>
#include <string>
#include <vector>

Param_Configs::Param_Configs()
{
  config = YAML::LoadFile("/home/bob/文档/备份/demo05/src/driver/planner/config/configs.yaml");
  role_name_ = "default";
  scene_type = "none";
  frame_id = 0;
  avoid_vehicle_ = true;
  avoid_traffic_light_= true;
  detection_range = 50;
  lane_change = false;
  sudden_speed_change = false;
  change_speed = false;
  recovery_speed = false;
  speed_abnormal = false;
  lane_change_cruise_speed = 10;
  lane_change_planning_upper_speed_limit = 10;
  is_avoid_vehicle_ = true;
  elapsed_time = false;
  abnormal =false;
  merge_onramp_over_speed =false;
  ignored_vehicle_roles.clear();

  FLAGS_trajectory_time_resolution = config["FLAGS_trajectory_time_resolution"].as<double>();
  FLAGS_weight_target_speed = config["FLAGS_weight_target_speed"].as<double>();
  FLAGS_weight_dist_travelled = config["FLAGS_weight_dist_travelled"].as<double>();
  FLAGS_longitudinal_jerk_lower_bound = config["FLAGS_longitudinal_jerk_lower_bound"].as<double>();
  FLAGS_longitudinal_jerk_upper_bound = config["FLAGS_longitudinal_jerk_upper_bound"].as<double>();
  FLAGS_lon_collision_cost_std = config["FLAGS_lon_collision_cost_std"].as<double>();
  FLAGS_lon_collision_yield_buffer = config["FLAGS_lon_collision_yield_buffer"].as<double>();
  FLAGS_lon_collision_overtake_buffer = config["FLAGS_lon_collision_overtake_buffer"].as<double>();

  FLAGS_weight_lon_objective = config["FLAGS_weight_lon_objective"].as<double>();
  FLAGS_weight_lon_jerk = config["FLAGS_weight_lon_jerk"].as<double>();
  FLAGS_weight_lon_collision = config["FLAGS_weight_lon_collision"].as<double>();
  FLAGS_weight_centripetal_acceleration = config["FLAGS_weight_centripetal_acceleration"].as<double>();
  FLAGS_weight_lat_offset = config["FLAGS_weight_lat_offset"].as<double>();
  FLAGS_weight_lat_comfort = config["FLAGS_weight_lat_comfort"].as<double>();

  FLAGS_speed_lon_decision_horizon = config["FLAGS_speed_lon_decision_horizon"].as<double>();
  FLAGS_trajectory_space_resolution = config["FLAGS_trajectory_space_resolution"].as<double>();
  FLAGS_lat_offset_bound = config["FLAGS_lat_offset_bound"].as<double>();
  FLAGS_weight_opposite_side_offset = config["FLAGS_weight_opposite_side_offset"].as<double>();
  FLAGS_weight_same_side_offset = config["FLAGS_weight_same_side_offset"].as<double>();

  FLAGS_speed_lower_bound = config["FLAGS_speed_lower_bound"].as<double>();
  FLAGS_speed_upper_bound = config["FLAGS_speed_upper_bound"].as<double>();
  FLAGS_longitudinal_acceleration_lower_bound = config["FLAGS_longitudinal_acceleration_lower_bound"].as<double>();
  FLAGS_longitudinal_acceleration_upper_bound = config["FLAGS_longitudinal_acceleration_upper_bound"].as<double>();
  FLAGS_comfort_acceleration_factor = config["FLAGS_comfort_acceleration_factor"].as<double>();
  FLAGS_Max_curvature = config["FLAGS_Max_curvature"].as<double>();
  FLAGS_kappa_bound = config["FLAGS_kappa_bound"].as<double>();
  FLAGS_dkappa_bound = config["FLAGS_dkappa_bound"].as<double>();
  FLAGS_lateral_acceleration_bound = config["FLAGS_lateral_acceleration_bound"].as<double>();
  FLAGS_lateral_jerk_bound = config["FLAGS_lateral_jerk_bound"].as<double>();

  FLAGS_trajectory_time_length = config["FLAGS_trajectory_time_length"].as<double>();
  FLAGS_polynomial_minimal_param = config["FLAGS_polynomial_minimal_param"].as<double>();
  FLAGS_num_velocity_sample = config["FLAGS_num_velocity_sample"].as<double>();
  FLAGS_min_velocity_sample_gap = config["FLAGS_min_velocity_sample_gap"].as<double>();

  FLAGS_enable_osqp_debug = config["FLAGS_enable_osqp_debug"].as<bool>();
  FLAGS_weight_lateral_offset = config["FLAGS_weight_lateral_offset"].as<double>();
  FLAGS_weight_lateral_derivative = config["FLAGS_weight_lateral_derivative"].as<double>();
  FLAGS_weight_lateral_second_order_derivative = config["FLAGS_weight_lateral_second_order_derivative"].as<double>();
  FLAGS_lateral_third_order_derivative_max = config["FLAGS_lateral_third_order_derivative_max"].as<double>();
  FLAGS_weight_lateral_obstacle_distance = config["FLAGS_weight_lateral_obstacle_distance"].as<double>();
  FLAGS_default_delta_s_lateral_optimization = config["FLAGS_default_delta_s_lateral_optimization"].as<double>();
  FLAGS_max_s_lateral_optimization = config["FLAGS_max_s_lateral_optimization"].as<double>();
  FLAGS_bound_buffer = config["FLAGS_bound_buffer"].as<double>();
  FLAGS_numerical_epsilon = config["FLAGS_numerical_epsilon"].as<double>();
  FLAGS_nudge_buffer = config["FLAGS_nudge_buffer"].as<double>();
  kSampleDistance = config["kSampleDistance"].as<double>();
  FLAGS_lon_collision_buffer = config["FLAGS_lon_collision_buffer"].as<double>();
  FLAGS_lat_collision_buffer = config["FLAGS_lat_collision_buffer"].as<double>();
  FLAGS_lattice_stop_buffer = config["FLAGS_lattice_stop_buffer"].as<double>();

  min_turn_radius = config["min_turn_radius"].as<double>();
  max_acceleration = config["max_acceleration"].as<double>();
  min_deceleration = config["min_deceleration"].as<double>();
  max_deceleration = config["max_deceleration"].as<double>();
  preferred_max_acceleration = config["preferred_max_acceleration"].as<double>();
  preferred_min_deceleration = config["preferred_min_deceleration"].as<double>();
  wheel_rolling_radius = config["wheel_rolling_radius"].as<double>();
  brake_deadzone = config["brake_deadzone"].as<double>();
  throttle_deadzone = config["throttle_deadzone"].as<double>();

  FLAGS_default_lon_buffer = config["FLAGS_default_lon_buffer"].as<double>();
  FLAGS_time_min_density = config["FLAGS_time_min_density"].as<double>();
  FLAGS_num_sample_follow_per_timestamp = config["FLAGS_num_sample_follow_per_timestamp"].as<double>();
  FLAGS_vehicle_width = config["FLAGS_vehicle_width"].as<double>();
  FLAGS_vehicle_length = config["FLAGS_vehicle_length"].as<double>();

  front_edge_to_center = config["front_edge_to_center"].as<double>();
  back_edge_to_center = config["back_edge_to_center"].as<double>();
  left_edge_to_center = config["left_edge_to_center"].as<double>();
  right_edge_to_center = config["right_edge_to_center"].as<double>();

  wheel_base = config["wheel_base"].as<double>();
  max_steer_angle = config["max_steer_angle"].as<double>();
  max_steer_angle_rate = config["max_steer_angle_rate"].as<double>();
  steer_ratio = config["steer_ratio"].as<double>();
  max_abs_speed_when_stopped = config["max_abs_speed_when_stopped"].as<double>();

  FLAGS_enable_sqp_solver = config["FLAGS_enable_sqp_solver"].as<bool>();
  default_active_set_eps_num = config["default_active_set_eps_num"].as<double>();
  default_active_set_eps_den = config["default_active_set_eps_den"].as<double>();
  default_active_set_eps_iter_ref = config["default_active_set_eps_iter_ref"].as<double>();
  default_qp_smoothing_eps_num = config["default_qp_smoothing_eps_num"].as<double>();
  default_qp_smoothing_eps_den = config["default_qp_smoothing_eps_den"].as<double>();
  default_qp_smoothing_eps_iter_ref = config["default_qp_smoothing_eps_iter_ref"].as<double>();
  default_enable_active_set_debug_info = config["default_enable_active_set_debug_info"].as<bool>();
  default_qp_iteration_num = config["default_qp_iteration_num"].as<uint32_t>();

  look_forward_time_sec = config["look_forward_time_sec"].as<double>();
  PathLength = config["PathLength"].as<double>();
  step_length_max = config["step_length_max"].as<double>();
  step_length_min = config["step_length_min"].as<double>();
  uturn_speed_limit = config["uturn_speed_limit"].as<double>();
  default_cruise_speed = config["default_cruise_speed"].as<double>();
  default_init_speed = config["default_init_speed"].as<double>();
  dynamic_obs_predict_time = config["dynamic_obs_predict_time"].as<double>();

  planning_upper_speed_limit = config["planning_upper_speed_limit"].as<double>();
  max_spline_length = config["max_spline_length"].as<double>();
  max_constraint_interval = config["max_constraint_interval"].as<double>();
  change_lane_speed_relax_percentage = config["change_lane_speed_relax_percentage"].as<double>();
  reference_line_weight = config["reference_line_weight"].as<double>();
  history_path_weight = config["history_path_weight"].as<double>();
  derivative_weight = config["derivative_weight"].as<double>();
  first_spline_weight_factor = config["first_spline_weight_factor"].as<double>();
  second_derivative_weight = config["second_derivative_weight"].as<double>();
  third_derivative_weight = config["third_derivative_weight"].as<double>();
  lane_change_mid_l = config["lane_change_mid_l"].as<double>();

  FLAGS_dl_bound = config["FLAGS_dl_bound"].as<double>();
  spline_order = config["spline_order"].as<uint32_t>();
  time_resolution = config["time_resolution"].as<double>();
  num_output = config["num_output"].as<uint32_t>();
  point_constraint_s_position = config["point_constraint_s_position"].as<double>();
  cross_lane_lateral_extension = config["cross_lane_lateral_extension"].as<double>();
  FLAGS_default_reference_line_width = config["FLAGS_default_reference_line_width"].as<double>();

  unit_t = config["unit_t"].as<double>();
  safe_distance = config["safe_distance"].as<double>();
  total_path_length = config["total_path_length"].as<double>();
  total_time = config["total_time"].as<double>();
  dense_dimension_s = config["dense_dimension_s"].as<uint32_t>();
  dense_unit_s = config["dense_unit_s"].as<double>();
  sparse_unit_s = config["sparse_unit_s"].as<double>();
  matrix_dimension_s = config["matrix_dimension_s"].as<uint32_t>();
  matrix_dimension_t = config["matrix_dimension_t"].as<double>();
  obstacle_weight = config["obstacle_weight"].as<double>();
  default_obstacle_cost = config["default_obstacle_cost"].as<double>();
  spatial_potential_penalty = config["spatial_potential_penalty"].as<double>();

  reference_weight = config["reference_weight"].as<double>();
  keep_clear_low_speed_penalty = config["keep_clear_low_speed_penalty"].as<double>();
  default_speed_cost = config["default_speed_cost"].as<double>();
  exceed_speed_penalty = config["exceed_speed_penalty"].as<double>();
  low_speed_penalty = config["low_speed_penalty"].as<double>();
  reference_speed_penalty = config["reference_speed_penalty"].as<double>();
  accel_penalty = config["accel_penalty"].as<double>();
  decel_penalty = config["decel_penalty"].as<double>();
  positive_jerk_coeff = config["positive_jerk_coeff"].as<double>();
  negative_jerk_coeff = config["negative_jerk_coeff"].as<double>();
  FLAGS_use_st_drivable_boundary = config["FLAGS_use_st_drivable_boundary"].as<bool>();
  FLAGS_enable_dp_reference_speed = config["FLAGS_enable_dp_reference_speed"].as<bool>();

  FLAGS_use_navigation_mode = config["FLAGS_use_navigation_mode"].as<bool>();
  IsChangeLanePath = config["IsChangeLanePath"].as<bool>();
  IsClearToChangeLane = config["IsClearToChangeLane"].as<bool>();
  sample_points_num_each_level = config["sample_points_num_each_level"].as<int>();
  navigator_sample_num_each_level = config["navigator_sample_num_each_level"].as<int>();
  lateral_sample_offset = config["lateral_sample_offset"].as<double>();
  lateral_adjust_coeff = config["lateral_adjust_coeff"].as<double>();
  sidepass_distance = config["sidepass_distance"].as<double>();

  eval_time_interval = config["eval_time_interval"].as<double>();
  path_resolution = config["path_resolution"].as<double>();
  obstacle_ignore_distance = config["obstacle_ignore_distance"].as<double>();
  obstacle_collision_distance = config["obstacle_collision_distance"].as<double>();
  obstacle_risk_distance = config["obstacle_risk_distance"].as<double>();
  obstacle_collision_cost = config["obstacle_collision_cost"].as<double>();
  path_l_cost = config["path_l_cost"].as<double>();
  path_dl_cost = config["path_dl_cost"].as<double>();
  path_ddl_cost = config["path_ddl_cost"].as<double>();
  path_l_cost_param_l0 = config["path_l_cost_param_l0"].as<double>();
  path_l_cost_param_b = config["path_l_cost_param_b"].as<double>();
  path_l_cost_param_k = config["path_l_cost_param_k"].as<double>();
  path_out_lane_cost = config["path_out_lane_cost"].as<double>();
  path_end_l_cost = config["path_end_l_cost"].as<double>();
  FLAGS_prediction_total_time = config["FLAGS_prediction_total_time"].as<double>();
  FLAGS_lateral_ignore_buffer = config["FLAGS_lateral_ignore_buffer"].as<double>();
  FLAGS_slowdown_profile_deceleration = config["FLAGS_slowdown_profile_deceleration"].as<double>();
  FLAGS_trajectory_time_min_interval = config["FLAGS_trajectory_time_min_interval"].as<double>();
  FLAGS_trajectory_time_max_interval = config["FLAGS_trajectory_time_max_interval"].as<double>();
  FLAGS_trajectory_time_high_density_period = config["FLAGS_trajectory_time_high_density_period"].as<double>();
  FLAGS_min_stop_distance_obstacle = config["FLAGS_min_stop_distance_obstacle"].as<double>();
  FLAGS_use_osqp_optimizer_for_qp_st = config["FLAGS_use_osqp_optimizer_for_qp_st"].as<bool>();
  FLAGS_enable_follow_accel_constraint = config["FLAGS_enable_follow_accel_constraint"].as<bool>();
  number_of_discrete_graph_t = config["number_of_discrete_graph_t"].as<double>();
  accel_kernel_weight = config["accel_kernel_weight"].as<double>();
  jerk_kernel_weight = config["jerk_kernel_weight"].as<double>();
  stop_weight = config["stop_weight"].as<double>();
  cruise_weight = config["cruise_weight"].as<double>();
  follow_weight = config["follow_weight"].as<double>();
  regularization_weight = config["regularization_weight"].as<double>();
  follow_drag_distance = config["follow_drag_distance"].as<double>();
  dp_st_reference_weight = config["dp_st_reference_weight"].as<double>();
  init_jerk_kernel_weight = config["init_jerk_kernel_weight"].as<double>();
  yield_weight = config["yield_weight"].as<double>();
  yield_drag_distance = config["yield_drag_distance"].as<double>();
  number_of_evaluated_graph_t = config["number_of_evaluated_graph_t"].as<double>();

  boundary_buffer = config["boundary_buffer"].as<double>();
  high_speed_centric_acceleration_limit = config["high_speed_centric_acceleration_limit"].as<double>();
  low_speed_centric_acceleration_limit = config["low_speed_centric_acceleration_limit"].as<double>();
  high_speed_threshold = config["high_speed_threshold"].as<double>();
  low_speed_threshold = config["low_speed_threshold"].as<double>();
  minimal_kappa = config["minimal_kappa"].as<double>();
  point_extension = config["point_extension"].as<double>();
  lowest_speed = config["lowest_speed"].as<double>();
  num_points_to_avg_kappa = config["num_points_to_avg_kappa"].as<double>();
  static_obs_nudge_speed_ratio = config["static_obs_nudge_speed_ratio"].as<double>();
  dynamic_obs_nudge_speed_ratio = config["dynamic_obs_nudge_speed_ratio"].as<double>();
  centri_jerk_speed_coeff = config["centri_jerk_speed_coeff"].as<double>();
  FLAGS_enable_nudge_slowdown = config["FLAGS_enable_nudge_slowdown"].as<bool>();

  max_forward_v = config["max_forward_v"].as<double>();
  max_reverse_v = config["max_reverse_v"].as<double>();
  max_forward_acc = config["max_forward_acc"].as<double>();
  max_reverse_acc = config["max_reverse_acc"].as<double>();
  max_acc_jerk = config["max_acc_jerk"].as<double>();

  xy_grid_resolution = config["xy_grid_resolution"].as<double>();
  phi_grid_resolution = config["phi_grid_resolution"].as<double>();
  grid_a_star_xy_resolution = config["grid_a_star_xy_resolution"].as<double>();
  node_radius = config["node_radius"].as<double>();
  step_size = config["step_size"].as<double>();
  FLAGS_enable_parallel_hybrid_a = config["FLAGS_enable_parallel_hybrid_a"].as<bool>();

  next_node_num = config["next_node_num"].as<double>();
  delta_t = config["delta_t"].as<double>();
  traj_forward_penalty = config["traj_forward_penalty"].as<double>();
  traj_back_penalty = config["traj_back_penalty"].as<double>();
  traj_gear_switch_penalty = config["traj_gear_switch_penalty"].as<double>();
  traj_steer_penalty = config["traj_steer_penalty"].as<double>();
  traj_steer_change_penalty = config["traj_steer_change_penalty"].as<double>();
  FLAGS_use_s_curve_speed_smooth = config["FLAGS_use_s_curve_speed_smooth"].as<bool>();

  interpolated_delta_s = config["interpolated_delta_s"].as<double>();
  reanchoring_trails_num = config["reanchoring_trails_num"].as<double>();
  reanchoring_pos_stddev = config["reanchoring_pos_stddev"].as<double>();
  reanchoring_length_stddev = config["reanchoring_length_stddev"].as<double>();
  estimate_bound = config["estimate_bound"].as<bool>();
  default_bound = config["default_bound"].as<double>();
  vehicle_shortest_dimension = config["vehicle_shortest_dimension"].as<double>();
  collision_decrease_ratio = config["collision_decrease_ratio"].as<double>();

  acc_weight = config["acc_weight"].as<double>();
  jerk_weight = config["jerk_weight"].as<double>();
  kappa_penalty_weight = config["kappa_penalty_weight"].as<double>();
  ref_s_weight = config["ref_s_weight"].as<double>();
  ref_v_weight = config["ref_v_weight"].as<double>();
  fallback_total_time = config["fallback_total_time"].as<double>();
  fallback_time_unit = config["fallback_time_unit"].as<double>();

  qp_delta_s = config["qp_delta_s"].as<double>();
  min_look_ahead_time = config["min_look_ahead_time"].as<double>();
  min_look_ahead_distance = config["min_look_ahead_distance"].as<double>();
  lateral_buffer = config["lateral_buffer"].as<double>();
  path_output_resolution = config["path_output_resolution"].as<double>();
  FLAGS_replan_lateral_distance_threshold = config["FLAGS_replan_lateral_distance_threshold"].as<double>();
  FLAGS_replan_longitudinal_distance_threshold = config["FLAGS_replan_longitudinal_distance_threshold"].as<double>();
  InitParamMap();
}

Param_Configs::~Param_Configs()
{
}

// =============================
// double 类型
// =============================
bool Param_Configs::SetParamValue(const std::string& name, double value) {
    // 2. 查映射表
    auto it = double_params_.find(name);
    if (it != double_params_.end()) {
        *(it->second) = value;
        std::cout << "[Config] ✅ 更新 double 参数: " << name << " = " << value << std::endl;
        return true;
    }

    std::cerr << "[Config] ⚠️ 未注册的 double 参数: " << name << std::endl;
    return false;
}

bool Param_Configs::SetParamValue(
    const std::string& name,
    const std::vector<std::string>& values)
{
    if (name == "ignored_vehicle_roles") {
        ignored_vehicle_roles = values;

        std::cout << "[Config] ✅ 更新 ignored_vehicle_roles: ";
        for (const auto& v : values) std::cout << v << " ";
        std::cout << std::endl;

        return true;
    }

    std::cerr << "[Config] ⚠️ 未注册的 string list 参数: "
              << name << std::endl;
    return false;
}

// =============================
// int 类型
// =============================
bool Param_Configs::SetParamValue(const std::string& name, int value) {
    auto it = int_params_.find(name);
    if (it != int_params_.end()) {
        *(it->second) = value;
        std::cout << "[Config] ✅ 更新 int 参数: " << name << " = " << value << std::endl;
        return true;
    }

    std::cerr << "[Config] ⚠️ 未注册的 int 参数: " << name << std::endl;
    return false;
}

// =============================
// bool 类型
// =============================
bool Param_Configs::SetParamValue(const std::string& name, bool value) {
    auto it = bool_params_.find(name);
    if (it != bool_params_.end()) {
        *(it->second) = value;
        std::cout << "[Config] ✅ 更新 bool 参数: " << name << " = " << (value ? "true" : "false") << std::endl;
        return true;
    }

    std::cerr << "[Config] ⚠️ 未注册的 bool 参数: " << name << std::endl;
    return false;
}

// =============================
// string 类型
// =============================
bool Param_Configs::SetParamValue(const std::string& name, const std::string& value) {
    auto it = string_params_.find(name);
    if (it != string_params_.end()) {
        *(it->second) = value;
        std::cout << "[Config] ✅ 更新 string 参数: " << name << " = '" << value << "'" << std::endl;
        return true;
    }

    // 尝试解析成 double 或 bool
    if (value == "true" || value == "false") {
        return SetParamValue(name, value == "true");
    }
    try {
        double d = std::stod(value);
        return SetParamValue(name, d);
    } catch (...) {}

    std::cerr << "[Config] ⚠️ 未注册的 string 参数: " << name << std::endl;
    return false;
}

// =============================
// size_t 类型
// =============================
bool Param_Configs::SetParamValue(const std::string& name, size_t value) {
    auto it = size_t_params_.find(name);
    if (it != size_t_params_.end()) {
        *(it->second) = value;
        std::cout << "[Config] ✅ 更新 size_t 参数: " << name << " = " << value << std::endl;
        return true;
    }

    std::cerr << "[Config] ⚠️ 未注册的 size_t 参数: " << name << std::endl;
    return false;
}

bool Param_Configs::SetParamValueAuto(const std::string& name, double value)
{
    // 优先查 double 表
    if (double_params_.count(name)) {
        return SetParamValue(name, value);
    }

    // bool：0 或 1 自动转为 false / true
    if (bool_params_.count(name)) {
        return SetParamValue(name, value > 0.5);
    }

    // int：四舍五入
    if (int_params_.count(name)) {
        return SetParamValue(name, static_cast<int>(std::round(value)));
    }

    // size_t：取整
    if (size_t_params_.count(name)) {
        return SetParamValue(name, static_cast<size_t>(std::round(value)));
    }

    if (list_params_.count(name)) {
        return SetParamValue(name, value);
    }

    if (string_params_.count(name)) {
        return SetParamValue(name, value);
    }
    std::cerr << "[Config] ⚠️ 未识别的参数: " << name << std::endl;
    return false;
}

void Param_Configs::InitParamMap() {
    // ========= double =========
    double_params_ = {
        {"FLAGS_trajectory_time_resolution", &FLAGS_trajectory_time_resolution},
        {"FLAGS_weight_target_speed", &FLAGS_weight_target_speed},
        {"FLAGS_weight_dist_travelled", &FLAGS_weight_dist_travelled},
        {"FLAGS_longitudinal_jerk_lower_bound", &FLAGS_longitudinal_jerk_lower_bound},
        {"FLAGS_longitudinal_jerk_upper_bound", &FLAGS_longitudinal_jerk_upper_bound},
        {"FLAGS_lon_collision_cost_std", &FLAGS_lon_collision_cost_std},
        {"FLAGS_lon_collision_yield_buffer", &FLAGS_lon_collision_yield_buffer},
        {"FLAGS_lon_collision_overtake_buffer", &FLAGS_lon_collision_overtake_buffer},

        {"FLAGS_weight_lon_objective", &FLAGS_weight_lon_objective},
        {"FLAGS_weight_lon_jerk", &FLAGS_weight_lon_jerk},
        {"FLAGS_weight_lon_collision", &FLAGS_weight_lon_collision},
        {"FLAGS_weight_centripetal_acceleration", &FLAGS_weight_centripetal_acceleration},
        {"FLAGS_weight_lat_offset", &FLAGS_weight_lat_offset},
        {"FLAGS_weight_lat_comfort", &FLAGS_weight_lat_comfort},

        {"FLAGS_speed_lon_decision_horizon", &FLAGS_speed_lon_decision_horizon},
        {"FLAGS_trajectory_space_resolution", &FLAGS_trajectory_space_resolution},
        {"FLAGS_lat_offset_bound", &FLAGS_lat_offset_bound},
        {"FLAGS_weight_opposite_side_offset", &FLAGS_weight_opposite_side_offset},
        {"FLAGS_weight_same_side_offset", &FLAGS_weight_same_side_offset},

        {"FLAGS_speed_lower_bound", &FLAGS_speed_lower_bound},
        {"FLAGS_speed_upper_bound", &FLAGS_speed_upper_bound},
        {"FLAGS_longitudinal_acceleration_lower_bound", &FLAGS_longitudinal_acceleration_lower_bound},
        {"FLAGS_longitudinal_acceleration_upper_bound", &FLAGS_longitudinal_acceleration_upper_bound},
        {"FLAGS_comfort_acceleration_factor", &FLAGS_comfort_acceleration_factor},
        {"FLAGS_Max_curvature", &FLAGS_Max_curvature},
        {"FLAGS_kappa_bound", &FLAGS_kappa_bound},
        {"FLAGS_dkappa_bound", &FLAGS_dkappa_bound},
        {"FLAGS_lateral_acceleration_bound", &FLAGS_lateral_acceleration_bound},
        {"FLAGS_lateral_jerk_bound", &FLAGS_lateral_jerk_bound},

        {"FLAGS_trajectory_time_length", &FLAGS_trajectory_time_length},
        {"FLAGS_polynomial_minimal_param", &FLAGS_polynomial_minimal_param},
        {"FLAGS_num_velocity_sample", &FLAGS_num_velocity_sample},
        {"FLAGS_min_velocity_sample_gap", &FLAGS_min_velocity_sample_gap},

        {"FLAGS_weight_lateral_offset", &FLAGS_weight_lateral_offset},
        {"FLAGS_weight_lateral_derivative", &FLAGS_weight_lateral_derivative},
        {"FLAGS_weight_lateral_second_order_derivative", &FLAGS_weight_lateral_second_order_derivative},
        {"FLAGS_lateral_third_order_derivative_max", &FLAGS_lateral_third_order_derivative_max},
        {"FLAGS_weight_lateral_obstacle_distance", &FLAGS_weight_lateral_obstacle_distance},
        {"FLAGS_default_delta_s_lateral_optimization", &FLAGS_default_delta_s_lateral_optimization},
        {"FLAGS_max_s_lateral_optimization", &FLAGS_max_s_lateral_optimization},
        {"FLAGS_bound_buffer", &FLAGS_bound_buffer},
        {"FLAGS_numerical_epsilon", &FLAGS_numerical_epsilon},
        {"FLAGS_nudge_buffer", &FLAGS_nudge_buffer},

        {"kSampleDistance", &kSampleDistance},
        {"FLAGS_lon_collision_buffer", &FLAGS_lon_collision_buffer},
        {"FLAGS_lat_collision_buffer", &FLAGS_lat_collision_buffer},
        {"FLAGS_lattice_stop_buffer", &FLAGS_lattice_stop_buffer},

        {"min_turn_radius", &min_turn_radius},
        {"max_acceleration", &max_acceleration},
        {"min_deceleration", &min_deceleration},
        {"max_deceleration", &max_deceleration},
        {"preferred_max_acceleration", &preferred_max_acceleration},
        {"preferred_min_deceleration", &preferred_min_deceleration},
        {"wheel_rolling_radius", &wheel_rolling_radius},
        {"brake_deadzone", &brake_deadzone},
        {"throttle_deadzone", &throttle_deadzone},

        {"FLAGS_default_lon_buffer", &FLAGS_default_lon_buffer},
        {"FLAGS_time_min_density", &FLAGS_time_min_density},
        {"FLAGS_num_sample_follow_per_timestamp", &FLAGS_num_sample_follow_per_timestamp},
        {"FLAGS_vehicle_width", &FLAGS_vehicle_width},
        {"FLAGS_vehicle_length", &FLAGS_vehicle_length},

        {"front_edge_to_center", &front_edge_to_center},
        {"back_edge_to_center", &back_edge_to_center},
        {"left_edge_to_center", &left_edge_to_center},
        {"right_edge_to_center", &right_edge_to_center},

        {"wheel_base", &wheel_base},
        {"max_steer_angle", &max_steer_angle},
        {"max_steer_angle_rate", &max_steer_angle_rate},
        {"steer_ratio", &steer_ratio},
        {"max_abs_speed_when_stopped", &max_abs_speed_when_stopped},

        {"default_active_set_eps_num", &default_active_set_eps_num},
        {"default_active_set_eps_den", &default_active_set_eps_den},
        {"default_active_set_eps_iter_ref", &default_active_set_eps_iter_ref},
        {"default_qp_smoothing_eps_num", &default_qp_smoothing_eps_num},
        {"default_qp_smoothing_eps_den", &default_qp_smoothing_eps_den},
        {"default_qp_smoothing_eps_iter_ref", &default_qp_smoothing_eps_iter_ref},

        {"look_forward_time_sec", &look_forward_time_sec},
        {"PathLength", &PathLength},
        {"step_length_max", &step_length_max},
        {"step_length_min", &step_length_min},
        {"uturn_speed_limit", &uturn_speed_limit},
        {"default_cruise_speed", &default_cruise_speed},
        {"default_init_speed", &default_init_speed},
        {"dynamic_obs_predict_time", &dynamic_obs_predict_time},
        {"planning_upper_speed_limit", &planning_upper_speed_limit},
        {"max_spline_length", &max_spline_length},
        {"max_constraint_interval", &max_constraint_interval},
        {"change_lane_speed_relax_percentage", &change_lane_speed_relax_percentage},
        {"reference_line_weight", &reference_line_weight},
        {"history_path_weight", &history_path_weight},
        {"derivative_weight", &derivative_weight},
        {"first_spline_weight_factor", &first_spline_weight_factor},
        {"second_derivative_weight", &second_derivative_weight},
        {"third_derivative_weight", &third_derivative_weight},
        {"lane_change_mid_l", &lane_change_mid_l},

        {"FLAGS_dl_bound", &FLAGS_dl_bound},
        {"time_resolution", &time_resolution},
        {"point_constraint_s_position", &point_constraint_s_position},
        {"cross_lane_lateral_extension", &cross_lane_lateral_extension},
        {"FLAGS_default_reference_line_width", &FLAGS_default_reference_line_width},
        {"unit_t", &unit_t},
        {"safe_distance", &safe_distance},
        {"overtake_distance_s", &overtake_distance_s},
        {"total_path_length", &total_path_length},
        {"total_time", &total_time},
        {"dense_unit_s", &dense_unit_s},
        {"sparse_unit_s", &sparse_unit_s},
        {"matrix_dimension_t", &matrix_dimension_t},
        {"obstacle_weight", &obstacle_weight},
        {"default_obstacle_cost", &default_obstacle_cost},
        {"spatial_potential_penalty", &spatial_potential_penalty},
        {"reference_weight", &reference_weight},
        {"keep_clear_low_speed_penalty", &keep_clear_low_speed_penalty},
        {"default_speed_cost", &default_speed_cost},
        {"exceed_speed_penalty", &exceed_speed_penalty},
        {"low_speed_penalty", &low_speed_penalty},
        {"reference_speed_penalty", &reference_speed_penalty},
        {"accel_penalty", &accel_penalty},
        {"decel_penalty", &decel_penalty},
        {"positive_jerk_coeff", &positive_jerk_coeff},
        {"negative_jerk_coeff", &negative_jerk_coeff},

        {"lateral_sample_offset", &lateral_sample_offset},
        {"lateral_adjust_coeff", &lateral_adjust_coeff},
        {"sidepass_distance", &sidepass_distance},
        {"eval_time_interval", &eval_time_interval},
        {"path_resolution", &path_resolution},
        {"obstacle_ignore_distance", &obstacle_ignore_distance},
        {"obstacle_collision_distance", &obstacle_collision_distance},
        {"obstacle_risk_distance", &obstacle_risk_distance},
        {"obstacle_collision_cost", &obstacle_collision_cost},
        {"path_l_cost", &path_l_cost},
        {"path_dl_cost", &path_dl_cost},
        {"path_ddl_cost", &path_ddl_cost},
        {"path_l_cost_param_l0", &path_l_cost_param_l0},
        {"path_l_cost_param_b", &path_l_cost_param_b},
        {"path_l_cost_param_k", &path_l_cost_param_k},
        {"path_out_lane_cost", &path_out_lane_cost},
        {"path_end_l_cost", &path_end_l_cost},
        {"FLAGS_prediction_total_time", &FLAGS_prediction_total_time},
        {"FLAGS_lateral_ignore_buffer", &FLAGS_lateral_ignore_buffer},
        {"FLAGS_slowdown_profile_deceleration", &FLAGS_slowdown_profile_deceleration},
        {"FLAGS_trajectory_time_min_interval", &FLAGS_trajectory_time_min_interval},
        {"FLAGS_trajectory_time_max_interval", &FLAGS_trajectory_time_max_interval},
        {"FLAGS_trajectory_time_high_density_period", &FLAGS_trajectory_time_high_density_period},
        {"FLAGS_min_stop_distance_obstacle", &FLAGS_min_stop_distance_obstacle},
        {"detection_range", &detection_range},

        {"number_of_discrete_graph_t", &number_of_discrete_graph_t},
        {"accel_kernel_weight", &accel_kernel_weight},
        {"jerk_kernel_weight", &jerk_kernel_weight},
        {"stop_weight", &stop_weight},
        {"cruise_weight", &cruise_weight},
        {"follow_weight", &follow_weight},
        {"regularization_weight", &regularization_weight},
        {"follow_drag_distance", &follow_drag_distance},
        {"dp_st_reference_weight", &dp_st_reference_weight},
        {"init_jerk_kernel_weight", &init_jerk_kernel_weight},
        {"yield_weight", &yield_weight},
        {"yield_drag_distance", &yield_drag_distance},
        {"number_of_evaluated_graph_t", &number_of_evaluated_graph_t},
        {"boundary_buffer", &boundary_buffer},
        {"high_speed_centric_acceleration_limit", &high_speed_centric_acceleration_limit},
        {"low_speed_centric_acceleration_limit", &low_speed_centric_acceleration_limit},
        {"high_speed_threshold", &high_speed_threshold},
        {"low_speed_threshold", &low_speed_threshold},
        {"minimal_kappa", &minimal_kappa},
        {"point_extension", &point_extension},
        {"lowest_speed", &lowest_speed},
        {"num_points_to_avg_kappa", &num_points_to_avg_kappa},
        {"static_obs_nudge_speed_ratio", &static_obs_nudge_speed_ratio},
        {"dynamic_obs_nudge_speed_ratio", &dynamic_obs_nudge_speed_ratio},
        {"centri_jerk_speed_coeff", &centri_jerk_speed_coeff},

        {"max_forward_v", &max_forward_v},
        {"max_reverse_v", &max_reverse_v},
        {"max_forward_acc", &max_forward_acc},
        {"max_reverse_acc", &max_reverse_acc},
        {"max_acc_jerk", &max_acc_jerk},

        {"xy_grid_resolution", &xy_grid_resolution},
        {"phi_grid_resolution", &phi_grid_resolution},
        {"grid_a_star_xy_resolution", &grid_a_star_xy_resolution},
        {"node_radius", &node_radius},
        {"step_size", &step_size},

        {"next_node_num", &next_node_num},
        {"delta_t", &delta_t},
        {"traj_forward_penalty", &traj_forward_penalty},
        {"traj_back_penalty", &traj_back_penalty},
        {"traj_gear_switch_penalty", &traj_gear_switch_penalty},
        {"traj_steer_penalty", &traj_steer_penalty},
        {"traj_steer_change_penalty", &traj_steer_change_penalty},
        {"interpolated_delta_s", &interpolated_delta_s},
        {"reanchoring_trails_num", &reanchoring_trails_num},
        {"reanchoring_pos_stddev", &reanchoring_pos_stddev},
        {"reanchoring_length_stddev", &reanchoring_length_stddev},
        {"default_bound", &default_bound},
        {"vehicle_shortest_dimension", &vehicle_shortest_dimension},
        {"collision_decrease_ratio", &collision_decrease_ratio},

        {"acc_weight", &acc_weight},
        {"jerk_weight", &jerk_weight},
        {"kappa_penalty_weight", &kappa_penalty_weight},
        {"ref_s_weight", &ref_s_weight},
        {"ref_v_weight", &ref_v_weight},
        {"fallback_total_time", &fallback_total_time},
        {"fallback_time_unit", &fallback_time_unit},
        {"qp_delta_s", &qp_delta_s},
        {"min_look_ahead_time", &min_look_ahead_time},
        {"min_look_ahead_distance", &min_look_ahead_distance},
        {"lateral_buffer", &lateral_buffer},
        {"path_output_resolution", &path_output_resolution},
        {"FLAGS_replan_lateral_distance_threshold", &FLAGS_replan_lateral_distance_threshold},
        {"FLAGS_replan_longitudinal_distance_threshold", &FLAGS_replan_longitudinal_distance_threshold},
        {"lane_change_cruise_speed", &lane_change_cruise_speed},
        {"lane_change_planning_upper_speed_limit", &lane_change_planning_upper_speed_limit},
        {"speed_change_cruise_speed", &speed_change_cruise_speed},
        {"speed_change_planning_upper_speed_limit", &speed_change_planning_upper_speed_limit},

        {"last_cruise_speed", &last_cruise_speed},
        {"last_planning_upper_speed_limit", &last_planning_upper_speed_limit},
        {"recovery_start_time", &recovery_start_time},
        {"recovery_wait_time", &recovery_wait_time}
    };

    // ========= bool =========
    bool_params_ = {
        {"avoid_vehicle_", &avoid_vehicle_},
        {"avoid_traffic_light_", &avoid_traffic_light_},
        {"FLAGS_enable_osqp_debug", &FLAGS_enable_osqp_debug},
        {"FLAGS_enable_sqp_solver", &FLAGS_enable_sqp_solver},
        {"default_enable_active_set_debug_info", &default_enable_active_set_debug_info},
        {"FLAGS_use_st_drivable_boundary", &FLAGS_use_st_drivable_boundary},
        {"FLAGS_enable_dp_reference_speed", &FLAGS_enable_dp_reference_speed},
        {"FLAGS_use_navigation_mode", &FLAGS_use_navigation_mode},
        {"IsChangeLanePath", &IsChangeLanePath},
        {"IsClearToChangeLane", &IsClearToChangeLane},
        {"FLAGS_enable_follow_accel_constraint", &FLAGS_enable_follow_accel_constraint},
        {"FLAGS_enable_nudge_slowdown", &FLAGS_enable_nudge_slowdown},
        {"FLAGS_enable_parallel_hybrid_a", &FLAGS_enable_parallel_hybrid_a},
        {"FLAGS_use_s_curve_speed_smooth", &FLAGS_use_s_curve_speed_smooth},
        {"estimate_bound", &estimate_bound},
        {"lane_change", &lane_change},
        {"sudden_speed_change", &sudden_speed_change},
        {"change_speed", &change_speed},
        {"recovery_speed", &recovery_speed},
        {"speed_abnormal", &speed_abnormal},
        {"use_recovery_by_point", &use_recovery_by_point},
        {"is_avoid_vehicle_", &is_avoid_vehicle_},
        {"elapsed_time", &elapsed_time},
        {"abnormal", &abnormal},
        {"merge_onramp_over_speed", &merge_onramp_over_speed}
    };

    // ========= int / uint =========
    int_params_ = {
        {"sample_points_num_each_level", &sample_points_num_each_level},
        {"navigator_sample_num_each_level", &navigator_sample_num_each_level}
    };

    uint_params_ = {
        {"spline_order", &spline_order},
        {"num_output", &num_output},
        {"dense_dimension_s", &dense_dimension_s},
        {"matrix_dimension_s", &matrix_dimension_s},
        {"default_qp_iteration_num", &default_qp_iteration_num}
    };
    // ========= string =========
    string_params_ = {
        {"role_name_", &role_name_},
        {"scene_type", &scene_type}
    };

    list_params_ = {
        {"ignored_vehicle_roles", &ignored_vehicle_roles}
    };
    // ========== size_t ==========
    size_t_params_ = {
        {"frame_id", &frame_id}
    };
}