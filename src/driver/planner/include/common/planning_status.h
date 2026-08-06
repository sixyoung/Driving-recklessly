#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <iostream>

namespace planning {

// ==================== 辅助结构 ====================

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// ==================== BareIntersectionStatus ====================

struct BareIntersectionStatus {
  std::string current_pnc_junction_overlap_id;
  std::string done_pnc_junction_overlap_id;
  uint32_t clear_counter = 0;
};

// ==================== ChangeLaneStatus ====================

struct ChangeLaneStatus {
  enum class Status {
    IN_CHANGE_LANE = 1,        // 正在换道
    CHANGE_LANE_FAILED = 2,    // 换道失败
    CHANGE_LANE_FINISHED = 3   // 换道完成
  };

  Status status = Status::IN_CHANGE_LANE;
  std::string path_id;                     // 当前行驶参考线 ID
  double timestamp = 0.0;                  // 状态开始时间
  bool exist_lane_change_start_position = false;
  Point3D lane_change_start_position;      // 换道起点
  double last_succeed_timestamp = 0.0;     // 上次成功换道时间
  bool is_current_opt_succeed = false;     // 当前规划是否成功
  bool is_clear_to_change_lane = false;    // 周围环境是否允许换道

  void Print() const {
    std::cout << "[ChangeLaneStatus] status=" << static_cast<int>(status)
              << " path_id=" << path_id
              << " clear=" << is_clear_to_change_lane
              << " succeed=" << is_current_opt_succeed << std::endl;
  }
};

// ==================== StopTime ====================

struct StopTime {
  std::string obstacle_id;
  double stop_timestamp_sec = 0.0;
};

// ==================== CrosswalkStatus ====================

struct CrosswalkStatus {
  std::string crosswalk_id;
  std::vector<StopTime> stop_time;
  std::vector<std::string> finished_crosswalk;
};

// ==================== DestinationStatus ====================

struct DestinationStatus {
  bool has_passed_destination = false;
};

// ==================== EmergencyStopStatus ====================

struct EmergencyStopStatus {
  Point3D stop_fence_point;
};

// ==================== PathDeciderStatus ====================

struct PathDeciderStatus {
  enum class LaneBorrowDirection {
    LEFT_BORROW = 1,
    RIGHT_BORROW = 2
  };
  int32_t front_static_obstacle_cycle_counter = 0;
  int32_t able_to_use_self_lane_counter = 0;
  bool is_in_path_lane_borrow_scenario = false;
  std::string front_static_obstacle_id;
  std::vector<LaneBorrowDirection> decided_side_pass_direction;
};

// ==================== ReroutingStatus ====================

struct ReroutingStatus {
  double last_rerouting_time = 0.0;
  bool need_rerouting = false;
  // RoutingRequest 可根据你自己的类型扩展
};

// ==================== PlanningStatus ====================

struct PlanningStatus {
  BareIntersectionStatus bare_intersection;
  ChangeLaneStatus change_lane;
  CrosswalkStatus crosswalk;
  DestinationStatus destination;
  EmergencyStopStatus emergency_stop;
  PathDeciderStatus path_decider;
  ReroutingStatus rerouting;
};

}  // namespace planning
