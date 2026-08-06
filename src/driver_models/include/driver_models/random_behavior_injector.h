#pragma once
/**
 * RandomBehaviorInjector
 * ----------------------
 * 目标：在“事件窗口”内接管车辆控制；窗口外不干预（上层发布基线控制）。
 * 设计：
 *  - beginCycle(state): 每帧喂状态（可由模块内部用(x,y)+全局路径计算s）
 *  - stepOverride(caps): 若处于事件窗口 -> 返回RBI的全量ControlCmd；否则返回std::nullopt
 *  - setEvents / clearEvents: 配置随意驾驶事件（按里程at_m触发，目标速度target等）
 *  - setGlobalPath: 提供路径，内部做s投影
 *
 * 注意：本模块内部**不保留/不使用**基线控制，以实现“完全解耦”。
 */

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <mutex>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <driver_models_types/PathWithOptions.h>
#include <common/common.h>

namespace rbt {

// 简单路径点
struct PathPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0}; // [rad]
};
using GlobalPath = std::vector<PathPoint>;

// 注入器配置
struct InjectorConfig {
  // —— 纵向 —— //
  double kp_throttle        = 0.25; // P-油门
  double kp_brake           = 0.45; // P-制动
  double speed_deadband     = 0.20; // 速度误差死区[m/s]
  double default_v_slew_rate= 1.8;  // 速度参考默认爬升/下降斜率[m/s^2]

  // —— 横向（简单P控制） —— //
  double k_yaw              = 0.80; // 航向误差权重
  double k_lat              = 0.12; // 横向误差权重
  double steer_slew_rate    = 2.00; // 方向盘斜坡限速[unit/s]
  double steer_limit        = 1.00; // 方向盘饱和[-1,1]
  // 【新增】前视距离：L(v) = clamp(la_min + la_k_v*v, la_min, la_max)
  double la_min_m       = 2.0;   // 最小前视距离 [m]
  double la_max_m       = 15.0;  // 最大前视距离 [m]
  double la_k_v         = 0.2;   // 每 m/s 增加的前视距离 [m/(m/s)]
  // 【新增】速度相关增益的“随速递减”因子（越快越保守）
  double yaw_gain_decay = 0.10;  // yaw 增益随速衰减系数 [1/(m/s)]
  double lat_gain_decay = 0.10;  // lat 增益随速衰减系数 [1/(m/s)]

  // —— s 投影行为 —— //
  bool   compute_s_internal = true; // 若true，则用(x,y)+路径自动计算s
  double hop_gain           = 2.0;  // s 的前向跳限（与位移成比例）
  double back_tolerance_m   = 1.0;  // s 的允许回退阈值

  // —— 其他 —— //
  std::string role_name{};
};

// 车辆状态（每帧输入）
struct State {
  double speed_mps{0.0};
  double x{0.0}, y{0.0}, yaw{0.0};   // 位置与航向(rad)
  double s_progress_m{NAN};          // 若为NaN且 compute_s_internal=true，则内部计算
  double dt{0.05};                   // 控制周期[s]
};

// 控制命令（RBI 产出或上层基线）
struct ControlCmd {
  double throttle{0.0};   // [0,1]
  double brake{0.0};      // [0,1]
  double steer{0.0};      // [-1,1]
  bool   hand_brake{false};
  bool   reverse{false};
  double target_speed_mps{0.0}; // 仅作信息记录
};

// 安全约束
struct SafetyCaps {
  bool   must_emergency_brake{false};
  double max_safe_speed_mps{1e9}; // 当前未用到，留作扩展
};

//速度事件
struct SpeedEvent
{
  std::string name;
  std::string trigger_type;   // "distance_m"

  // 新增：显式窗口
  double start_at_m = std::numeric_limits<double>::quiet_NaN(); // ← 新：开始
  double end_at_m   = std::numeric_limits<double>::quiet_NaN(); // ← 新：结束

  double target = 0.0;
  double slew   = 0.0;
  int    brake_ticks = 0;
  bool   once  = true;

  // 运行时
  bool applied = false;
};

class RandomBehaviorInjector {
public:
  explicit RandomBehaviorInjector(const InjectorConfig& cfg, const carla::client::World& world);
  ~RandomBehaviorInjector();

  // 禁止拷贝&移动（含mutex）
  RandomBehaviorInjector(const RandomBehaviorInjector&) = delete;
  RandomBehaviorInjector& operator=(const RandomBehaviorInjector&) = delete;
  RandomBehaviorInjector(RandomBehaviorInjector&&) = delete;
  RandomBehaviorInjector& operator=(RandomBehaviorInjector&&) = delete;

  // —— 路径 —— //
  void setGlobalPath(const std::shared_ptr<driver_models_types::PathWithOptions>& msg); // 从ROS消息
  void setGlobalPath(const GlobalPath& path);                                           // 直接点列
  void setDefaultSlewRate(double v);

  // —— 事件 —— //
  void setEvents(const std::vector<SpeedEvent>& evs);
  void clearEvents();

  // —— 每帧接口 —— //
  void beginCycle(const State& st_in);

  /**
   * @brief 计算随意驾驶的“覆盖控制”
   * @return std::nullopt 表示窗口外（上层应使用基线控制）；返回值存在表示窗口内（RBI接管）
   */
  std::optional<ControlCmd> stepOverride(const SafetyCaps& caps);

  // —— 兼容旧接口：有些上层还在调用 —— //
  void afterBaseline(const ControlCmd&) {} // 完全解耦：保留空实现
  ControlCmd computeFinal(const ControlCmd& baseline, const SafetyCaps& caps) {
    auto o = stepOverride(caps);
    return o.has_value() ? *o : baseline;
  }

  // —— 调试/查询 —— //
  inline const std::string& roleName() const noexcept { return role_name_; }
  inline double             currentS() const noexcept { return st_.s_progress_m; }
  size_t dbg_next_event_idx() const;
  size_t dbg_events_size()   const;
  double dbg_last_event_start() const;
  double dbg_next_event_at() const;

private:
  // —— 行为树封装 —— //
  struct BTImpl;
  void buildBehaviorTree_();
  void tickBehaviorTree_();

  // —— 供BT节点调用的“公有动作/条件”（但仅模块内部使用） —— //
  bool BT_CondEventDue() const;
  bool BT_ActApplyEvent();
  bool BT_CondInEventWindow() const;
  void BT_ActTrackRefSpeed();
  void BT_ActPassThrough();

  // —— 内部算法 —— //
  double projectS_(double x, double y); // (x,y) -> s
  double computeLateral_();             // 产生横向转向
  bool samplePathAtS_(double s, double& x, double& y,
                      double& tx, double& ty) const;
private:
  // 配置
  InjectorConfig cfg_;
  std::string    role_name_;
  carla::client::World carla_world_;
  // 路径与弧长
  GlobalPath           path_;
  std::vector<double>  path_s_;

  // 投影缓存
  int    proj_nearest_idx_{0};
  double proj_s_prev_{0.0};
  bool   proj_has_prev_xy_{false};
  double proj_last_x_{0.0}, proj_last_y_{0.0};

  // 事件
  std::vector<SpeedEvent> events_;
  size_t  next_event_idx_{0};
  bool    events_activated_{false};
  bool    events_disabled_{false}; // 预留开关

  // 运行态
  State  st_{};
  double v_target_mps_{0.0};   // 事件期望速度
  double v_ref_mps_{0.0};      // RBI内部参考速度（斜坡）
  double v_slew_rate_{0.0};    // 斜坡率
  double last_event_start_m_{-1e18};
  double window_start_m_ = std::numeric_limits<double>::quiet_NaN();
  double window_end_m_   = std::numeric_limits<double>::quiet_NaN();

  // 横向控制状态
  double last_steer_{0.0};

  // 输出
  ControlCmd out_cmd_{};
  bool       override_used_{false}; // 本tick是否由RBI接管

  // 行为树
  std::unique_ptr<BTImpl> bt_;

  // 线程安全
  mutable std::mutex mtx_;
};

} // namespace rbt
