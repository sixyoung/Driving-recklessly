#include "driver_models/random_behavior_injector.h"
// 仅在 .cpp 引入重头文件，避免头文件污染
#include <ros/ros.h>
// BehaviorTree.CPP v3
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <iomanip>
#include <limits>

using namespace common; // ros_pose_to_carla_transform

namespace rbt {

// ============================ 构造/析构 ============================

RandomBehaviorInjector::RandomBehaviorInjector(const InjectorConfig& cfg, const carla::client::World& world)
: cfg_(cfg),
  role_name_(cfg.role_name),
  v_slew_rate_(cfg.default_v_slew_rate),
  carla_world_(world){
  v_ref_mps_ = 0.0;
  v_target_mps_ = 0.0;
  buildBehaviorTree_();
}

RandomBehaviorInjector::~RandomBehaviorInjector() = default;

// ============================ 路径 ============================

void RandomBehaviorInjector::setGlobalPath(const std::shared_ptr<driver_models_types::PathWithOptions>& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  path_.clear(); path_s_.clear();
  proj_nearest_idx_ = 0; proj_s_prev_ = 0.0; proj_has_prev_xy_ = false;
  proj_last_x_ = proj_last_y_ = 0.0;
  last_event_start_m_ = -std::numeric_limits<double>::infinity();

  if (!msg) {
    ROS_WARN("[%s][RBI] setGlobalPath: null msg", role_name_.c_str());
    return;
  }

  const auto& wps = msg->waypoints;
  path_.reserve(wps.size());
  auto deg2rad = [](double d){ return d * M_PI / 180.0; };

  for (const auto& w : wps) {
    const auto tf = ros_pose_to_carla_transform(w.pose);
    PathPoint p;
    p.x = static_cast<double>(tf.location.x);
    p.y = static_cast<double>(tf.location.y);
    p.yaw = deg2rad(static_cast<double>(tf.rotation.yaw));
    path_.push_back(p);
  }

  path_s_.assign(path_.size(), 0.0);
  for (size_t i = 1; i < path_.size(); ++i) {
    const double dx = path_[i].x - path_[i-1].x;
    const double dy = path_[i].y - path_[i-1].y;
    path_s_[i] = path_s_[i-1] + std::hypot(dx, dy);
  }

  ROS_INFO("[%s][RBI] GlobalPath set: N=%zu, length=%.1f m",
           role_name_.c_str(), path_.size(),
           path_.empty()?0.0:path_s_.back());
}

void RandomBehaviorInjector::setDefaultSlewRate(double v)
{
  std::lock_guard<std::mutex> lk(mtx_);
  v_slew_rate_ = std::max(0.0, v);
}

// ============================ 事件 ============================

void RandomBehaviorInjector::setEvents(const std::vector<SpeedEvent>& evs)
{
  std::lock_guard<std::mutex> lk(mtx_);
  events_ = evs;

  // 按 start 排序
  std::sort(events_.begin(), events_.end(),
           [](const SpeedEvent& a, const SpeedEvent& b){ return a.start_at_m < b.start_at_m; });
  for (auto& e : events_) e.applied = false;

  // 为未提供 end 的事件补上默认结束：下一事件的 start 或者 events_stop_at_m_
  for (size_t i = 0; i < events_.size(); ++i) {
    auto& e = events_[i];
    if (!std::isfinite(e.end_at_m)) {
      double next_start = std::numeric_limits<double>::infinity();
      if (i + 1 < events_.size()) {
        next_start = std::min(next_start, events_[i+1].start_at_m);
      }
      e.end_at_m = next_start;
    }
    // 修正非法窗口：end <= start
    if (!(e.end_at_m > e.start_at_m)) {
      ROS_WARN("[%s][RBI] Fix invalid window: '%s' end_at_m(%.2f) <= start_at_m(%.2f). Expanding by +0.5m.",
               role_name_.c_str(), e.name.c_str(), e.end_at_m, e.start_at_m);
      e.end_at_m = e.start_at_m + 0.5;
    }
  }

  // 运行时状态
  next_event_idx_        = 0;
  events_activated_      = false;
  events_disabled_       = false;
  v_target_mps_          = 0.0;
  v_ref_mps_             = 0.0;
  v_slew_rate_           = std::max(0.0, cfg_.default_v_slew_rate);
  last_event_start_m_    = std::numeric_limits<double>::quiet_NaN();
  window_start_m_        = std::numeric_limits<double>::quiet_NaN();  // ← 新
  window_end_m_          = std::numeric_limits<double>::quiet_NaN();  // ← 新

  if (!bt_) { buildBehaviorTree_(); }
}


void RandomBehaviorInjector::clearEvents()
{
  std::lock_guard<std::mutex> lk(mtx_);
  events_.clear();
  next_event_idx_     = 0;
  events_activated_   = false;
  events_disabled_    = false;
  last_event_start_m_ = -std::numeric_limits<double>::infinity();
  v_target_mps_       = 0.0;
  v_ref_mps_          = 0.0;
  window_start_m_ = std::numeric_limits<double>::quiet_NaN();
  window_end_m_   = std::numeric_limits<double>::quiet_NaN();
  ROS_INFO("[%s][RBI] ClearEvents", role_name_.c_str());
}

// ============================ 每帧流程 ============================

void RandomBehaviorInjector::beginCycle(const State& st_in)
{
  std::lock_guard<std::mutex> lk(mtx_);
  st_ = st_in;

  if (cfg_.compute_s_internal || !std::isfinite(st_in.s_progress_m)) {
    st_.s_progress_m = projectS_(st_.x, st_.y);

    // 关键状态节流打印（0.5s）
    double next_at = std::numeric_limits<double>::infinity();
    if (next_event_idx_ < events_.size()) next_at = events_[next_event_idx_].start_at_m;

    const bool in_window = BT_CondInEventWindow(); // 只读，无副作用
    ROS_INFO_STREAM_THROTTLE(0.5,
      "[" << role_name_ << "][RBI] s=" << std::fixed << std::setprecision(2) << st_.s_progress_m
      << " m, window=[" 
      << (std::isfinite(window_start_m_) ? std::to_string(window_start_m_) : std::string("NaN"))
      << "," 
      << (std::isfinite(window_end_m_)   ? std::to_string(window_end_m_)   : std::string("NaN"))
      << "), in_window=" << int(BT_CondInEventWindow())
      << ", v_ref=" << v_ref_mps_
      << ", v_tgt=" << v_target_mps_
      << ", next_start=" 
      << (next_event_idx_ < events_.size() ? std::to_string(events_[next_event_idx_].start_at_m) : "NONE")
      << ", idx=" << next_event_idx_ << "/" << events_.size());
  }

  // 标志清零：本tick尚未接管
  override_used_ = false;
}

// 完全解耦：事件窗口内 -> 返回RBI控制；窗口外 -> std::nullopt
std::optional<ControlCmd> RandomBehaviorInjector::stepOverride(const SafetyCaps& caps)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (caps.must_emergency_brake) {
    ControlCmd e{};
    e.throttle = 0.0; e.brake = 1.0; e.steer = 0.0;
    e.target_speed_mps = 0.0; e.hand_brake = false; e.reverse = false;
    return e;
  }

  // 执行一次行为树
  tickBehaviorTree_();

  if (!override_used_) {
    return std::nullopt; // 窗口外：由上层发布基线控制
  }
  return out_cmd_;        // 窗口内：RBI全量控制
}

// ============================ 调试getter ============================
size_t RandomBehaviorInjector::dbg_next_event_idx() const { return next_event_idx_; }
size_t RandomBehaviorInjector::dbg_events_size()   const { return events_.size(); }
double RandomBehaviorInjector::dbg_last_event_start() const { return last_event_start_m_; }
double RandomBehaviorInjector::dbg_next_event_at() const {
  return (next_event_idx_ < events_.size()) ? events_[next_event_idx_].start_at_m
                                            : std::numeric_limits<double>::infinity();
}

// ============================ s 投影 ============================

double RandomBehaviorInjector::projectS_(double x, double y)
{
  const int N = static_cast<int>(path_.size());
  if (N < 2) {
    proj_last_x_ = x; proj_last_y_ = y; proj_has_prev_xy_ = true;
    return proj_s_prev_;
  }

  const int W = 15; // 局部窗口
  proj_nearest_idx_ = std::clamp(proj_nearest_idx_, 0, N-2);
  const int i_min = std::max(0,      proj_nearest_idx_ - W);
  const int i_max = std::min(N - 2,  proj_nearest_idx_ + W);

  double best_d2 = 1e18;
  int    best_i  = proj_nearest_idx_;
  double best_t  = 0.0;

  for (int i = i_min; i <= i_max; ++i) {
    const double Ax = path_[i].x,     Ay = path_[i].y;
    const double Bx = path_[i+1].x,   By = path_[i+1].y;
    const double vx = Bx - Ax,        vy = By - Ay;
    const double wx = x  - Ax,        wy = y  - Ay;
    const double vv = vx*vx + vy*vy;
    double t = (vv > 1e-12) ? ((wx*vx + wy*vy) / vv) : 0.0;
    t = std::clamp(t, 0.0, 1.0);

    const double px = Ax + t*vx, py = Ay + t*vy;
    const double dx = x - px,    dy = y - py;
    const double d2 = dx*dx + dy*dy;

    if (d2 < best_d2) { best_d2 = d2; best_i = i; best_t = t; }
  }

  const double s_geo = path_s_[best_i] + best_t * (path_s_[best_i+1] - path_s_[best_i]);

  // 限制 s 跳变（向前限幅 + 允许小回退）
  double step = 0.0;
  if (proj_has_prev_xy_) {
    step = std::hypot(x - proj_last_x_, y - proj_last_y_);
  }
  const double max_adv = step * cfg_.hop_gain;

  double s_now = s_geo;
  s_now = std::max(proj_s_prev_ - cfg_.back_tolerance_m, s_now);          // 不明显回退
  s_now = std::min(s_now, proj_s_prev_ + std::max(0.0, max_adv));         // 不猛跳

  proj_nearest_idx_ = (best_t > 0.5) ? (best_i + 1) : best_i;
  proj_s_prev_ = s_now;
  proj_last_x_ = x; proj_last_y_ = y; proj_has_prev_xy_ = true;

  return s_now;
}

// ============================ 行为树封装 ============================

struct RandomBehaviorInjector::BTImpl {
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  BTImpl() = default;
};

void RandomBehaviorInjector::buildBehaviorTree_()
{
  bt_ = std::make_unique<BTImpl>();

  // 注册节点类
  struct CtxGetter {
    static RandomBehaviorInjector* self(BT::TreeNode& node) {
      RandomBehaviorInjector* s = nullptr;
      node.config().blackboard->get("self", s);
      return s;
    }
  };

  // —— 条件：是否到达下一个事件的触发里程 —— //
  class CondEventDue : public BT::ConditionNode {
  public:
    CondEventDue(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto* s = CtxGetter::self(*this);
      if (!s) return BT::NodeStatus::FAILURE;
      const bool due = s->BT_CondEventDue();

      if(due){
          // 选择要显示的位置：车顶上方（按你的成员改：若无 egoLocation()，用 egoTransform().location）
          carla::geom::Location p {s->st_.x, s->st_.y, 2.5f};
          p.z += 2.5f; // 抬高到车顶上方，避免被车体遮挡

          std::ostringstream oss;
          oss.setf(std::ios::fixed); oss.precision(1);
          oss << "Event due  idx " << s->dbg_next_event_idx()+1
              << "/" << s->dbg_events_size()
              << "   s=" << s->currentS();

          // 注意：CARLA C++ 颜色是 BGRA 顺序；{0,255,255} ≈ 黄
          s->carla_world_.MakeDebugHelper().DrawString(
            p, oss.str(), /*draw_shadow=*/true,
            carla::client::DebugHelper::Color{0u, 255u, 255u},
            /*life_time=*/60.0f, /*persistent_lines=*/false);
      }

      ROS_INFO_THROTTLE(0.2, "[%s][RBI][BT] CondEventDue: s=%.2f -> %s (idx=%zu/%zu)",
                        s->roleName().c_str(), s->currentS(), due?"YES":"NO",
                        s->dbg_next_event_idx(), s->dbg_events_size());
      return due ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  // —— 动作：应用事件（更新目标速度/斜率/紧急制动Tick等） —— //
  class ActApplyEvent : public BT::SyncActionNode {
  public:
    ActApplyEvent(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto* s = CtxGetter::self(*this);
      if (!s) return BT::NodeStatus::FAILURE;
      bool ok = s->BT_ActApplyEvent();
      ROS_INFO_THROTTLE(0.2, "[%s][RBI][BT] ActApplyEvent: %s (idx=%zu/%zu, s=%.2f)",
                        s->roleName().c_str(), ok?"SUCCESS":"FAIL",
                        s->dbg_next_event_idx(), s->dbg_events_size(), s->currentS());
      return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  // —— 条件：处于事件窗口（“上一事件触发点 ~ 下一事件触发点”开区间） —— //
  class CondInEventWindow : public BT::ConditionNode {
  public:
    CondInEventWindow(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto* s = CtxGetter::self(*this);
      if (!s) return BT::NodeStatus::FAILURE;
      const bool in = s->BT_CondInEventWindow();
      ROS_INFO_THROTTLE(0.2, "[%s][RBI][BT] CondInWindow: %s (s=%.2f, start=%.2f, idx=%zu/%zu)",
                        s->roleName().c_str(), in?"YES":"NO", s->currentS(),
                        s->dbg_last_event_start(), s->dbg_next_event_idx(), s->dbg_events_size());
      return in ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  // —— 动作：跟踪参考速度（纵向），并产出横向转向 —— //
  class ActTrackRefSpeed : public BT::SyncActionNode {
  public:
    ActTrackRefSpeed(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto* s = CtxGetter::self(*this);
      if (!s) return BT::NodeStatus::FAILURE;
      s->BT_ActTrackRefSpeed();
      return BT::NodeStatus::SUCCESS;
    }
  };

  // —— 动作：透传（窗口外，不接管） —— //
  class ActPassThrough : public BT::SyncActionNode {
  public:
    ActPassThrough(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto* s = CtxGetter::self(*this);
      if (!s) return BT::NodeStatus::FAILURE;
      s->BT_ActPassThrough();
      return BT::NodeStatus::SUCCESS;
    }
  };

  bt_->factory.registerNodeType<CondEventDue>("CondEventDue");
  bt_->factory.registerNodeType<ActApplyEvent>("ActApplyEvent");
  bt_->factory.registerNodeType<CondInEventWindow>("CondInEventWindow");
  bt_->factory.registerNodeType<ActTrackRefSpeed>("ActTrackRefSpeed");
  bt_->factory.registerNodeType<ActPassThrough>("ActPassThrough");

  static const char* kXML = R"(
  <root main_tree_to_execute="Main">
    <BehaviorTree ID="Main">
      <Fallback name="Root">
        <Sequence name="WindowFromDue">
          <CondEventDue/>
          <ActApplyEvent/>
          <ActTrackRefSpeed/>
        </Sequence>
        <Sequence name="WindowActive">
          <CondInEventWindow/>
          <ActTrackRefSpeed/>
        </Sequence>
        <ActPassThrough/>
      </Fallback>
    </BehaviorTree>
  </root>
  )";

  auto bb = BT::Blackboard::create();
  bb->set<RandomBehaviorInjector*>("self", this);
  bt_->tree = bt_->factory.createTreeFromText(kXML, bb);
}

void RandomBehaviorInjector::tickBehaviorTree_()
{
  (void)bt_->tree.rootNode()->executeTick();
}

// ============================ BT_* 实现 ============================

bool RandomBehaviorInjector::BT_CondEventDue() const
{
  if (events_disabled_ || next_event_idx_ >= events_.size()) return false;
  const auto& ev = events_[next_event_idx_];
  return (st_.s_progress_m + 1e-6 >= ev.start_at_m);
}

bool RandomBehaviorInjector::BT_ActApplyEvent()
{
  if (events_disabled_ || next_event_idx_ >= events_.size()) return false;
  auto& ev = events_[next_event_idx_];
  if (st_.s_progress_m + 1e-6 < ev.start_at_m) return false;

  // 从当前速度启动参考，避免v_ref从0“慢启动”
  v_ref_mps_ = std::max(0.0, st_.speed_mps);
  v_target_mps_ = std::max(0.0, ev.target);
  if (ev.slew > 0.0) v_slew_rate_ = ev.slew;

  // 如配置了“强制刹车Tick”
  if (ev.brake_ticks > 0) {
    // 在TrackRef里倒计时；此处仅记录
  }

  ev.applied          = true;
  events_activated_   = true;
  last_event_start_m_ = ev.start_at_m;      // 兼容保留
  window_start_m_     = ev.start_at_m;      // ← 新
  window_end_m_       = ev.end_at_m;        // ← 新
  ++next_event_idx_;

  ROS_INFO("[RBI][%s] ApplyEvent: at=%.1f target=%.2f slew=%.2f brake=%d",
           role_name_.c_str(), ev.start_at_m, ev.target, ev.slew, ev.brake_ticks);
  return true;
}

bool RandomBehaviorInjector::BT_CondInEventWindow() const
{
  if (events_disabled_ || !events_activated_ || !std::isfinite(window_start_m_) || !std::isfinite(window_end_m_)) return false;

  const double x = st_.s_progress_m;
  const bool in = (x + 1e-6 >= window_start_m_) && (x < window_end_m_ - 1e-6);
  return in;
}

void RandomBehaviorInjector::BT_ActTrackRefSpeed()
{
  // —— 纵向：v_ref 斜坡 —— //
  const double v_cap_target = std::max(0.0, v_target_mps_);
  const double dv_max       = std::max(0.0, v_slew_rate_) * std::max(0.0, st_.dt);
  const double dv           = std::clamp(v_cap_target - v_ref_mps_, -dv_max, dv_max);
  v_ref_mps_ += dv;

  // 输出结构重新填充（完全不依赖基线）
  out_cmd_ = {};
  out_cmd_.target_speed_mps = v_cap_target;

  // 刹车Tick优先（如果你需要，请在事件里设置 brake_ticks 并在此管理倒计时）
  static int brake_ticks_left = 0;
  if (brake_ticks_left > 0) {
    out_cmd_.throttle = 0.0;
    out_cmd_.brake    = 1.0;
    --brake_ticks_left;
  } else {
    const double err  = v_ref_mps_ - st_.speed_mps;
    const double dead = std::max(0.0, cfg_.speed_deadband);
    if (err > dead) { // 需要加速
      out_cmd_.throttle = std::clamp(cfg_.kp_throttle * (err - dead), 0.0, 1.0);
      out_cmd_.brake    = 0.0;
    } else if (err < -dead) { // 需要减速
      out_cmd_.throttle = 0.0;
      out_cmd_.brake    = std::clamp(cfg_.kp_brake * (-err - dead), 0.0, 1.0);
    } else { // 小误差带
      out_cmd_.throttle = 0.0;
      out_cmd_.brake    = 0.0;
    }
  }

  // —— 横向：简单P控制 + 斜坡限幅 —— //
  out_cmd_.steer      = computeLateral_();
  out_cmd_.hand_brake = false;
  out_cmd_.reverse    = false;

  // 标记“本tick由RBI接管”
  override_used_ = true;
}

void RandomBehaviorInjector::BT_ActPassThrough()
{
  // 窗口外：本tick不接管
  override_used_ = false;
}

// ============================ 横向控制 ============================

// 【新增】在弧长 s 处采样路径点与切向
bool RandomBehaviorInjector::samplePathAtS_(double s,
                                            double& x, double& y,
                                            double& tx, double& ty) const
{
  const int N = static_cast<int>(path_.size());
  if (N < 2) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] samplePathAtS_: path too short (N=%d)", role_name_.c_str(), N); // 【新增日志】
    return false;
  }
  if (s <= path_s_.front()) {
    const auto& A = path_[0];
    const auto& B = path_[1];
    x = A.x; y = A.y; tx = B.x - A.x; ty = B.y - A.y;
  } else if (s >= path_s_.back()) {
    const auto& A = path_[N-2];
    const auto& B = path_[N-1];
    x = B.x; y = B.y; tx = B.x - A.x; ty = B.y - A.y;
  } else {
    auto it = std::upper_bound(path_s_.begin(), path_s_.end(), s);
    int i = static_cast<int>(std::distance(path_s_.begin(), it)) - 1;
    i = std::max(0, std::min(i, N-2));
    const double sA = path_s_[i];
    const double sB = path_s_[i+1];
    const double seg = std::max(1e-12, sB - sA);
    const double t = (s - sA) / seg;
    const auto& A = path_[i];
    const auto& B = path_[i+1];
    x = A.x + t * (B.x - A.x);
    y = A.y + t * (B.y - A.y);
    tx = (B.x - A.x);
    ty = (B.y - A.y);
  }

  const double seg_len = std::hypot(tx, ty);
  if (seg_len < 1e-6) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] samplePathAtS_: segment too short (|t|=%.3g) at s=%.2f",
                      role_name_.c_str(), seg_len, s); // 【新增日志】
    return false; // 【防御】
  }
  return true;
}

// 【修改】采用前视点与速度相关增益
double RandomBehaviorInjector::computeLateral_()
{
  const int N = static_cast<int>(path_.size());
  if (N < 2) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] path size <2, steer=0", role_name_.c_str()); // 【新增日志】
    return 0.0;
  }

  // ---- 前视距离 ----
  const double v   = std::max(0.0, st_.speed_mps);
  const double L_v = cfg_.la_min_m + cfg_.la_k_v * v;
  const double L   = std::min(cfg_.la_max_m, std::max(cfg_.la_min_m, L_v));

  // ---- 采样前视点 ----
  const double s_ego  = st_.s_progress_m;
  const double s_look = s_ego + L;
  double px=0, py=0, tx=0, ty=0;
  if (!samplePathAtS_(s_look, px, py, tx, ty)) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] samplePathAtS_ failed (s_ego=%.2f, L=%.2f, s_look=%.2f). steer=0",
                      role_name_.c_str(), s_ego, L, s_look); // 【新增日志】
    return 0.0;
  }

  const double seg_len = std::hypot(tx, ty);
  if (seg_len < 1e-6) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] seg_len too small after sample (%.3g), steer=0",
                      role_name_.c_str(), seg_len); // 【防御+日志】
    return 0.0;
  }

  // ---- 航向误差 ----
  const double yaw_path = std::atan2(ty, tx);
  auto wrap = [](double a){ while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; };
  const double yaw_err = wrap(yaw_path - st_.yaw);

  // ---- 横向误差（左法向）----
  const double nx = -ty, ny = tx;
  const double ex = st_.x - px, ey = st_.y - py;
  const double lat_err = -1.0*(nx*ex + ny*ey) / std::hypot(nx, ny);

  // ---- 线性合成 ----
  double steer_cmd_raw = cfg_.k_yaw * yaw_err + cfg_.k_lat * lat_err;

  // ---- 斜坡限幅 ----
  double dt = st_.dt;
  if (!(dt > 0.0 && std::isfinite(dt))) {
    ROS_WARN_THROTTLE(1.0, "[%s][RBI][LAT] invalid dt=%.3g, fallback to 0.05",
                      role_name_.c_str(), dt); // 【新增日志+防御】
    dt = 0.1;
  }
  const double ds_max = std::max(0.0, cfg_.steer_slew_rate) * dt;
  const double ds     = std::clamp(steer_cmd_raw - last_steer_, -ds_max, ds_max);
  last_steer_ += ds;

  // ---- 饱和 ----
  const double lim = std::max(1e-6, cfg_.steer_limit);
  const double steer_out = std::clamp(steer_cmd_raw, -lim, lim);

  // 【新增日志】关键量可 0.2s 打一次
  ROS_INFO_THROTTLE(0.2,
    "[%s][RBI][LAT] v=%.2f L=%.2f s_look=%.2f |px,py|=(%.2f,%.2f) "
    "seg_len=%.2f yaw_err=%.3f lat_err=%.3f "
    "f cmd_raw=%.3f "
    "dt=%.3f ds_max=%.3f ds=%.3f last_steer=%.3f lim=%.2f out=%.3f",
    role_name_.c_str(), v, L, s_look, px, py,
    seg_len, yaw_err, lat_err,
    steer_cmd_raw,
    dt, ds_max, ds, last_steer_, lim, steer_out);

  // 【参数健康检查】偶发打印一次
  static bool printed_once = false;
  if (!printed_once) {
    printed_once = true;
    ROS_INFO("[%s][RBI][LAT] params: k_yaw=%.3f k_lat=%.3f yaw_decay=%.3f lat_decay=%.3f "
             "steer_slew=%.3f steer_limit=%.3f la_min=%.2f la_max=%.2f la_k_v=%.2f",
             role_name_.c_str(),
             cfg_.k_yaw, cfg_.k_lat, cfg_.yaw_gain_decay, cfg_.lat_gain_decay,
             cfg_.steer_slew_rate, cfg_.steer_limit, cfg_.la_min_m, cfg_.la_max_m, cfg_.la_k_v);
  }

  return steer_out;
}


} // namespace rbt
