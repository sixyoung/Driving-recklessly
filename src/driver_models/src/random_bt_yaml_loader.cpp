// random_bt_yaml_loader.cpp
#include "driver_models/random_bt_yaml_loader.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <limits>

namespace rbt {

// —— 安全读取小工具（带默认值）——
static double as_double_def(const YAML::Node& n, const char* key, double defv){
  if (!n || !n[key]) return defv;
  try { return n[key].as<double>(); } catch (...) { return defv; }
}
static int as_int_def(const YAML::Node& n, const char* key, int defv){
  if (!n || !n[key]) return defv;
  try { return n[key].as<int>(); } catch (...) { return defv; }
}
static bool as_bool_def(const YAML::Node& n, const char* key, bool defv){
  if (!n || !n[key]) return defv;
  try { return n[key].as<bool>(); } catch (...) { return defv; }
}
static std::string as_str_def(const YAML::Node& n, const char* key, const std::string& defv){
  if (!n || !n[key]) return defv;
  try { return n[key].as<std::string>(); } catch (...) { return defv; }
}

std::vector<SpeedEvent>
load_events_from_yaml(const std::string& path,
                      const std::string& role_name,
                      double fallback_slew,
                      std::optional<double>* out_default_slew,
                      std::optional<double>* out_stop_at_m)
{
  std::vector<SpeedEvent> out;

  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    std::cerr << "[RBI][YAML] LoadFile failed: " << e.what() << " file=" << path << std::endl;
    return out;
  }

  const YAML::Node root = doc["random_bt"];
  if (!root) {
    std::cerr << "[RBI][YAML] key 'random_bt' not found in: " << path << std::endl;
    return out;
  }

  // 读取 default 段
  double default_slew = fallback_slew;
  std::optional<double> stop_at_m;

  if (const YAML::Node def = root["default"]) {
    default_slew = as_double_def(def, "v_slew_rate", default_slew);
    if (def["stop_at_m"]) {
      try { stop_at_m = def["stop_at_m"].as<double>(); } catch (...) {}
    }
  }
  if (out_default_slew) *out_default_slew = default_slew;
  if (out_stop_at_m)    *out_stop_at_m    = stop_at_m;

  // 读取 role 段
  const YAML::Node role = root[role_name];
  if (!role) {
    std::cerr << "[RBI][YAML] role '" << role_name << "' not found under 'random_bt'." << std::endl;
    return out;
  }
  const YAML::Node evs = role["events"];
  if (!evs || !evs.IsSequence()) {
    std::cerr << "[RBI][YAML] '" << role_name << ".events' not found or not a sequence." << std::endl;
    return out;
  }

  // 解析事件（支持 start_at_m / end_at_m，兼容 at_m）
  out.reserve(evs.size());
  for (const YAML::Node& one : evs) {
    SpeedEvent ev;

    ev.name  = as_str_def(one, "name", "");
    const std::string trig = as_str_def(one, "trigger_type", "distance_m");
    if (trig != "distance_m") {
      std::cerr << "[RBI][YAML] unsupported trigger_type '" << trig << "', skip." << std::endl;
      continue;
    }

    // ---- 新：窗口化字段 ----
    bool has_start = false;
    if (one["start_at_m"]) {
      try { ev.start_at_m = one["start_at_m"].as<double>(); has_start = true; } catch (...) {}
    }
    if (!has_start && one["at_m"]) {  // 兼容旧字段
      try { ev.start_at_m = one["at_m"].as<double>(); has_start = true; } catch (...) {}
    }
    if (!has_start) {
      std::cerr << "[RBI][YAML] missing 'start_at_m' (or legacy 'at_m'), skip one event." << std::endl;
      continue;
    }
    // end_at_m 可选，缺省时由 setEvents() 自动补“下一事件开始/stop_at_m”
    if (one["end_at_m"]) {
      try { ev.end_at_m = one["end_at_m"].as<double>(); } catch (...) {
        ev.end_at_m = std::numeric_limits<double>::quiet_NaN();
      }
    } else {
      ev.end_at_m = std::numeric_limits<double>::quiet_NaN();
    }

    // 目标速度必需
    if (!one["target"]) {
      std::cerr << "[RBI][YAML] missing 'target', skip one event." << std::endl;
      continue;
    }
    try { ev.target = one["target"].as<double>(); } catch (...) { continue; }

    // slew：未给则使用 default_slew
    if (one["slew"]) {
      try { ev.slew = one["slew"].as<double>(); } catch (...) { ev.slew = default_slew; }
    } else {
      ev.slew = default_slew;
    }

    ev.brake_ticks = as_int_def(one, "brake_ticks", 0);
    ev.once        = as_bool_def(one, "once", true);
    ev.applied     = false;

    out.emplace_back(std::move(ev));
  }

  // 按“窗口开始”升序
  std::sort(out.begin(), out.end(),
            [](const SpeedEvent& a, const SpeedEvent& b){ return a.start_at_m < b.start_at_m; });

  // === 摘要日志（非 ROS 依赖） ===
  std::cout << "[RBI][YAML] loaded " << out.size()
            << " event(s) for role '" << role_name
            << "' from " << path
            << " (default_slew=" << default_slew << ", stop_at_m=";
  if (stop_at_m) std::cout << *stop_at_m; else std::cout << "none";
  std::cout << ")\n";
  for (const auto& e : out) {
    std::cout << "  - '" << e.name << "' window=[" << e.start_at_m << ", ";
    if (std::isfinite(e.end_at_m)) std::cout << e.end_at_m; else std::cout << "NaN";
    std::cout << ") target=" << e.target << " slew=" << e.slew << " brake=" << e.brake_ticks << "\n";
  }

  return out;
}

} // namespace rbt
