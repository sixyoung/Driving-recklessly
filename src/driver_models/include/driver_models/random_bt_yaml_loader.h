// random_bt_yaml_loader.h
#pragma once
// =====================================================
// 从 YAML 读取随意驾驶事件（不依赖 ROS 参数服务器）
//
// 约定 YAML 结构：
// random_bt:
//   default:
//     v_slew_rate: 2.0        # [可选] 全局默认速度坡率 m/s^2
//     stop_at_m:   120.0      # [可选] s>=该值后不再处理事件
//   <role_name>:
//     events:
//       - name: "speedup_50"
//         trigger_type: "distance_m"  # [可选] 仅支持里程触发；缺省视为 distance_m
//         at_m:   50.0
//         target: 12.0
//         slew:   1.8                 # [可选] 未给则用 default.v_slew_rate
//         brake_ticks: 0              # [可选]
//         once: true                  # [可选]
//
// 用法：
//   std::optional<double> def_slew, stop_at;
//   auto evs = rbt::load_events_from_yaml(path, role_name, 1.5, &def_slew, &stop_at);
//   if (def_slew) injector_cfg.default_v_slew_rate = *def_slew;
//   injector.setEvents(evs);
//   if (stop_at) injector.setEventsStopAt(*stop_at);
// =====================================================

#include <string>
#include <vector>
#include <optional>

#include "driver_models/random_behavior_injector.h"  // rbt::SpeedEvent / 等

namespace rbt {

// 读取事件：
// - path: YAML 文件路径
// - role_name: 车辆名（在 YAML 中的 key）
// - fallback_slew: 当 default.v_slew_rate 未配置时用此默认（如 1.5）
// - out_default_slew: 若提供且 YAML 配置了 default.v_slew_rate，则回填
// - out_stop_at_m:    若提供且 YAML 配置了 default.stop_at_m，则回填
std::vector<SpeedEvent>
load_events_from_yaml(const std::string& path,
                      const std::string& role_name,
                      double fallback_slew = 1.5,
                      std::optional<double>* out_default_slew = nullptr,
                      std::optional<double>* out_stop_at_m    = nullptr);

} // namespace rbt
