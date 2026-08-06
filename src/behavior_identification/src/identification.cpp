#include <behavior_identification/identification.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior_identification
{


// ===============================
// 运行期缓存状态
// ===============================
// 所有可调阈值都放到 BehaviorIdentificationConfig g_config 中，默认值在头文件，
// 运行时由 config/behavior_identification.yaml 覆盖。
static BehaviorIdentificationConfig g_config;

// 同一辆车只在第一次经过入口停止线时判断一次。
static std::set<uint32_t> g_checked_vehicle_ids;

// 记录上一帧车辆位置。
static std::unordered_map<uint32_t, geometry_msgs::Point> g_last_vehicle_pos;

// 记录车辆第一次被观测到的位置，用于判断车辆是否“出生在路口内部”。
static std::unordered_map<uint32_t, geometry_msgs::Point> g_vehicle_birth_pos;
static std::set<uint32_t> g_vehicle_spawned_in_intersection_ids;

// 场景开始后的全局识别保护时间状态。
static bool g_identification_start_time_initialized = false;
static ros::Time g_identification_start_time;

// 红绿灯消息连续性与有效性状态。
static ros::Time g_last_light_msg_stamp;
static int g_continuous_light_msg_count = 0;
static bool g_light_msg_received_in_current_scene = false;

// 场景类型运行期状态。
static std::string g_scene_type = "unknown";
static bool g_scene_type_loaded = false;
static bool g_scene_type_is_signalized = false;
static std::string g_scene_type_source = "none";

static bool g_signalized_intersection_active = false;
static ros::Time g_last_signal_evidence_stamp;

// 左转不让直行去重状态。
static std::set<std::string> g_lty_reported_pairs;

// 低速识别运行期状态。
static std::unordered_map<uint32_t, ros::Time> g_slow_braking_suppression_until;
static std::set<uint32_t> g_slow_ready_vehicle_ids;
static std::unordered_map<uint32_t, ros::Time> g_slow_speed_begin_stamp;
static std::unordered_map<uint32_t, ros::Time> g_vehicle_first_seen_stamp;

// 历史异常队列面板状态。
static std::vector<std::string> g_abnormal_history_queue;
static std::set<std::string> g_abnormal_history_dedup;
static bool g_panel_anchor_initialized = false;
static double g_panel_anchor_x = 0.0;
static double g_panel_anchor_y = 0.0;
static double g_panel_anchor_z = 8.0;
static ros::Time g_last_panel_publish_time;

static std::vector<FixedStopLine> g_fixed_stop_lines;
static bool g_stop_lines_initialized = false;

static bool IsPointInsideStopLineIntersectionArea(
    const geometry_msgs::Point& p,
    const std::vector<FixedStopLine>& stop_lines)
{
    if (static_cast<int>(stop_lines.size()) < g_config.spawn_intersection_min_stop_lines)
    {
        return false;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& line : stop_lines)
    {
        const double lx = line.has_virtual_line ? line.center_x : line.raw_x;
        const double ly = line.has_virtual_line ? line.center_y : line.raw_y;
        if (!std::isfinite(lx) || !std::isfinite(ly))
        {
            continue;
        }

        min_x = std::min(min_x, lx);
        max_x = std::max(max_x, lx);
        min_y = std::min(min_y, ly);
        max_y = std::max(max_y, ly);
    }

    if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
        !std::isfinite(min_y) || !std::isfinite(max_y))
    {
        return false;
    }

    const double margin = std::max(0.0, g_config.spawn_intersection_bbox_margin);
    return p.x >= min_x - margin && p.x <= max_x + margin &&
           p.y >= min_y - margin && p.y <= max_y + margin;
}

static bool ShouldSkipIdentificationDuringStartup(const ros::Time& now)
{
    if (g_config.global_startup_grace_time <= 0.0)
    {
        return false;
    }

    if (!g_identification_start_time_initialized)
    {
        return true;
    }

    const double elapsed = (now - g_identification_start_time).toSec();
    return elapsed >= 0.0 && elapsed < g_config.global_startup_grace_time;
}

// ===============================
// 工具函数
// ===============================
static std::string ToLower(std::string state)
{
    std::transform(
        state.begin(),
        state.end(),
        state.begin(),
        [](unsigned char c) { return std::tolower(c); }
    );
    return state;
}

static bool IsRedState(std::string state)
{
    state = ToLower(state);
    return state == "red" || state == "r" || state.find("red") != std::string::npos;
}

static bool IsYellowState(std::string state)
{
    state = ToLower(state);
    return state == "yellow" || state == "y" || state.find("yellow") != std::string::npos;
}

static bool IsGreenState(std::string state)
{
    state = ToLower(state);
    return state == "green" || state == "g" || state.find("green") != std::string::npos;
}

static bool IsPassState(std::string state)
{
    return IsGreenState(state) || IsYellowState(state);
}

static bool IsKnownLightState(std::string state)
{
    return IsRedState(state) || IsYellowState(state) || IsGreenState(state);
}

static std::string TrimSceneString(const std::string& input)
{
    const std::string blanks = " \t\r\n\'\"";
    const size_t begin = input.find_first_not_of(blanks);
    if (begin == std::string::npos)
    {
        return "";
    }

    const std::string tail_blanks = " \t\r\n,;}]>\'\"";
    const size_t end = input.find_last_not_of(tail_blanks);
    if (end == std::string::npos || end < begin)
    {
        return "";
    }

    return input.substr(begin, end - begin + 1);
}

static std::string ExtractSceneTypeFromText(const std::string& content)
{
    // 兼容常见格式：
    //   scene_type: 信控路口
    //   "scene_type": "信控路口"
    //   'scene_type': '信控路口'
    //   <scene_type>信控路口</scene_type>
    const std::string open_tag = "<scene_type>";
    const std::string close_tag = "</scene_type>";
    size_t tag_pos = content.find(open_tag);
    if (tag_pos != std::string::npos)
    {
        const size_t value_begin = tag_pos + open_tag.size();
        const size_t value_end = content.find(close_tag, value_begin);
        if (value_end != std::string::npos)
        {
            return TrimSceneString(content.substr(value_begin, value_end - value_begin));
        }
    }

    const size_t key_pos = content.find("scene_type");
    if (key_pos == std::string::npos)
    {
        return "";
    }

    size_t sep_pos = content.find_first_of(":=", key_pos);
    if (sep_pos == std::string::npos)
    {
        return "";
    }

    size_t value_begin = sep_pos + 1;
    while (value_begin < content.size() &&
           (content[value_begin] == ' ' || content[value_begin] == '\t' ||
            content[value_begin] == '\r' || content[value_begin] == '\n'))
    {
        ++value_begin;
    }

    if (value_begin >= content.size())
    {
        return "";
    }

    if (content[value_begin] == '"' || content[value_begin] == '\'')
    {
        const char quote = content[value_begin];
        const size_t value_end = content.find(quote, value_begin + 1);
        if (value_end != std::string::npos)
        {
            return TrimSceneString(content.substr(value_begin + 1, value_end - value_begin - 1));
        }
    }

    size_t value_end = content.find_first_of(",\r\n}];<", value_begin);
    if (value_end == std::string::npos)
    {
        value_end = content.size();
    }

    return TrimSceneString(content.substr(value_begin, value_end - value_begin));
}

static bool IsSignalizedSceneTypeValue(const std::string& scene_type)
{
    const std::string trimmed = TrimSceneString(scene_type);
    const std::string lowered = ToLower(trimmed);
    return lowered == "tl_intersection";
}

static void ApplySceneType(const std::string& scene_type, const std::string& source)
{
    const std::string trimmed = TrimSceneString(scene_type);

    if (trimmed.empty())
    {
        g_scene_type = "unknown";
        g_scene_type_loaded = false;
        g_scene_type_is_signalized = false;
        g_scene_type_source = source;
    }
    else
    {
        g_scene_type = trimmed;
        g_scene_type_loaded = true;
        g_scene_type_is_signalized = IsSignalizedSceneTypeValue(trimmed);
        g_scene_type_source = source;
    }

    ROS_INFO(
        "Scene type updated: scene_type=%s, signalized=%d, source=%s",
        g_scene_type.c_str(),
        static_cast<int>(g_scene_type_is_signalized),
        g_scene_type_source.c_str()
    );
}

static bool LoadSceneTypeFromFile(const std::string& file_path)
{
    if (file_path.empty())
    {
        ApplySceneType("", "empty_scene_file_path");
        return false;
    }

    std::ifstream ifs(file_path.c_str());
    if (!ifs.is_open())
    {
        ROS_WARN("Failed to open scene file: %s. Treat as non-signalized for red-light identification.", file_path.c_str());
        ApplySceneType("", "open_failed:" + file_path);
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string scene_type = ExtractSceneTypeFromText(buffer.str());

    if (scene_type.empty())
    {
        ROS_WARN("scene_type was not found in scene file: %s. Treat as non-signalized for red-light identification.", file_path.c_str());
        ApplySceneType("", "missing_scene_type:" + file_path);
        return false;
    }

    ApplySceneType(scene_type, "scene_file:" + file_path);
    return true;
}

static bool IsRedLightIdentificationAllowedBySceneType()
{
    if (!g_config.use_scene_type_for_signal_control)
    {
        return true;
    }

    return g_scene_type_loaded && g_scene_type_is_signalized;
}

static bool ShouldTreatLineAsSignalControlled(
    bool known_state,
    bool pass_state,
    bool line_seen_pass_state,
    const ros::Time& line_last_pass_stamp,
    const ros::Time& now)
{
    if (!known_state)
    {
        return false;
    }

    // v10：如果启用了 scene_type 门控，那么是否属于信控路口由场景文件决定，
    // 不再要求同一停止线必须先出现过 green/yellow。否则全红初始相位会被误认为无效。
    if (g_config.use_scene_type_for_signal_control)
    {
        return IsRedLightIdentificationAllowedBySceneType();
    }

    if (!g_config.require_signal_evidence_for_red_light)
    {
        return true;
    }

    if (!g_config.red_light_requires_line_seen_pass)
    {
        return g_signalized_intersection_active || pass_state;
    }

    if (pass_state)
    {
        return true;
    }

    if (!line_seen_pass_state || line_last_pass_stamp.isZero())
    {
        return false;
    }

    const double age = (now - line_last_pass_stamp).toSec();
    return age >= 0.0 && age <= g_config.signal_evidence_timeout;
}

static bool GetCorrectedLightInfoForLineId(
    int line_id,
    const std::vector<FixedStopLine>& lines,
    std::string& state,
    ros::Time& stamp,
    bool& has_signal_control)
{
    int source_id = line_id;
    if (g_config.swap_sl01_sl03_light_state)
    {
        if (line_id == 1) source_id = 3;
        else if (line_id == 3) source_id = 1;
    }

    for (const auto& line : lines)
    {
        if (line.id == source_id)
        {
            state = line.cached_state;
            stamp = line.state_stamp;
            has_signal_control = line.has_signal_control;
            return true;
        }
    }

    state = "unknown";
    stamp = ros::Time(0);
    has_signal_control = false;
    return false;
}

static bool IsSameRawStopPoint(double x1, double y1, double x2, double y2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy) < 0.75;
}

static int FindRawStopLineIndex(double x, double y)
{
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < g_fixed_stop_lines.size(); ++i)
    {
        const double dx = x - g_fixed_stop_lines[i].raw_x;
        const double dy = y - g_fixed_stop_lines[i].raw_y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_dist > 1.2)
    {
        return -1;
    }

    return best_idx;
}

static const FixedStopLine* FindStopLineById(int line_id)
{
    for (const auto& line : g_fixed_stop_lines)
    {
        if (line.id == line_id)
        {
            return &line;
        }
    }
    return nullptr;
}

static int CorrectedLightSourceId(int line_id)
{
    if (!g_config.swap_sl01_sl03_light_state)
    {
        return line_id;
    }

    if (line_id == 1) return 3;
    if (line_id == 3) return 1;
    return line_id;
}

static std::string GetCorrectedStateForLineId(int line_id)
{
    const int source_id = CorrectedLightSourceId(line_id);
    const FixedStopLine* source_line = FindStopLineById(source_id);

    if (source_line == nullptr)
    {
        return "unknown";
    }

    return source_line->cached_state;
}

static ros::Time GetCorrectedStateStampForLineId(int line_id)
{
    const int source_id = CorrectedLightSourceId(line_id);
    const FixedStopLine* source_line = FindStopLineById(source_id);

    if (source_line == nullptr)
    {
        return ros::Time(0);
    }

    return source_line->state_stamp;
}

static bool BuildCrossCandidateByVehicleMotion(
    double px,
    double py,
    double cx,
    double cy,
    const FixedStopLine& line,
    CrossCandidate& cand)
{
    const double vx = cx - px;
    const double vy = cy - py;
    const double move_len = std::sqrt(vx * vx + vy * vy);

    if (move_len < 0.05)
    {
        return false;
    }

    // 用车辆真实运动方向作为停止线法向。
    // 这样 stop_location 即便横向偏到对向车道，也只提供“纵向过线位置”。
    const double nx = vx / move_len;
    const double ny = vy / move_len;

    const double prev_s = (px - line.raw_x) * nx + (py - line.raw_y) * ny;
    const double curr_s = (cx - line.raw_x) * nx + (cy - line.raw_y) * ny;

    // 车辆从停止线前方运动到停止线后方。
    // 因为 nx,ny 就是车辆运动方向，所以应该是 prev_s <= 0, curr_s >= 0。
    if (prev_s > 0.0 || curr_s < 0.0)
    {
        return false;
    }

    const double denom = curr_s - prev_s;
    if (denom < 1e-6)
    {
        return false;
    }

    const double ratio = -prev_s / denom;
    if (ratio < 0.0 || ratio > 1.0)
    {
        return false;
    }

    const double cross_x = px + ratio * (cx - px);
    const double cross_y = py + ratio * (cy - py);

    // 停止线方向：车辆运动方向的垂线。
    const double tx = -ny;
    const double ty = nx;

    // raw stop_location 到车辆所在车道停止线中心的横向距离。
    // 这个距离允许比较大，因为你的 raw 点已经偏到道路左侧。
    const double lateral = std::fabs((cross_x - line.raw_x) * tx + (cross_y - line.raw_y) * ty);
    if (lateral > g_config.raw_to_lane_lateral_gate)
    {
        return false;
    }

    cand.valid = true;
    cand.line_id = line.id;
    cand.road_id = line.road_id;

    cand.raw_x = line.raw_x;
    cand.raw_y = line.raw_y;

    cand.center_x = cross_x;
    cand.center_y = cross_y;
    cand.normal_x = nx;
    cand.normal_y = ny;
    cand.tangent_x = tx;
    cand.tangent_y = ty;

    cand.lateral_from_raw = lateral;
    cand.ratio = ratio;
    cand.score = lateral + 0.2 * ratio;

    // 这里只保存当前 line 的原始状态。
    // 是否进行 SL[01]/SL[03] 灯色互换，以及是否是真实信号灯控制，
    // 会在 STChannelIdentify() 中基于 local_stop_lines 快照统一修正，避免无锁读取全局变量。
    cand.state = line.cached_state;
    cand.state_stamp = line.state_stamp;
    cand.source_has_signal_control = line.has_signal_control;

    return true;
}

static void UpdateVirtualStopLineFromCandidate(const CrossCandidate& cand)
{
    for (auto& line : g_fixed_stop_lines)
    {
        if (line.id != cand.line_id)
        {
            continue;
        }

        line.has_virtual_line = true;
        line.center_x = cand.center_x;
        line.center_y = cand.center_y;
        line.normal_x = cand.normal_x;
        line.normal_y = cand.normal_y;
        line.tangent_x = cand.tangent_x;
        line.tangent_y = cand.tangent_y;
        return;
    }
}

static void AddTextMarker(
    visualization_msgs::MarkerArray& marker_array,
    const std::string& ns,
    int id,
    const std::string& text,
    double x,
    double y,
    double z,
    double scale,
    double r,
    double g,
    double b,
    double lifetime)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;

    marker.text = text;
    marker.scale.z = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(lifetime);

    marker_array.markers.push_back(marker);
}

static void UpdateHistoryPanelAnchor()
{
    if (g_panel_anchor_initialized)
    {
        return;
    }

    if (g_fixed_stop_lines.empty())
    {
        if (g_config.use_default_panel_anchor_when_no_stop_lines)
        {
            g_panel_anchor_x = g_config.default_panel_anchor_x;
            g_panel_anchor_y = g_config.default_panel_anchor_y;
            g_panel_anchor_z = g_config.default_panel_anchor_z;
            g_panel_anchor_initialized = true;
        }
        return;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& line : g_fixed_stop_lines)
    {
        min_x = std::min(min_x, line.raw_x);
        max_x = std::max(max_x, line.raw_x);
        min_y = std::min(min_y, line.raw_y);
        max_y = std::max(max_y, line.raw_y);
    }

    // 固定在路口包围盒左上方附近。不是屏幕 HUD，但不会跟随某辆车移动。
    g_panel_anchor_x = min_x - g_config.history_panel_left_offset;
    g_panel_anchor_y = max_y + g_config.history_panel_top_offset;
    g_panel_anchor_z = g_config.history_panel_height;
    g_panel_anchor_initialized = true;
}

static void AddAbnormalHistory(uint32_t veh_id, const std::string& behavior)
{
    std::ostringstream key;
    key << veh_id << "|" << behavior;

    if (g_abnormal_history_dedup.find(key.str()) != g_abnormal_history_dedup.end())
    {
        return;
    }

    g_abnormal_history_dedup.insert(key.str());

    std::ostringstream item;
    item << "ID " << veh_id << ": " << behavior;
    g_abnormal_history_queue.push_back(item.str());

    while (static_cast<int>(g_abnormal_history_queue.size()) > g_config.abnormal_history_max_size)
    {
        g_abnormal_history_queue.erase(g_abnormal_history_queue.begin());
    }
}

static void RemoveAbnormalHistoryByBehavior(const std::string& behavior)
{
    // 删除历史面板里某一类行为，主要用于非信控路口清除旧的 Running Red Light。
    const std::string suffix = "|" + behavior;

    g_abnormal_history_queue.erase(
        std::remove_if(
            g_abnormal_history_queue.begin(),
            g_abnormal_history_queue.end(),
            [&](const std::string& item)
            {
                return item.find(behavior) != std::string::npos;
            }
        ),
        g_abnormal_history_queue.end()
    );

    for (auto it = g_abnormal_history_dedup.begin(); it != g_abnormal_history_dedup.end(); )
    {
        const std::string& key = *it;
        if (key.size() >= suffix.size() &&
            key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
            it = g_abnormal_history_dedup.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

static bool HasRecentPassEvidenceInCachedStopLines(const ros::Time& now)
{
    for (const auto& line : g_fixed_stop_lines)
    {
        if (!line.seen_pass_light_state || line.last_pass_state_stamp.isZero())
        {
            continue;
        }

        const double age = (now - line.last_pass_state_stamp).toSec();
        if (age >= 0.0 && age <= g_config.signal_evidence_timeout)
        {
            return true;
        }
    }

    return false;
}

static void ClearSignalCacheNoLock()
{
    g_fixed_stop_lines.clear();
    g_stop_lines_initialized = false;
    g_signalized_intersection_active = false;
    g_last_signal_evidence_stamp = ros::Time(0);
    // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
}


static std::string BuildTrafficLightInfoPanelText()
{
    const ros::Time now = ros::Time::now();

    std::ostringstream text;
    text << std::fixed << std::setprecision(2);
    text << "Traffic Light Info";

    if (g_config.use_scene_type_for_signal_control)
    {
        text << "\nscene_type: " << (g_scene_type_loaded ? g_scene_type : "unknown");
        text << "\nscene_source: " << g_scene_type_source;
        text << "\nscene_signal: " << (g_scene_type_is_signalized ? "signalized" : "non-signalized/unknown");

        if (!IsRedLightIdentificationAllowedBySceneType())
        {
            text << "\nstatus: unknown";
            text << "\nreason: scene_type is not signalized";
            return text.str();
        }
    }

    if (!g_light_msg_received_in_current_scene)
    {
        text << "\nstatus: unknown";
        text << "\nreason: no /traffic_light/phases msg";
        return text.str();
    }

    const double msg_age = g_last_light_msg_stamp.isZero()
        ? 999999.0
        : (now - g_last_light_msg_stamp).toSec();
    const double allowed_gap = g_config.treat_intermittent_light_msg_as_non_signalized
        ? g_config.strict_traffic_light_msg_timeout
        : g_config.traffic_light_msg_timeout;

    if (msg_age > allowed_gap)
    {
        text << "\nstatus: unknown";
        text << "\nreason: stale/intermittent msg";
        text << "\nlast_msg_age: " << msg_age << "s";
        text << "\nallowed_gap: " << allowed_gap << "s";
        text << "\ncontinuous_msg_count: 0";
        return text.str();
    }

    if (g_config.require_continuous_light_msgs_for_red_light &&
        g_continuous_light_msg_count < g_config.min_continuous_light_msg_count)
    {
        text << "\nstatus: unknown";
        text << "\nreason: msg not continuous enough";
        text << "\nlast_msg_age: " << msg_age << "s";
        text << "\ncontinuous_msg_count: " << g_continuous_light_msg_count
             << "/" << g_config.min_continuous_light_msg_count;
        return text.str();
    }

    if (g_fixed_stop_lines.empty())
    {
        text << "\nstatus: unknown";
        text << "\nreason: no valid stop lines/phases";
        text << "\nlast_msg_age: " << msg_age << "s";
        text << "\ncontinuous_msg_count: " << g_continuous_light_msg_count;
        return text.str();
    }

    text << "\nstatus: " << (g_signalized_intersection_active ? "working" : "unknown/non-working");
    text << "\nlast_msg_age: " << msg_age << "s";
    text << "\ncontinuous_msg_count: " << g_continuous_light_msg_count;
    text << "\nphase_count: " << g_fixed_stop_lines.size();

    int shown = 0;
    for (const auto& line : g_fixed_stop_lines)
    {
        if (shown >= g_config.traffic_light_info_max_lines)
        {
            text << "\n...";
            break;
        }

        const double state_age = line.state_stamp.isZero()
            ? 999999.0
            : (now - line.state_stamp).toSec();

        text << "\nSL[" << line.id << "] road=" << line.road_id
             << " state=" << (line.cached_state.empty() ? "unknown" : line.cached_state)
             << " sig=" << (line.has_signal_control ? "Y" : "N")
             << " pass_seen=" << (line.seen_pass_light_state ? "Y" : "N")
             << " age=" << state_age << "s";
        ++shown;
    }

    return text.str();
}

static void PublishAbnormalHistoryPanel(ros::Publisher& pub, bool force_publish = false)
{
    if (!g_panel_anchor_initialized)
    {
        UpdateHistoryPanelAnchor();
    }

    if (!g_panel_anchor_initialized)
    {
        return;
    }

    const ros::Time now = ros::Time::now();

    if (!force_publish &&
        !g_last_panel_publish_time.isZero() &&
        (now - g_last_panel_publish_time).toSec() < g_config.history_panel_publish_interval)
    {
        return;
    }

    g_last_panel_publish_time = now;

    visualization_msgs::MarkerArray marker_array;

    // 先删除上一帧同 ns/id 的文字，再添加新文字。
    // 对 CARLA 来说，这比单纯 ADD 更不容易出现文字重影。
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = now;
    clear_marker.ns = "abnormal_history_panel";
    clear_marker.id = 900001;
    clear_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(clear_marker);

    visualization_msgs::Marker panel;

    panel.header.frame_id = "map";
    panel.header.stamp = now;
    panel.ns = "abnormal_history_panel";
    panel.id = 900001;
    panel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    panel.action = visualization_msgs::Marker::ADD;

    panel.pose.position.x = g_panel_anchor_x;
    panel.pose.position.y = g_panel_anchor_y;
    panel.pose.position.z = g_panel_anchor_z;
    panel.pose.orientation.w = 1.0;

    std::ostringstream text;
    text << "Abnormal History Queue";

    if (g_abnormal_history_queue.empty())
    {
        text << "\nNo abnormal behavior";
    }
    else
    {
        for (const auto& item : g_abnormal_history_queue)
        {
            text << "\n" << item;
        }
    }

    if (g_config.show_traffic_light_info_on_panel)
    {
        text << "\n\n" << BuildTrafficLightInfoPanelText();
    }

    panel.text = text.str();

    // 稍微放大字体，降低小字号插值造成的糊感。
    panel.scale.z = 1.15;

    panel.color.r = g_config.history_panel_text_r;
    panel.color.g = g_config.history_panel_text_g;
    panel.color.b = g_config.history_panel_text_b;
    panel.color.a = 1.0;

    // 生命周期略短于发布周期，避免多帧文字叠加。
    panel.lifetime = ros::Duration(0.25);

    marker_array.markers.push_back(panel);
    pub.publish(marker_array);
}

// ===============================
// 构造函数与回调
// ===============================
void BehaviorIdentification::LoadConfig(ros::NodeHandle& nh)
{
    nh.param<std::string>("object_array_topic", object_array_topic_, object_array_topic_);
    nh.param<double>("speed_threshold", speed_threshold_, speed_threshold_);

    // 其余可调参数由 config/behavior_identification.yaml 加载。
#define LOAD_CFG_PARAM(name) nh.param(#name, g_config.name, g_config.name)
    LOAD_CFG_PARAM(raw_to_lane_lateral_gate);
    LOAD_CFG_PARAM(virtual_stop_line_half_len);
    LOAD_CFG_PARAM(light_state_cache_timeout);
    LOAD_CFG_PARAM(require_recent_light_msg_for_red_light);
    LOAD_CFG_PARAM(traffic_light_msg_timeout);
    LOAD_CFG_PARAM(treat_intermittent_light_msg_as_non_signalized);
    LOAD_CFG_PARAM(strict_traffic_light_msg_timeout);
    LOAD_CFG_PARAM(require_continuous_light_msgs_for_red_light);
    LOAD_CFG_PARAM(min_continuous_light_msg_count);
    LOAD_CFG_PARAM(clear_stale_signal_cache_when_no_recent_msg);
    LOAD_CFG_PARAM(clear_red_light_history_when_no_signal);
    LOAD_CFG_PARAM(enable_red_light_identification);
    LOAD_CFG_PARAM(enable_yellow_light_identification);
    LOAD_CFG_PARAM(require_signal_evidence_for_red_light);
    LOAD_CFG_PARAM(red_light_requires_line_seen_pass);
    LOAD_CFG_PARAM(hard_require_working_signal_for_red_light);
    LOAD_CFG_PARAM(clear_stop_lines_on_empty_light_msg);
    LOAD_CFG_PARAM(signal_evidence_timeout);
    LOAD_CFG_PARAM(swap_sl01_sl03_light_state);
    LOAD_CFG_PARAM(enable_left_turn_yield_identification);
    LOAD_CFG_PARAM(lty_turn_min_deg);
    LOAD_CFG_PARAM(lty_turn_max_deg);
    LOAD_CFG_PARAM(lty_straight_max_deg);
    LOAD_CFG_PARAM(lty_conflict_dist);
    LOAD_CFG_PARAM(lty_time_gap);
    LOAD_CFG_PARAM(lty_straight_brake_acc);
    LOAD_CFG_PARAM(lty_speed_drop);
    LOAD_CFG_PARAM(lty_min_speed);
    LOAD_CFG_PARAM(lty_dt);
    LOAD_CFG_PARAM(left_turn_use_signed_direction);
    LOAD_CFG_PARAM(left_turn_sign);
    LOAD_CFG_PARAM(enable_speed_identification);
    LOAD_CFG_PARAM(enable_overspeed_identification);
    LOAD_CFG_PARAM(overspeed_threshold);
    LOAD_CFG_PARAM(enable_abnormally_slow_identification);
    LOAD_CFG_PARAM(slow_speed_threshold);
    LOAD_CFG_PARAM(slow_speed_duration);
    LOAD_CFG_PARAM(slow_front_check_distance);
    LOAD_CFG_PARAM(slow_front_lateral_gate);
    LOAD_CFG_PARAM(slow_stop_line_front_distance);
    LOAD_CFG_PARAM(slow_stop_line_lateral_gate);
    LOAD_CFG_PARAM(slow_red_light_wait_stop_line_distance);
    LOAD_CFG_PARAM(slow_red_light_wait_lateral_gate);
    LOAD_CFG_PARAM(slow_ignore_red_light_braking);
    LOAD_CFG_PARAM(slow_red_light_braking_window);
    LOAD_CFG_PARAM(slow_red_light_braking_speed_drop);
    LOAD_CFG_PARAM(slow_red_light_braking_min_start_speed);
    LOAD_CFG_PARAM(slow_red_light_braking_hold_time);
    LOAD_CFG_PARAM(slow_red_light_braking_stop_line_distance);
    LOAD_CFG_PARAM(slow_red_light_braking_lateral_gate);
    LOAD_CFG_PARAM(skip_red_light_for_spawn_in_intersection);
    LOAD_CFG_PARAM(spawn_intersection_bbox_margin);
    LOAD_CFG_PARAM(spawn_intersection_min_stop_lines);
    LOAD_CFG_PARAM(global_startup_grace_time);
    LOAD_CFG_PARAM(slow_startup_grace_time);
    LOAD_CFG_PARAM(slow_min_travel_distance);
    LOAD_CFG_PARAM(slow_moving_evidence_speed);
    LOAD_CFG_PARAM(slow_require_previous_normal_speed);
    LOAD_CFG_PARAM(slow_activation_speed);
    LOAD_CFG_PARAM(slow_require_continuous_low_speed);
    LOAD_CFG_PARAM(slow_continuous_window);
    LOAD_CFG_PARAM(slow_min_pose_count);
    LOAD_CFG_PARAM(slow_ignore_sustained_zero_speed);
    LOAD_CFG_PARAM(slow_zero_speed_epsilon);
    LOAD_CFG_PARAM(slow_zero_speed_duration);
    LOAD_CFG_PARAM(abnormal_history_max_size);
    LOAD_CFG_PARAM(history_panel_left_offset);
    LOAD_CFG_PARAM(history_panel_top_offset);
    LOAD_CFG_PARAM(history_panel_height);
    LOAD_CFG_PARAM(history_panel_text_r);
    LOAD_CFG_PARAM(history_panel_text_g);
    LOAD_CFG_PARAM(history_panel_text_b);
    LOAD_CFG_PARAM(history_panel_publish_interval);
    LOAD_CFG_PARAM(show_traffic_light_info_on_panel);
    LOAD_CFG_PARAM(traffic_light_info_max_lines);
    LOAD_CFG_PARAM(use_default_panel_anchor_when_no_stop_lines);
    LOAD_CFG_PARAM(default_panel_anchor_x);
    LOAD_CFG_PARAM(default_panel_anchor_y);
    LOAD_CFG_PARAM(default_panel_anchor_z);
    LOAD_CFG_PARAM(use_scene_type_for_signal_control);
    LOAD_CFG_PARAM(scene_file_path);
    LOAD_CFG_PARAM(signalized_scene_type_value);
#undef LOAD_CFG_PARAM

    g_config.scene_file_path.clear();

    std::string scene_file_path_param;
    std::string scenario_file_path_param;

    // 1. 优先读取 identification_node 内部参数：
    ros::NodeHandle private_nh("~");

    if (private_nh.getParam("scene_file_path", scene_file_path_param) &&
        !scene_file_path_param.empty())
    {
        g_config.scene_file_path = scene_file_path_param;
    }

    if (private_nh.getParam("scenario_file_path", scenario_file_path_param) &&
        !scenario_file_path_param.empty())
    {
        g_config.scene_file_path = scenario_file_path_param;
    }

    // 2. 兼容你原来 launch 里的全局参数
    if (g_config.scene_file_path.empty())
    {
        if (ros::param::get("/scene_file_path", scene_file_path_param) &&
            !scene_file_path_param.empty())
        {
            g_config.scene_file_path = scene_file_path_param;
        }
    }

    if (g_config.scene_file_path.empty())
    {
        if (ros::param::get("/scenario_file_path", scenario_file_path_param) &&
            !scenario_file_path_param.empty())
        {
            g_config.scene_file_path = scenario_file_path_param;
        }
    }

    ROS_INFO("scenario/scene file path used by behavior_identification: %s",
            g_config.scene_file_path.c_str());

    if (!g_config.scene_file_path.empty())
    {
        LoadSceneTypeFromFile(g_config.scene_file_path);
    }
    else if (g_config.use_scene_type_for_signal_control)
    {
        ApplySceneType("", "no_scene_file_path");
        ROS_WARN(
            "use_scene_type_for_signal_control=true, but scene_file_path/scenario_file_path is empty. "
            "Red-light identification will be disabled until scene_type is provided."
        );
    }
}

BehaviorIdentification::BehaviorIdentification(ros::NodeHandle& nh)
{
    LoadConfig(nh);

    object_array_sub_ = nh.subscribe(
        object_array_topic_,
        10,
        &BehaviorIdentification::vehStateSeqCallback,
        this);

    scenario_status_sub_ = nh.subscribe(
        "/scenario_status",
        10,
        &BehaviorIdentification::scenarioStatusCallback,
        this
    );

    traffic_light_sub_ = nh.subscribe(
        "/traffic_light/phases",
        10,
        &BehaviorIdentification::trafficLightCallback,
        this
    );

    // 保留你原来能在 CARLA 显示的 MarkerArray 话题。
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/carla/debug_marker", 10);
}

void BehaviorIdentification::PublishTextMarker(
    uint32_t veh_id,
    const std::string& behavior_text,
    double center_x,
    double center_y,
    double center_z)
{
    (void)center_x;
    (void)center_y;
    (void)center_z;

    // 不再贴到车辆旁边，统一写入历史异常队列面板。
    AddAbnormalHistory(veh_id, behavior_text);
    PublishAbnormalHistoryPanel(marker_pub_, true);
}

void BehaviorIdentification::vehStateSeqCallback(
    const behavior_identification::VehStateSequenceArray::ConstPtr& msg)
{
    ROS_INFO("The number of vehicles: %lu", msg->vehicles.size());

    const ros::Time now_for_startup = ros::Time::now();
    if (!g_identification_start_time_initialized && !msg->vehicles.empty())
    {
        g_identification_start_time = now_for_startup;
        g_identification_start_time_initialized = true;
        ROS_INFO(
            "Behavior identification global startup grace started. duration=%.2fs",
            g_config.global_startup_grace_time
        );
    }

    std::vector<VehicleState> new_vehicle_states;
    new_vehicle_states.reserve(msg->vehicles.size());

    for (const auto& veh : msg->vehicles)
    {
        VehicleState temp_state;
        temp_state.id = veh.id;

        if (!veh.speed.empty())
        {
            temp_state.speed = veh.speed;
        }
        else
        {
            ROS_WARN("Vehicle ID: %u has an empty speed vector.", veh.id);
        }

        if (!veh.pose.empty())
        {
            temp_state.pose = veh.pose;
        }
        else
        {
            ROS_WARN("Vehicle ID: %u has an empty pose vector.", veh.id);
        }

        if (!veh.accel.empty())
        {
            temp_state.accel = veh.accel;
        }
        else
        {
            ROS_WARN("Vehicle ID: %u has an empty accel vector.", veh.id);
        }

        if (!temp_state.pose.empty() &&
            g_vehicle_birth_pos.find(temp_state.id) == g_vehicle_birth_pos.end())
        {
            g_vehicle_birth_pos[temp_state.id] = temp_state.pose.front().position;
        }

        new_vehicle_states.push_back(temp_state);
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        all_vehicle_states_ = new_vehicle_states;

        bool has_enough_data = false;
        for (const auto& vs : all_vehicle_states_)
        {
            if (vs.pose.size() > 5)
            {
                has_enough_data = true;
                break;
            }
        }

        if (has_enough_data)
        {
            last_valid_states_ = all_vehicle_states_;
        }
    }

    IdentificationOutput();
}

void BehaviorIdentification::trafficLightCallback(
    const road_side_system::TrafficLightPhaseArray::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(light_mutex_);

    current_traffic_lights_ = msg->phases;

    if (!IsRedLightIdentificationAllowedBySceneType())
    {
        ROS_INFO_THROTTLE(
            2.0,
            "Ignore /traffic_light/phases for red-light identification because scene_type=%s is not signalized.",
            g_scene_type.c_str()
        );

        current_traffic_lights_.clear();
        g_light_msg_received_in_current_scene = false;
        g_last_light_msg_stamp = ros::Time(0);
        g_continuous_light_msg_count = 0;
        ClearSignalCacheNoLock();

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    const ros::Time now = ros::Time::now();
    const double msg_interval = g_last_light_msg_stamp.isZero()
        ? 999999.0
        : (now - g_last_light_msg_stamp).toSec();
    const double allowed_light_msg_gap = g_config.treat_intermittent_light_msg_as_non_signalized
        ? g_config.strict_traffic_light_msg_timeout
        : g_config.traffic_light_msg_timeout;

    if (g_last_light_msg_stamp.isZero() || msg_interval > allowed_light_msg_gap)
    {
        g_continuous_light_msg_count = 1;
    }
    else
    {
        ++g_continuous_light_msg_count;
    }

    g_last_light_msg_stamp = now;
    g_light_msg_received_in_current_scene = true;

    if (msg->phases.empty())
    {
        ROS_WARN_THROTTLE(
            2.0,
            "Received empty /traffic_light/phases. Treat current scene as non-signalized; skip red-light identification."
        );

        if (g_config.clear_stop_lines_on_empty_light_msg)
        {
            g_fixed_stop_lines.clear();
            g_stop_lines_initialized = false;
            // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
        }

        g_signalized_intersection_active = false;
        g_last_signal_evidence_stamp = ros::Time(0);
        g_continuous_light_msg_count = 0;

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    bool msg_has_pass_state = false;
    bool msg_has_known_state = false;

    for (const auto& light : msg->phases)
    {
        if (IsKnownLightState(light.state))
        {
            msg_has_known_state = true;
        }

        if (IsPassState(light.state))
        {
            msg_has_pass_state = true;
        }
    }

    // 硬门控：场景名里有“信控路口”并不代表红绿灯真的工作。
    // 只要当前消息没有 green/yellow，并且当前缓存也从未见过 green/yellow，
    // 就认为这是“红绿灯未工作/默认全红”的路口，直接关闭闯红灯识别。
    if (!g_config.use_scene_type_for_signal_control &&
        g_config.hard_require_working_signal_for_red_light &&
        !msg_has_pass_state)
    {
        const bool has_recent_cached_pass = HasRecentPassEvidenceInCachedStopLines(now);

        if (!has_recent_cached_pass)
        {
            ROS_WARN_THROTTLE(
                2.0,
                "Traffic light phases have no green/yellow pass evidence in this scene. "
                "Treat as non-working signal or non-signalized intersection; skip red-light identification."
            );

            if (g_config.clear_stale_signal_cache_when_no_recent_msg)
            {
                ClearSignalCacheNoLock();
            }
            else
            {
                g_signalized_intersection_active = false;
                g_last_signal_evidence_stamp = ros::Time(0);
            }

            if (g_config.clear_red_light_history_when_no_signal)
            {
                RemoveAbnormalHistoryByBehavior("Running Red Light");
                PublishAbnormalHistoryPanel(marker_pub_, true);
            }

            return;
        }
    }

    // v10：优先以场景文件 scene_type 判定是否为信控路口。
    // 如果 scene_type == tl_intersection，只要 /traffic_light/phases 中有 red/yellow/green 任意已知状态，
    // 就认为红绿灯信息有效；不再强制要求先看到 green/yellow。
    if (g_config.use_scene_type_for_signal_control)
    {
        g_signalized_intersection_active = IsRedLightIdentificationAllowedBySceneType() && msg_has_known_state;
        if (g_signalized_intersection_active)
        {
            g_last_signal_evidence_stamp = now;
        }
    }
    else if (msg_has_pass_state)
    {
        g_signalized_intersection_active = true;
        g_last_signal_evidence_stamp = now;
    }
    else if (g_config.require_signal_evidence_for_red_light)
    {
        const double evidence_age = g_last_signal_evidence_stamp.isZero()
            ? 999999.0
            : (now - g_last_signal_evidence_stamp).toSec();

        if (evidence_age > g_config.signal_evidence_timeout)
        {
            g_signalized_intersection_active = false;
        }
    }
    else
    {
        // 如果用户关闭“信号证据门控”，则只要有 red/yellow/green 状态就按信号灯处理。
        g_signalized_intersection_active = msg_has_known_state;
    }

    if (!g_config.use_scene_type_for_signal_control &&
        !g_signalized_intersection_active &&
        g_config.require_signal_evidence_for_red_light)
    {
        ROS_WARN_THROTTLE(
            2.0,
            "Traffic light phases contain no green/yellow evidence. Red-light identification is gated off until pass-state evidence appears."
        );
    }

    // 注意：这里不再“只初始化一次”后就不更新。
    // 因为你的红绿灯会注销/重新出现，所以每次消息都要刷新已有 raw stop point 的灯色。
    for (const auto& light : msg->phases)
    {
        const double sx = light.stop_location.x;
        const double sy = light.stop_location.y;

        if (!std::isfinite(sx) || !std::isfinite(sy))
        {
            continue;
        }

        const bool known_state = IsKnownLightState(light.state);
        const bool pass_state = IsPassState(light.state);

        int idx = FindRawStopLineIndex(sx, sy);

        if (idx < 0)
        {
            FixedStopLine line;
            line.id = static_cast<int>(g_fixed_stop_lines.size());
            line.raw_x = sx;
            line.raw_y = sy;
            line.raw_yaw = light.stop_yaw;
            line.road_id = light.road_id;
            line.cached_state = known_state ? light.state : "unknown";
            line.state_stamp = now;
            line.seen_known_light_state = known_state;
            line.seen_pass_light_state = pass_state;
            line.last_pass_state_stamp = pass_state ? now : ros::Time(0);
            line.has_signal_control = ShouldTreatLineAsSignalControlled(
                known_state,
                pass_state,
                line.seen_pass_light_state,
                line.last_pass_state_stamp,
                now
            );

            g_fixed_stop_lines.push_back(line);

            ROS_INFO(
                "Add raw stop ref SL[%d]: raw=(%.2f, %.2f), yaw=%.2f, road=%d, state=%s, signal_control=%d",
                line.id,
                line.raw_x,
                line.raw_y,
                line.raw_yaw,
                line.road_id,
                line.cached_state.c_str(),
                static_cast<int>(line.has_signal_control)
            );
        }
        else
        {
            g_fixed_stop_lines[idx].raw_yaw = light.stop_yaw;
            g_fixed_stop_lines[idx].road_id = light.road_id;
            g_fixed_stop_lines[idx].cached_state = known_state ? light.state : "unknown";
            g_fixed_stop_lines[idx].state_stamp = now;
            g_fixed_stop_lines[idx].seen_known_light_state =
                g_fixed_stop_lines[idx].seen_known_light_state || known_state;
            g_fixed_stop_lines[idx].seen_pass_light_state =
                g_fixed_stop_lines[idx].seen_pass_light_state || pass_state;
            if (pass_state)
            {
                g_fixed_stop_lines[idx].last_pass_state_stamp = now;
            }

            g_fixed_stop_lines[idx].has_signal_control = ShouldTreatLineAsSignalControlled(
                known_state,
                pass_state,
                g_fixed_stop_lines[idx].seen_pass_light_state,
                g_fixed_stop_lines[idx].last_pass_state_stamp,
                now
            );
        }
    }

    // 这里仍然可以初始化停止线缓存，但真正是否进行闯红灯识别，
    // 要在 STChannelIdentify() 中再次检查 g_signalized_intersection_active 和 line.has_signal_control。
    if (!g_fixed_stop_lines.empty())
    {
        g_stop_lines_initialized = true;
    }

    UpdateHistoryPanelAnchor();
}

// ===============================
// 原有角度工具函数
// ===============================
double BehaviorIdentification::Deg2Rad(double deg)
{
    return deg * M_PI / 180.0;
}

double BehaviorIdentification::Rad2Deg(double rad)
{
    return rad * 180.0 / M_PI;
}

double BehaviorIdentification::NormalizeDeg(double deg)
{
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

double BehaviorIdentification::NormalizeRad(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

double BehaviorIdentification::AngleDiffDeg(double a, double b)
{
    a = NormalizeDeg(a);
    b = NormalizeDeg(b);

    double diff = std::fabs(a - b);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

double BehaviorIdentification::AngleDiffRad(double a, double b)
{
    return std::fabs(NormalizeRad(a - b));
}

double BehaviorIdentification::QuaternionToYaw(const geometry_msgs::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool BehaviorIdentification::IsQuaternionValid(const geometry_msgs::Quaternion& q)
{
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return norm > 0.5;
}

bool BehaviorIdentification::IsTurningTrajectory(const VehicleState& veh)
{
    if (veh.pose.size() < 4) return false;

    size_t n = veh.pose.size();

    if (IsQuaternionValid(veh.pose.front().orientation) &&
        IsQuaternionValid(veh.pose.back().orientation))
    {
        double yaw_start = QuaternionToYaw(veh.pose.front().orientation);
        double yaw_end = QuaternionToYaw(veh.pose.back().orientation);
        double yaw_diff = AngleDiffRad(yaw_start, yaw_end);

        if (yaw_diff > 10.0 * M_PI / 180.0)
        {
            return true;
        }
    }

    if (n >= 6)
    {
        size_t i1 = n / 5;
        size_t i2 = 4 * n / 5;

        if (i1 == 0) i1 = 1;
        if (i2 >= n - 1) i2 = n - 2;

        double vx1 = veh.pose[i1].position.x - veh.pose[0].position.x;
        double vy1 = veh.pose[i1].position.y - veh.pose[0].position.y;

        double vx2 = veh.pose[n - 1].position.x - veh.pose[i2].position.x;
        double vy2 = veh.pose[n - 1].position.y - veh.pose[i2].position.y;

        double d1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
        double d2 = std::sqrt(vx2 * vx2 + vy2 * vy2);

        if (d1 > 0.5 && d2 > 0.5)
        {
            double yaw1 = std::atan2(vy1, vx1);
            double yaw2 = std::atan2(vy2, vx2);

            double diff = AngleDiffRad(yaw1, yaw2);

            if (diff > 10.0 * M_PI / 180.0)
            {
                return true;
            }
        }
    }

    double cumulative_yaw_change = 0.0;
    bool has_last_yaw = false;
    double last_yaw = 0.0;

    for (size_t i = 1; i < n; ++i)
    {
        double dx = veh.pose[i].position.x - veh.pose[i - 1].position.x;
        double dy = veh.pose[i].position.y - veh.pose[i - 1].position.y;
        double ds = std::sqrt(dx * dx + dy * dy);

        if (ds < 0.15) continue;

        double yaw = std::atan2(dy, dx);

        if (has_last_yaw)
        {
            cumulative_yaw_change += AngleDiffRad(yaw, last_yaw);
        }

        last_yaw = yaw;
        has_last_yaw = true;
    }

    return cumulative_yaw_change > 15.0 * M_PI / 180.0;
}


// ===============================
// LTY：左转不让直行识别
// ===============================
template <typename VehicleStateT>
static bool GetTrajectoryYawInfo(const VehicleStateT& veh, double& yaw_start, double& yaw_end, double& signed_diff)
{
    if (veh.pose.size() < 6)
    {
        return false;
    }

    const size_t n = veh.pose.size();
    size_t i1 = std::max<size_t>(1, n / 5);
    size_t i2 = std::min<size_t>(n - 2, 4 * n / 5);

    const double vx1 = veh.pose[i1].position.x - veh.pose[0].position.x;
    const double vy1 = veh.pose[i1].position.y - veh.pose[0].position.y;
    const double vx2 = veh.pose[n - 1].position.x - veh.pose[i2].position.x;
    const double vy2 = veh.pose[n - 1].position.y - veh.pose[i2].position.y;

    const double d1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
    const double d2 = std::sqrt(vx2 * vx2 + vy2 * vy2);

    if (d1 < 0.5 || d2 < 0.5)
    {
        return false;
    }

    yaw_start = std::atan2(vy1, vx1);
    yaw_end = std::atan2(vy2, vx2);
    signed_diff = yaw_end - yaw_start;

    while (signed_diff > M_PI) signed_diff -= 2.0 * M_PI;
    while (signed_diff < -M_PI) signed_diff += 2.0 * M_PI;

    return true;
}

template <typename VehicleStateT>
static bool IsLeftTurnLikeTrajectory(const VehicleStateT& veh)
{
    double yaw_start = 0.0;
    double yaw_end = 0.0;
    double signed_diff = 0.0;

    if (!GetTrajectoryYawInfo(veh, yaw_start, yaw_end, signed_diff))
    {
        return false;
    }

    const double abs_deg = std::fabs(signed_diff) * 180.0 / M_PI;
    if (abs_deg < g_config.lty_turn_min_deg || abs_deg > g_config.lty_turn_max_deg)
    {
        return false;
    }

    // 默认不强制区分左/右转方向，避免 CARLA 地图坐标系方向不同导致漏检。
    // 如果你已经验证了地图坐标系，可在 yaml/launch 中设置：
    // left_turn_use_signed_direction=true, left_turn_sign=1 或 -1。
    if (g_config.left_turn_use_signed_direction)
    {
        return signed_diff * static_cast<double>(g_config.left_turn_sign) > 0.0;
    }

    return true;
}

template <typename VehicleStateT>
static bool IsStraightTrajectoryForLTY(const VehicleStateT& veh)
{
    double yaw_start = 0.0;
    double yaw_end = 0.0;
    double signed_diff = 0.0;

    if (!GetTrajectoryYawInfo(veh, yaw_start, yaw_end, signed_diff))
    {
        return false;
    }

    const double abs_deg = std::fabs(signed_diff) * 180.0 / M_PI;
    return abs_deg <= g_config.lty_straight_max_deg;
}

template <typename VehicleStateT>
static double EstimateLastSpeed(const VehicleStateT& veh)
{
    if (!veh.speed.empty())
    {
        return veh.speed.back();
    }

    if (veh.pose.size() >= 2 && g_config.lty_dt > 1e-4)
    {
        const auto& p0 = veh.pose[veh.pose.size() - 2].position;
        const auto& p1 = veh.pose.back().position;
        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        return std::sqrt(dx * dx + dy * dy) / g_config.lty_dt;
    }

    return 0.0;
}

template <typename VehicleStateT>
static bool StraightVehicleBraked(const VehicleStateT& veh)
{
    if (veh.speed.size() >= 2)
    {
        const double drop = veh.speed.front() - veh.speed.back();
        if (drop > g_config.lty_speed_drop)
        {
            return true;
        }
    }

    if (veh.accel.empty() || veh.pose.size() < 2)
    {
        return false;
    }

    const size_t n = veh.pose.size();
    const auto& p0 = veh.pose[n - 2].position;
    const auto& p1 = veh.pose[n - 1].position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double vlen = std::sqrt(vx * vx + vy * vy);

    if (vlen < 1e-3)
    {
        return false;
    }

    const double dir_x = vx / vlen;
    const double dir_y = vy / vlen;

    for (const auto& acc : veh.accel)
    {
        const double along_acc = acc.linear.x * dir_x + acc.linear.y * dir_y;
        if (along_acc < g_config.lty_straight_brake_acc)
        {
            return true;
        }
    }

    return false;
}

template <typename VehicleStateT>
static bool FindTrajectoryConflict(
    const VehicleStateT& left_veh,
    const VehicleStateT& straight_veh,
    double& best_dist,
    double& best_time_gap,
    size_t& best_left_idx,
    size_t& best_straight_idx)
{
    best_dist = std::numeric_limits<double>::infinity();
    best_time_gap = std::numeric_limits<double>::infinity();
    best_left_idx = 0;
    best_straight_idx = 0;

    if (left_veh.pose.empty() || straight_veh.pose.empty())
    {
        return false;
    }

    for (size_t i = 0; i < left_veh.pose.size(); ++i)
    {
        const auto& lp = left_veh.pose[i].position;
        for (size_t j = 0; j < straight_veh.pose.size(); ++j)
        {
            const auto& sp = straight_veh.pose[j].position;
            const double dx = lp.x - sp.x;
            const double dy = lp.y - sp.y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double time_gap = std::fabs(static_cast<double>(i) - static_cast<double>(j)) * g_config.lty_dt;

            // 先选空间距离最近的点；空间距离接近时，选时间差更小的点。
            if (dist < best_dist ||
                (std::fabs(dist - best_dist) < 0.2 && time_gap < best_time_gap))
            {
                best_dist = dist;
                best_time_gap = time_gap;
                best_left_idx = i;
                best_straight_idx = j;
            }
        }
    }

    return best_dist <= g_config.lty_conflict_dist && best_time_gap <= g_config.lty_time_gap;
}

template <typename VehicleStateContainerT>
static void LeftTurnYieldIdentifyImpl(
    const VehicleStateContainerT& local_states,
    ros::Publisher& marker_pub)
{
    if (!g_config.enable_left_turn_yield_identification || local_states.size() < 2)
    {
        return;
    }

    for (const auto& left_veh : local_states)
    {
        if (!IsLeftTurnLikeTrajectory(left_veh))
        {
            continue;
        }

        const double left_speed = EstimateLastSpeed(left_veh);
        if (left_speed < g_config.lty_min_speed)
        {
            continue;
        }

        for (const auto& straight_veh : local_states)
        {
            if (left_veh.id == straight_veh.id)
            {
                continue;
            }

            if (!IsStraightTrajectoryForLTY(straight_veh))
            {
                continue;
            }

            const double straight_speed = EstimateLastSpeed(straight_veh);
            if (straight_speed < g_config.lty_min_speed)
            {
                continue;
            }

            double best_dist = 0.0;
            double best_time_gap = 0.0;
            size_t best_left_idx = 0;
            size_t best_straight_idx = 0;

            if (!FindTrajectoryConflict(
                    left_veh,
                    straight_veh,
                    best_dist,
                    best_time_gap,
                    best_left_idx,
                    best_straight_idx))
            {
                continue;
            }

            // 左转车先到或几乎同时到达冲突点，且直行车有制动/避让迹象，才判定为“不让直行”。
            const bool left_entered_first =
                best_left_idx <= best_straight_idx + static_cast<size_t>(std::ceil(0.5 / std::max(g_config.lty_dt, 1e-4)));
            const bool straight_braked = StraightVehicleBraked(straight_veh);
            const bool high_risk_time_gap = best_time_gap < 0.8;

            if (!left_entered_first || (!straight_braked && !high_risk_time_gap))
            {
                continue;
            }

            std::ostringstream pair_key;
            pair_key << left_veh.id << "|" << straight_veh.id << "|LTY";
            if (g_lty_reported_pairs.find(pair_key.str()) != g_lty_reported_pairs.end())
            {
                continue;
            }
            g_lty_reported_pairs.insert(pair_key.str());

            ROS_WARN(
                "Vehicle %u LTY violation: Left turn failed to yield to straight vehicle %u. conflict_dist=%.2f, time_gap=%.2f, straight_braked=%d",
                left_veh.id,
                straight_veh.id,
                best_dist,
                best_time_gap,
                static_cast<int>(straight_braked)
            );

            AddAbnormalHistory(left_veh.id, "Left Turn Fail To Yield Straight");
            PublishAbnormalHistoryPanel(marker_pub, true);
        }
    }
}

// ===============================
// ST：闯红灯识别
// ===============================
void BehaviorIdentification::STChannelIdentify()
{
    std::vector<FixedStopLine> local_stop_lines;
    std::vector<VehicleState> local_states;
    bool local_stop_lines_initialized = false;
    bool local_signalized_intersection_active = false;
    bool local_light_msg_received = false;
    ros::Time local_last_light_msg_stamp;
    int local_continuous_light_msg_count = 0;

    {
        std::lock_guard<std::mutex> lock(light_mutex_);
        local_stop_lines = g_fixed_stop_lines;
        local_stop_lines_initialized = g_stop_lines_initialized;
        local_signalized_intersection_active = g_signalized_intersection_active;
        local_light_msg_received = g_light_msg_received_in_current_scene;
        local_last_light_msg_stamp = g_last_light_msg_stamp;
        local_continuous_light_msg_count = g_continuous_light_msg_count;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_states = last_valid_states_;
    }

    if (!g_config.enable_red_light_identification)
    {
        return;
    }

    const ros::Time now = ros::Time::now();

    // v10 最强门控：直接由场景文件 scene_type 控制是否做闯红灯识别。
    // scene_type 不是“tl_intersection”时，不再参考 /traffic_light/phases，也不再使用旧 stop line 缓存。
    if (!IsRedLightIdentificationAllowedBySceneType())
    {
        ROS_INFO_THROTTLE(
            2.0,
            "ST red-light identification skipped by scene_type. scene_type=%s, loaded=%d, signalized=%d.",
            g_scene_type.c_str(),
            static_cast<int>(g_scene_type_loaded),
            static_cast<int>(g_scene_type_is_signalized)
        );

        if (g_config.clear_stale_signal_cache_when_no_recent_msg)
        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            g_fixed_stop_lines.clear();
            g_stop_lines_initialized = false;
            g_signalized_intersection_active = false;
            g_last_signal_evidence_stamp = ros::Time(0);
            g_light_msg_received_in_current_scene = false;
            g_last_light_msg_stamp = ros::Time(0);
            g_continuous_light_msg_count = 0;
        }

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    // 注意：下面这些“无信号灯”门控必须放在 stop line 为空的返回之前。
    // 否则无信控路口没有 stop line 时，旧面板里的 Running Red Light 不会被清掉。

    // 最强门控：当前场景从未收到过 /traffic_light/phases，直接认为是非信控路口。
    // 同时清理可能来自上一场景的旧 stop line 与旧 Running Red Light 历史。
    if (!g_config.use_scene_type_for_signal_control &&
        g_config.require_recent_light_msg_for_red_light &&
        !local_light_msg_received)
    {
        ROS_INFO_THROTTLE(
            2.0,
            "ST red-light identification skipped: current scene has never received /traffic_light/phases. Treat as non-signalized."
        );

        if (g_config.clear_stale_signal_cache_when_no_recent_msg)
        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            g_fixed_stop_lines.clear();
            g_stop_lines_initialized = false;
            g_signalized_intersection_active = false;
            g_last_signal_evidence_stamp = ros::Time(0);
            g_continuous_light_msg_count = 0;
            // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
        }

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    // 更强门控：没有持续收到 /traffic_light/phases，就说明当前场景可能是非信控路口，
    // 不能继续使用上一个场景或旧缓存中的红灯状态。
    if (!g_config.use_scene_type_for_signal_control &&
        g_config.require_recent_light_msg_for_red_light)
    {
        const double msg_age = local_last_light_msg_stamp.isZero()
            ? 999999.0
            : (now - local_last_light_msg_stamp).toSec();
        const double allowed_msg_gap = g_config.treat_intermittent_light_msg_as_non_signalized
            ? g_config.strict_traffic_light_msg_timeout
            : g_config.traffic_light_msg_timeout;

        if (msg_age > allowed_msg_gap)
        {
            ROS_INFO_THROTTLE(
                2.0,
                "ST red-light identification skipped: /traffic_light/phases is not continuous. age=%.2fs, allowed_gap=%.2fs. Treat as non-signalized.",
                msg_age,
                allowed_msg_gap
            );

            if (g_config.clear_stale_signal_cache_when_no_recent_msg)
            {
                std::lock_guard<std::mutex> lock(light_mutex_);
                g_fixed_stop_lines.clear();
                g_stop_lines_initialized = false;
                g_signalized_intersection_active = false;
                g_last_signal_evidence_stamp = ros::Time(0);
                // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
            }

            if (g_config.clear_red_light_history_when_no_signal)
            {
                RemoveAbnormalHistoryByBehavior("Running Red Light");
                PublishAbnormalHistoryPanel(marker_pub_, true);
            }
            return;
        }
    }

    // 如果 /traffic_light/phases 不是连续稳定发布，直接认定为无信控路口。
    // 这样可以防止偶发旧消息或间断红灯状态触发闯红灯误判。
    if (!g_config.use_scene_type_for_signal_control &&
        g_config.require_continuous_light_msgs_for_red_light &&
        local_continuous_light_msg_count < g_config.min_continuous_light_msg_count)
    {
        ROS_INFO_THROTTLE(
            2.0,
            "ST red-light identification skipped: /traffic_light/phases is not continuously available. count=%d/%d. Treat as non-signalized.",
            local_continuous_light_msg_count,
            g_config.min_continuous_light_msg_count
        );

        if (g_config.clear_stale_signal_cache_when_no_recent_msg)
        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            g_fixed_stop_lines.clear();
            g_stop_lines_initialized = false;
            g_signalized_intersection_active = false;
            g_last_signal_evidence_stamp = ros::Time(0);
            g_continuous_light_msg_count = 0;
            // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
        }

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    // 关键修正：当前场景没有真实信号灯证据时，不做闯红灯识别。
    // 这能避免“无红绿灯路口被默认 red，导致所有车都被判闯红灯”。
    if (!g_config.use_scene_type_for_signal_control &&
        g_config.require_signal_evidence_for_red_light &&
        !local_signalized_intersection_active)
    {
        ROS_INFO_THROTTLE(
            2.0,
            "ST red-light identification skipped: no valid signalized-intersection evidence in current scene."
        );

        if (g_config.clear_stale_signal_cache_when_no_recent_msg)
        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            g_fixed_stop_lines.clear();
            g_stop_lines_initialized = false;
            g_signalized_intersection_active = false;
            g_last_signal_evidence_stamp = ros::Time(0);
            g_continuous_light_msg_count = 0;
            // 保留面板锚点，避免无信号时 unknown 信息从 CARLA 画面消失。
        }

        if (g_config.clear_red_light_history_when_no_signal)
        {
            RemoveAbnormalHistoryByBehavior("Running Red Light");
            PublishAbnormalHistoryPanel(marker_pub_, true);
        }
        return;
    }

    if (!local_stop_lines_initialized ||
        local_stop_lines.empty() ||
        local_states.empty())
    {
        return;
    }

    for (const auto& veh : local_states)
    {
        if (veh.pose.empty())
        {
            continue;
        }

        const auto& curr_pose = veh.pose.back();
        const double cx = curr_pose.position.x;
        const double cy = curr_pose.position.y;

        if (g_config.skip_red_light_for_spawn_in_intersection)
        {
            auto birth_it = g_vehicle_birth_pos.find(veh.id);
            if (birth_it != g_vehicle_birth_pos.end() &&
                IsPointInsideStopLineIntersectionArea(birth_it->second, local_stop_lines))
            {
                g_vehicle_spawned_in_intersection_ids.insert(veh.id);
            }

            if (g_vehicle_spawned_in_intersection_ids.find(veh.id) !=
                g_vehicle_spawned_in_intersection_ids.end())
            {
                g_last_vehicle_pos[veh.id] = curr_pose.position;
                ROS_INFO_THROTTLE(
                    2.0,
                    "Vehicle %u spawned inside intersection area. Skip red-light identification for this vehicle.",
                    veh.id
                );
                continue;
            }
        }

        if (g_last_vehicle_pos.find(veh.id) == g_last_vehicle_pos.end())
        {
            g_last_vehicle_pos[veh.id] = curr_pose.position;
            continue;
        }

        // 每辆车只判断第一次入口停止线，避免出路口第二根线误判。
        if (g_checked_vehicle_ids.find(veh.id) != g_checked_vehicle_ids.end())
        {
            g_last_vehicle_pos[veh.id] = curr_pose.position;
            continue;
        }

        const auto& prev_point = g_last_vehicle_pos[veh.id];
        const double px = prev_point.x;
        const double py = prev_point.y;

        std::vector<CrossCandidate> candidates;

        for (const auto& line : local_stop_lines)
        {
            CrossCandidate cand;
            if (!BuildCrossCandidateByVehicleMotion(px, py, cx, cy, line, cand))
            {
                continue;
            }

            std::string corrected_state = "unknown";
            ros::Time corrected_stamp;
            bool source_has_signal_control = false;
            GetCorrectedLightInfoForLineId(
                line.id,
                local_stop_lines,
                corrected_state,
                corrected_stamp,
                source_has_signal_control
            );

            cand.state = corrected_state;
            cand.state_stamp = corrected_stamp;
            cand.source_has_signal_control = source_has_signal_control;

            if (!cand.source_has_signal_control || !IsKnownLightState(cand.state))
            {
                ROS_INFO_THROTTLE(
                    2.0,
                    "Vehicle %u crossed SL[%d], but it is not bound to a valid signal light. Skip ST candidate.",
                    veh.id,
                    line.id
                );
                continue;
            }

            const double age = cand.state_stamp.isZero() ? 999.0 : (now - cand.state_stamp).toSec();
            if (age > g_config.light_state_cache_timeout)
            {
                ROS_WARN(
                    "Vehicle %u crossed SL[%d], but light cache is stale: age=%.2fs. Skip this candidate.",
                    veh.id,
                    line.id,
                    age
                );
                continue;
            }

            candidates.push_back(cand);
        }

        g_last_vehicle_pos[veh.id] = curr_pose.position;

        if (candidates.empty())
        {
            continue;
        }

        // 先按几何距离选最可信候选。
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const CrossCandidate& a, const CrossCandidate& b)
            {
                return a.score < b.score;
            }
        );

        CrossCandidate selected = candidates.front();

        // 如果几何上相近的候选里存在绿/黄灯，优先选择绿/黄灯。
        // 这是为了避免同一入口附近存在左转/直行多个相位时，把正常左转车错判到红灯相位。
        for (const auto& cand : candidates)
        {
            if (cand.score > selected.score + 3.0)
            {
                continue;
            }

            if (IsPassState(cand.state))
            {
                selected = cand;
                break;
            }
        }

        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            UpdateVirtualStopLineFromCandidate(selected);
        }

        g_checked_vehicle_ids.insert(veh.id);

        ROS_INFO(
            "Vehicle %u first crossed virtual SL[%d], road=%d, state=%s, lateral_from_raw=%.2f, ratio=%.2f",
            veh.id,
            selected.line_id,
            selected.road_id,
            selected.state.c_str(),
            selected.lateral_from_raw,
            selected.ratio
        );

        if (selected.source_has_signal_control && IsRedState(selected.state))
        {
            ROS_WARN(
                "Vehicle %u ST violation: Running Red Light. virtual SL[%d], raw=(%.2f, %.2f), virtual_center=(%.2f, %.2f), state=%s",
                veh.id,
                selected.line_id,
                selected.raw_x,
                selected.raw_y,
                selected.center_x,
                selected.center_y,
                selected.state.c_str()
            );

            PublishTextMarker(
                veh.id,
                "Running Red Light",
                curr_pose.position.x,
                curr_pose.position.y,
                curr_pose.position.z
            );
        }
        else if (g_config.enable_yellow_light_identification &&
                 selected.source_has_signal_control &&
                 IsYellowState(selected.state))
        {
            ROS_WARN(
                "Vehicle %u ST violation: Running Yellow Light. virtual SL[%d], raw=(%.2f, %.2f), virtual_center=(%.2f, %.2f), state=%s",
                veh.id,
                selected.line_id,
                selected.raw_x,
                selected.raw_y,
                selected.center_x,
                selected.center_y,
                selected.state.c_str()
            );

            PublishTextMarker(
                veh.id,
                "Running Yellow Light",
                curr_pose.position.x,
                curr_pose.position.y,
                curr_pose.position.z
            );
        }
    }
}

// ===============================
// SL / AT：保留原有行为识别
// ===============================
void BehaviorIdentification::SLChannelIdentify()
{
    double max_lateral_dev = 1.5;

    std::vector<VehicleState> local_states;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_states = last_valid_states_;
    }

    for (const auto& veh : local_states)
    {
        if (veh.pose.size() < 4) continue;

        if (IsTurningTrajectory(veh))
        {
            ROS_INFO("Vehicle %u is turning. Skip normal SL lane-change identification.", veh.id);
            continue;
        }

        double start_x = veh.pose.front().position.x;
        double start_y = veh.pose.front().position.y;
        double end_x = veh.pose.back().position.x;
        double end_y = veh.pose.back().position.y;

        double total_dist = std::sqrt(
            std::pow(end_x - start_x, 2) +
            std::pow(end_y - start_y, 2)
        );

        if (total_dist < 2.0)
        {
            continue;
        }

        double dir_x = end_x - start_x;
        double dir_y = end_y - start_y;
        double dir_len = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        if (dir_len < 0.1) continue;

        double max_abs_l = 0.0;

        for (const auto& p : veh.pose)
        {
            double dx = p.position.x - start_x;
            double dy = p.position.y - start_y;

            double l_val = (dx * dir_y - dy * dir_x) / dir_len;
            max_abs_l = std::max(max_abs_l, std::fabs(l_val));
        }

        if (max_abs_l > max_lateral_dev)
        {
            ROS_WARN(
                "Vehicle %u SL Channel Violation: Swerved out of channel limit (L_dev: %.2fm).",
                veh.id,
                max_abs_l
            );

            PublishTextMarker(
                veh.id,
                "Abnormal Lane Change",
                veh.pose.back().position.x,
                veh.pose.back().position.y,
                veh.pose.back().position.z
            );
        }
    }
}

template <typename VehicleStateT>
static bool GetCurrentHeadingXY(const VehicleStateT& veh, double& hx, double& hy)
{
    if (veh.pose.empty())
    {
        return false;
    }

    // 优先用最近一段真实运动方向作为车辆前方方向。
    // 这样即使车辆 quaternion 不稳定，也能判断前方是否有车。
    if (veh.pose.size() >= 2)
    {
        for (size_t i = veh.pose.size() - 1; i > 0; --i)
        {
            const auto& p0 = veh.pose[i - 1].position;
            const auto& p1 = veh.pose[i].position;
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            const double len = std::sqrt(dx * dx + dy * dy);

            if (len > 0.05)
            {
                hx = dx / len;
                hy = dy / len;
                return true;
            }
        }
    }

    // 如果车辆几乎静止，则退化使用当前姿态 yaw。
    const auto& q = veh.pose.back().orientation;
    const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (q_norm > 0.5)
    {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        hx = std::cos(yaw);
        hy = std::sin(yaw);
        return true;
    }

    return false;
}

template <typename VehicleStateT, typename VehicleStateContainerT>
static bool HasVehicleAheadForSlowCheck(
    const VehicleStateT& ego,
    const VehicleStateContainerT& all_states,
    double front_dist,
    double lateral_gate)
{
    if (ego.pose.empty())
    {
        return false;
    }

    double hx = 1.0;
    double hy = 0.0;
    if (!GetCurrentHeadingXY(ego, hx, hy))
    {
        return false;
    }

    const auto& ep = ego.pose.back().position;
    const double tx = -hy;
    const double ty = hx;

    for (const auto& other : all_states)
    {
        if (other.id == ego.id || other.pose.empty())
        {
            continue;
        }

        const auto& op = other.pose.back().position;
        const double dx = op.x - ep.x;
        const double dy = op.y - ep.y;

        const double longitudinal = dx * hx + dy * hy;
        const double lateral = std::fabs(dx * tx + dy * ty);

        if (longitudinal > 0.0 && longitudinal < front_dist && lateral < lateral_gate)
        {
            return true;
        }
    }

    return false;
}

template <typename VehicleStateT>
static bool IsBeforeStopLineForSlowCheck(
    const VehicleStateT& veh,
    const std::vector<FixedStopLine>& stop_lines,
    double front_dist,
    double lateral_gate)
{
    if (veh.pose.empty() || stop_lines.empty())
    {
        return false;
    }

    double hx = 1.0;
    double hy = 0.0;
    if (!GetCurrentHeadingXY(veh, hx, hy))
    {
        return false;
    }

    const auto& p = veh.pose.back().position;
    const double tx = -hy;
    const double ty = hx;

    for (const auto& line : stop_lines)
    {
        const double lx = line.has_virtual_line ? line.center_x : line.raw_x;
        const double ly = line.has_virtual_line ? line.center_y : line.raw_y;

        const double dx = lx - p.x;
        const double dy = ly - p.y;

        const double longitudinal = dx * hx + dy * hy;
        const double lateral = std::fabs(dx * tx + dy * ty);

        if (longitudinal >= 0.0 && longitudinal <= front_dist && lateral <= lateral_gate)
        {
            return true;
        }
    }

    return false;
}

template <typename VehicleStateT>
static double ComputeTrajectoryTravelDistanceForSlowCheck(const VehicleStateT& veh)
{
    if (veh.pose.size() < 2)
    {
        return 0.0;
    }

    double dist = 0.0;
    for (size_t i = 1; i < veh.pose.size(); ++i)
    {
        const auto& p0 = veh.pose[i - 1].position;
        const auto& p1 = veh.pose[i].position;
        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        dist += std::sqrt(dx * dx + dy * dy);
    }
    return dist;
}

template <typename VehicleStateT>
static double EstimateMaxSpeedInHistoryForSlowCheck(const VehicleStateT& veh)
{
    double max_speed = 0.0;
    for (const auto& v : veh.speed)
    {
        if (std::isfinite(v))
        {
            max_speed = std::max(max_speed, static_cast<double>(v));
        }
    }
    return max_speed;
}

template <typename VehicleStateT>
static bool HasEnoughObservationForSlowCheck(
    const VehicleStateT& veh,
    const ros::Time& now,
    double& observed_time,
    double& travel_dist,
    double& max_speed)
{
    auto it = g_vehicle_first_seen_stamp.find(veh.id);
    if (it == g_vehicle_first_seen_stamp.end())
    {
        g_vehicle_first_seen_stamp[veh.id] = now;
        observed_time = 0.0;
    }
    else
    {
        observed_time = (now - it->second).toSec();
    }

    travel_dist = ComputeTrajectoryTravelDistanceForSlowCheck(veh);
    max_speed = EstimateMaxSpeedInHistoryForSlowCheck(veh);

    const bool enough_pose = static_cast<int>(veh.pose.size()) >= g_config.slow_min_pose_count;
    const bool enough_time = observed_time >= g_config.slow_startup_grace_time;
    const bool has_moved =
        travel_dist >= g_config.slow_min_travel_distance ||
        max_speed >= g_config.slow_moving_evidence_speed;

    return enough_pose && enough_time && has_moved;
}

template <typename VehicleStateT>
static bool HasSustainedZeroSpeedInRecentWindow(
    const VehicleStateT& veh,
    double& window_time,
    double& recent_max_speed)
{
    window_time = 0.0;
    recent_max_speed = 0.0;

    if (veh.speed.empty())
    {
        return false;
    }

    const double dt = std::max(g_config.lty_dt, 1e-3);
    const double zero_window = g_config.slow_zero_speed_duration > 0.0
        ? g_config.slow_zero_speed_duration
        : g_config.slow_speed_duration;
    const int required_count = std::max(1, static_cast<int>(std::ceil(zero_window / dt)));

    if (static_cast<int>(veh.speed.size()) < required_count)
    {
        return false;
    }

    const size_t start_idx = veh.speed.size() - static_cast<size_t>(required_count);
    for (size_t i = start_idx; i < veh.speed.size(); ++i)
    {
        const double v = static_cast<double>(veh.speed[i]);
        if (!std::isfinite(v))
        {
            return false;
        }

        recent_max_speed = std::max(recent_max_speed, std::fabs(v));
        if (std::fabs(v) > g_config.slow_zero_speed_epsilon)
        {
            return false;
        }
    }

    window_time = required_count * dt;
    return true;
}

template <typename VehicleStateT>
static bool HasRecentBrakingForSlowSuppression(
    const VehicleStateT& veh,
    double& window_time,
    double& recent_max_speed,
    double& speed_drop)
{
    window_time = 0.0;
    recent_max_speed = 0.0;
    speed_drop = 0.0;

    if (veh.speed.size() < 2)
    {
        return false;
    }

    const double curr_speed = EstimateLastSpeed(veh);
    if (!std::isfinite(curr_speed) || curr_speed < 0.0 || curr_speed >= g_config.slow_speed_threshold)
    {
        return false;
    }

    const double dt = std::max(g_config.lty_dt, 1e-3);
    const double braking_window = std::max(g_config.slow_red_light_braking_window, dt);
    const int max_count = std::max(2, static_cast<int>(std::ceil(braking_window / dt)));
    const int available_count = static_cast<int>(veh.speed.size());
    const int used_count = std::min(max_count, available_count);
    const size_t start_idx = veh.speed.size() - static_cast<size_t>(used_count);

    for (size_t i = start_idx; i < veh.speed.size(); ++i)
    {
        const double v = static_cast<double>(veh.speed[i]);
        if (!std::isfinite(v))
        {
            continue;
        }
        recent_max_speed = std::max(recent_max_speed, v);
    }

    const double braking_min_start_speed = g_config.slow_red_light_braking_min_start_speed > 0.0
        ? g_config.slow_red_light_braking_min_start_speed
        : g_config.slow_speed_threshold;

    speed_drop = recent_max_speed - curr_speed;
    window_time = used_count * dt;

    return recent_max_speed >= braking_min_start_speed &&
           speed_drop >= g_config.slow_red_light_braking_speed_drop;
}

template <typename VehicleStateT>
static bool HasContinuousLowSpeedInRecentWindow(
    const VehicleStateT& veh,
    double& window_time,
    double& recent_max_speed)
{
    window_time = 0.0;
    recent_max_speed = 0.0;

    if (!g_config.slow_require_continuous_low_speed)
    {
        const double curr_speed = EstimateLastSpeed(veh);
        recent_max_speed = curr_speed;
        return curr_speed >= 0.0 && curr_speed < g_config.slow_speed_threshold;
    }

    if (veh.speed.empty())
    {
        return false;
    }

    const double dt = std::max(g_config.lty_dt, 1e-3);
    const double continuous_window = g_config.slow_continuous_window > 0.0
        ? g_config.slow_continuous_window
        : g_config.slow_speed_duration;
    const int required_count = std::max(1, static_cast<int>(std::ceil(continuous_window / dt)));
    if (static_cast<int>(veh.speed.size()) < required_count)
    {
        return false;
    }

    const size_t start_idx = veh.speed.size() - static_cast<size_t>(required_count);

    for (size_t i = start_idx; i < veh.speed.size(); ++i)
    {
        const double v = static_cast<double>(veh.speed[i]);
        if (!std::isfinite(v))
        {
            return false;
        }

        recent_max_speed = std::max(recent_max_speed, v);

        if (v >= g_config.slow_speed_threshold)
        {
            return false;
        }
    }

    window_time = required_count * dt;
    return true;
}

void BehaviorIdentification::ATChannelIdentify()
{
    double max_accel = 3.0;

    std::vector<VehicleState> local_states;
    std::vector<FixedStopLine> local_stop_lines;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_states = last_valid_states_;
    }

    {
        std::lock_guard<std::mutex> lock(light_mutex_);
        local_stop_lines = g_fixed_stop_lines;
    }

    const ros::Time now = ros::Time::now();

    for (const auto& veh : local_states)
    {
        if (veh.pose.empty()) continue;

        for (const auto& a : veh.accel)
        {
            double a_mag = std::sqrt(a.linear.x * a.linear.x + a.linear.y * a.linear.y);

            if (a_mag > max_accel)
            {
                ROS_WARN(
                    "Vehicle %u AT Channel Violation: Aggressive acceleration (%.2f m/s^2).",
                    veh.id,
                    a_mag
                );

                PublishTextMarker(
                    veh.id,
                    "Aggressive Accel/Decel",
                    veh.pose.back().position.x,
                    veh.pose.back().position.y,
                    veh.pose.back().position.z
                );
                break;
            }
        }

        if (!g_config.enable_speed_identification)
        {
            continue;
        }

        const double curr_speed = EstimateLastSpeed(veh);

        // 超速：当前速度超过阈值即可认定。
        if (g_config.enable_overspeed_identification && curr_speed > g_config.overspeed_threshold)
        {
            ROS_WARN(
                "Vehicle %u Speed Violation: Overspeed. speed=%.2f, threshold=%.2f.",
                veh.id,
                curr_speed,
                g_config.overspeed_threshold
            );

            PublishTextMarker(
                veh.id,
                "Overspeed",
                veh.pose.back().position.x,
                veh.pose.back().position.y,
                veh.pose.back().position.z
            );
        }

        // 异常缓慢：前方无车、不在停止线前，且速度持续低于阈值。
        if (!g_config.enable_abnormally_slow_identification)
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            continue;
        }

        const double slow_activation_speed = g_config.slow_activation_speed > 0.0
            ? g_config.slow_activation_speed
            : g_config.slow_speed_threshold;

        // 关键门控：车辆必须先达到过正常行驶速度，才允许进入“异常缓慢”判定。
        // 这样场景刚开始所有车速度为 0 时，不会被直接判成低速行驶。
        if (curr_speed >= slow_activation_speed)
        {
            g_slow_ready_vehicle_ids.insert(veh.id);
            g_slow_speed_begin_stamp.erase(veh.id);
            continue;
        }

        if (g_config.slow_require_previous_normal_speed &&
            g_slow_ready_vehicle_ids.find(veh.id) == g_slow_ready_vehicle_ids.end())
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            ROS_INFO_THROTTLE(
                2.0,
                "Vehicle %u slow-check skipped: not activated yet. curr_speed=%.2f, activation_speed=%.2f.",
                veh.id,
                curr_speed,
                slow_activation_speed
            );
            continue;
        }

        double slow_observed_time = 0.0;
        double slow_travel_dist = 0.0;
        double slow_max_speed = 0.0;
        const bool slow_observation_ready = HasEnoughObservationForSlowCheck(
            veh,
            now,
            slow_observed_time,
            slow_travel_dist,
            slow_max_speed
        );

        if (!slow_observation_ready)
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            continue;
        }

        // 红灯减速保护：车辆在停止线前从正常速度逐渐降到低速时，
        // 不进入异常低速计时，避免把 4~5 秒的正常制动过程误判为低速行驶。
        if (g_config.slow_ignore_red_light_braking)
        {
            const bool braking_stop_line_ahead = IsBeforeStopLineForSlowCheck(
                veh,
                local_stop_lines,
                g_config.slow_red_light_braking_stop_line_distance,
                g_config.slow_red_light_braking_lateral_gate
            );

            double braking_window_time = 0.0;
            double braking_recent_max_speed = 0.0;
            double braking_speed_drop = 0.0;
            const bool recent_braking = HasRecentBrakingForSlowSuppression(
                veh,
                braking_window_time,
                braking_recent_max_speed,
                braking_speed_drop
            );

            if (braking_stop_line_ahead && recent_braking)
            {
                g_slow_speed_begin_stamp.erase(veh.id);
                g_slow_braking_suppression_until[veh.id] = now + ros::Duration(g_config.slow_red_light_braking_hold_time);
                ROS_INFO_THROTTLE(
                    2.0,
                    "Vehicle %u slow-check skipped: braking before stop line. window=%.2fs, max_speed=%.2f, curr_speed=%.2f, drop=%.2f.",
                    veh.id,
                    braking_window_time,
                    braking_recent_max_speed,
                    curr_speed,
                    braking_speed_drop
                );
                continue;
            }

            auto suppress_it = g_slow_braking_suppression_until.find(veh.id);
            if (suppress_it != g_slow_braking_suppression_until.end())
            {
                if (now <= suppress_it->second && braking_stop_line_ahead)
                {
                    g_slow_speed_begin_stamp.erase(veh.id);
                    ROS_INFO_THROTTLE(
                        2.0,
                        "Vehicle %u slow-check skipped: in braking hold protection before stop line.",
                        veh.id
                    );
                    continue;
                }
                else
                {
                    g_slow_braking_suppression_until.erase(suppress_it);
                }
            }
        }

        double zero_window_time = 0.0;
        double zero_recent_max_speed = 0.0;
        if (g_config.slow_ignore_sustained_zero_speed &&
            HasSustainedZeroSpeedInRecentWindow(veh, zero_window_time, zero_recent_max_speed))
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            ROS_INFO_THROTTLE(
                2.0,
                "Vehicle %u slow-check skipped: sustained zero speed. zero_window=%.2fs, max_abs_speed=%.3f.",
                veh.id,
                zero_window_time,
                zero_recent_max_speed
            );
            continue;
        }

        double continuous_window_time = 0.0;
        double recent_max_speed = 0.0;
        if (g_config.slow_require_continuous_low_speed &&
            !HasContinuousLowSpeedInRecentWindow(veh, continuous_window_time, recent_max_speed))
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            continue;
        }

        const bool has_front_vehicle = HasVehicleAheadForSlowCheck(
            veh,
            local_states,
            g_config.slow_front_check_distance,
            g_config.slow_front_lateral_gate
        );
        const bool before_stop_line = IsBeforeStopLineForSlowCheck(
            veh,
            local_stop_lines,
            g_config.slow_stop_line_front_distance,
            g_config.slow_stop_line_lateral_gate
        );
        const bool waiting_for_signal_stop_line = IsBeforeStopLineForSlowCheck(
            veh,
            local_stop_lines,
            g_config.slow_red_light_wait_stop_line_distance,
            g_config.slow_red_light_wait_lateral_gate
        );

        const bool low_speed_now = std::isfinite(curr_speed) && curr_speed >= 0.0 && curr_speed < g_config.slow_speed_threshold;
        const bool slow_condition_now =
            low_speed_now && !has_front_vehicle && !before_stop_line && !waiting_for_signal_stop_line;

        if (!slow_condition_now)
        {
            g_slow_speed_begin_stamp.erase(veh.id);
            continue;
        }

        auto begin_it = g_slow_speed_begin_stamp.find(veh.id);
        if (begin_it == g_slow_speed_begin_stamp.end())
        {
            g_slow_speed_begin_stamp[veh.id] = now;
            continue;
        }

        const double continuous_low_time = (now - begin_it->second).toSec();
        if (continuous_low_time >= g_config.slow_speed_duration)
        {
            ROS_WARN(
                "Vehicle %u Speed Violation: Abnormally slow. current_speed=%.2f, continuous_time=%.2fs, no front vehicle within %.2fm, not before stop line.",
                veh.id,
                curr_speed,
                continuous_low_time,
                g_config.slow_front_check_distance
            );

            PublishTextMarker(
                veh.id,
                "Abnormally Slow Driving",
                veh.pose.back().position.x,
                veh.pose.back().position.y,
                veh.pose.back().position.z
            );
        }
    }
}

void BehaviorIdentification::IdentificationOutput()
{
    ROS_INFO("=== Starting Behavior Identification ===");

    const ros::Time now = ros::Time::now();
    if (ShouldSkipIdentificationDuringStartup(now))
    {
        const double elapsed = g_identification_start_time_initialized
            ? (now - g_identification_start_time).toSec()
            : 0.0;
        ROS_INFO_THROTTLE(
            1.0,
            "Skip all behavior identification during startup grace. elapsed=%.2fs / %.2fs",
            elapsed,
            g_config.global_startup_grace_time
        );

        {
            std::lock_guard<std::mutex> lock(light_mutex_);
            UpdateHistoryPanelAnchor();
        }
        PublishAbnormalHistoryPanel(marker_pub_);
        ROS_INFO("=== Behavior Identification Skipped During Startup ===");
        return;
    }

    STChannelIdentify();

    std::vector<VehicleState> local_states_for_lty;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_states_for_lty = last_valid_states_;
    }
    LeftTurnYieldIdentifyImpl(local_states_for_lty, marker_pub_);

    SLChannelIdentify();
    ATChannelIdentify();

    {
        std::lock_guard<std::mutex> lock(light_mutex_);
        UpdateHistoryPanelAnchor();
    }
    PublishAbnormalHistoryPanel(marker_pub_);

    ROS_INFO("=== Behavior Identification Finished ===");
}

// ===============================
// 绘图和场景结束
// ===============================
void BehaviorIdentification::DrawPlot(std::vector<VehicleState> history_states)
{
    double dt = 0.02;

    if (history_states.empty()) return;

    namespace plt = matplotlibcpp;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char time_str[64];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm);

    std::string folder_name =
        std::string("/home/bob/文档/备份/demo06/src/behavior_identification/plots/plot_") +
        time_str;

    std::string mkdir_cmd = "mkdir -p " + folder_name;
    system(mkdir_cmd.c_str());

    struct PlotData {
        std::string label;
        std::vector<double> T, S, L, A;
    };

    std::vector<PlotData> plot_data_list;

    for (const auto& veh : history_states)
    {
        PlotData pd;
        pd.label = "Veh " + std::to_string(veh.id);

        double accumulated_s = 0.0;

        for (size_t i = 0; i < veh.pose.size(); ++i)
        {
            pd.T.push_back(i * dt);

            if (i == 0)
            {
                pd.S.push_back(0.0);
            }
            else
            {
                double dx = veh.pose[i].position.x - veh.pose[i - 1].position.x;
                double dy = veh.pose[i].position.y - veh.pose[i - 1].position.y;
                accumulated_s += std::sqrt(dx * dx + dy * dy);
                pd.S.push_back(accumulated_s);
            }

            pd.L.push_back(veh.pose[i].position.y);

            if (i < veh.accel.size())
            {
                // acc plot
                // double ax = veh.accel[i].linear.x;
                // double ay = veh.accel[i].linear.y;
                // pd.A.push_back(std::sqrt(ax * ax + ay * ay));

                // vel plot
                double velocity = veh.speed[i];
                pd.A.push_back(velocity);
            }
            else
            {
                pd.A.push_back(0.0);
            }
        }

        plot_data_list.push_back(pd);
    }

    plt::figure();
    for (const auto& pd : plot_data_list) plt::named_plot(pd.label, pd.T, pd.S);
    plt::title("S-T Graph");
    plt::xlabel("Time (s)");
    plt::ylabel("S (m)");
    plt::legend();
    std::string st_path = folder_name + "/ST_graph.png";
    plt::save(st_path);
    ROS_INFO("Saved ST plot to %s", st_path.c_str());

    plt::figure();
    for (const auto& pd : plot_data_list) plt::named_plot(pd.label, pd.S, pd.L);
    plt::title("S-L Graph");
    plt::xlabel("S (m)");
    plt::ylabel("L (m)");
    plt::legend();
    std::string sl_path = folder_name + "/SL_graph.png";
    plt::save(sl_path);
    ROS_INFO("Saved SL plot to %s", sl_path.c_str());

    plt::figure();
    for (const auto& pd : plot_data_list) plt::named_plot(pd.label, pd.T, pd.A);
    plt::title("A-T Graph");
    plt::xlabel("Time (s)");
    plt::ylabel("Accel (m/s^2)");
    plt::legend();
    std::string at_path = folder_name + "/AT_graph.png";
    plt::save(at_path);
    ROS_INFO("Saved AT plot to %s", at_path.c_str());

    plt::show(false);
}

void BehaviorIdentification::scenarioStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg != nullptr)
    {
        const std::string scene_type_from_msg = ExtractSceneTypeFromText(msg->data);
        if (!scene_type_from_msg.empty())
        {
            ApplySceneType(scene_type_from_msg, "scenario_status_msg");
        }
        else if (g_config.use_scene_type_for_signal_control && !g_config.scene_file_path.empty())
        {
            // 如果批量场景运行时会覆盖同一个场景文件，这里在场景状态变化时重新读取一次。
            LoadSceneTypeFromFile(g_config.scene_file_path);
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!last_valid_states_.empty())
        {
            ROS_INFO("Generating plot with valid cached data...");
            DrawPlot(last_valid_states_);
            last_valid_states_.clear();
        }
    }

    {
        std::lock_guard<std::mutex> lock(light_mutex_);
        g_fixed_stop_lines.clear();
        g_stop_lines_initialized = false;
    }

    g_checked_vehicle_ids.clear();
    g_last_vehicle_pos.clear();
    g_vehicle_birth_pos.clear();
    g_vehicle_spawned_in_intersection_ids.clear();
    g_lty_reported_pairs.clear();
    g_slow_speed_begin_stamp.clear();
    g_vehicle_first_seen_stamp.clear();
    g_slow_ready_vehicle_ids.clear();
    g_slow_braking_suppression_until.clear();
    g_identification_start_time_initialized = false;
    g_identification_start_time = ros::Time(0);
    g_light_msg_received_in_current_scene = false;
    g_signalized_intersection_active = false;
    g_last_signal_evidence_stamp = ros::Time(0);
    g_abnormal_history_queue.clear();
    g_abnormal_history_dedup.clear();
    g_panel_anchor_initialized = false;
    g_last_panel_publish_time = ros::Time(0);
    g_last_light_msg_stamp = ros::Time(0);
    g_continuous_light_msg_count = 0;
    g_signalized_intersection_active = false;
    g_last_signal_evidence_stamp = ros::Time(0);

    ROS_INFO("Scenario finished. Stop-line, vehicle, light-signal and abnormal-history caches cleared.");
}

}  // namespace behavior_identification