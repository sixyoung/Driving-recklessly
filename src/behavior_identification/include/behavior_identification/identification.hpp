#ifndef BEHAVIOR_IDENTIFICATION_IDENTIFICATION_HPP_
#define BEHAVIOR_IDENTIFICATION_IDENTIFICATION_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <derived_object_msgs/ObjectArray.h>
#include <road_side_system/TrafficLightPhaseArray.h>
#include <road_side_system/TrafficLightPhase.h>
#include <behavior_identification/VehStateSequenceArray.h>
#include <std_msgs/String.h>
#include <matplotlibcpp.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace behavior_identification
{

struct BehaviorIdentificationConfig
{
    double raw_to_lane_lateral_gate = 12.0;
    double virtual_stop_line_half_len = 3.0;
    double light_state_cache_timeout = 30.0;
    bool require_recent_light_msg_for_red_light = true;
    double traffic_light_msg_timeout = 1.0;
    bool treat_intermittent_light_msg_as_non_signalized = true;
    double strict_traffic_light_msg_timeout = 0.30;
    bool require_continuous_light_msgs_for_red_light = true;
    int min_continuous_light_msg_count = 3;
    bool clear_stale_signal_cache_when_no_recent_msg = true;
    bool clear_red_light_history_when_no_signal = true;
    bool enable_red_light_identification = true;
    bool enable_yellow_light_identification = true;
    bool require_signal_evidence_for_red_light = true;
    bool red_light_requires_line_seen_pass = true;
    bool hard_require_working_signal_for_red_light = true;
    bool clear_stop_lines_on_empty_light_msg = true;
    double signal_evidence_timeout = 120.0;
    bool swap_sl01_sl03_light_state = true;
    bool enable_left_turn_yield_identification = true;
    double lty_turn_min_deg = 35.0;
    double lty_turn_max_deg = 145.0;
    double lty_straight_max_deg = 20.0;
    double lty_conflict_dist = 3.5;
    double lty_time_gap = 1.5;
    double lty_straight_brake_acc = -1.5;
    double lty_speed_drop = 1.5;
    double lty_min_speed = 0.5;
    double lty_dt = 0.02;
    bool left_turn_use_signed_direction = false;
    int left_turn_sign = 1;
    bool enable_speed_identification = true;
    bool enable_overspeed_identification = true;
    double overspeed_threshold = 15.0;
    bool enable_abnormally_slow_identification = true;
    double slow_speed_threshold = 6.0;
    double slow_speed_duration = 2.0;
    double slow_front_check_distance = 15.0;
    double slow_front_lateral_gate = 3.0;
    double slow_stop_line_front_distance = 8.0;
    double slow_stop_line_lateral_gate = 4.0;
    double slow_red_light_wait_stop_line_distance = 45.0;
    double slow_red_light_wait_lateral_gate = 8.0;
    bool slow_ignore_red_light_braking = true;
    double slow_red_light_braking_window = 5.0;
    double slow_red_light_braking_speed_drop = 1.5;
    double slow_red_light_braking_min_start_speed = -1.0;
    double slow_red_light_braking_hold_time = 6.0;
    double slow_red_light_braking_stop_line_distance = 60.0;
    double slow_red_light_braking_lateral_gate = 10.0;
    bool skip_red_light_for_spawn_in_intersection = true;
    double spawn_intersection_bbox_margin = 4.0;
    int spawn_intersection_min_stop_lines = 2;
    double global_startup_grace_time = 5.0;
    double slow_startup_grace_time = 3.0;
    double slow_min_travel_distance = 2.0;
    double slow_moving_evidence_speed = 1.0;
    bool slow_require_previous_normal_speed = true;
    double slow_activation_speed = -1.0;
    bool slow_require_continuous_low_speed = true;
    double slow_continuous_window = -1.0;
    int slow_min_pose_count = 10;
    bool slow_ignore_sustained_zero_speed = true;
    double slow_zero_speed_epsilon = 0.05;
    double slow_zero_speed_duration = -1.0;
    int abnormal_history_max_size = 12;
    double history_panel_left_offset = 15.0;
    double history_panel_top_offset = 30.0;
    double history_panel_height = 8.0;
    double history_panel_text_r = 0.0;
    double history_panel_text_g = 0.0;
    double history_panel_text_b = 0.0;
    double history_panel_publish_interval = 0.20;
    bool show_traffic_light_info_on_panel = true;
    int traffic_light_info_max_lines = 8;
    bool use_default_panel_anchor_when_no_stop_lines = true;
    double default_panel_anchor_x = 0.0;
    double default_panel_anchor_y = 0.0;
    double default_panel_anchor_z = 8.0;
    bool use_scene_type_for_signal_control = true;
    std::string scene_file_path = std::string("");
    std::string signalized_scene_type_value = std::string("tl_intersection");
};

struct FixedStopLine
{
    int id = -1;

    // 原始 stop_location：只作为红绿灯状态绑定点，不再作为最终触发点。
    double raw_x = 0.0;
    double raw_y = 0.0;
    double raw_yaw = 0.0;

    int road_id = -1;
    std::string cached_state = "unknown";
    ros::Time state_stamp;

    // 是否有足够证据表明这条停止线来自真实红绿灯控制。
    bool has_signal_control = false;
    bool seen_known_light_state = false;
    bool seen_pass_light_state = false;
    ros::Time last_pass_state_stamp;

    // 由车辆轨迹和原始 stop_location 动态修正得到的虚拟停止线。
    bool has_virtual_line = false;
    double center_x = 0.0;
    double center_y = 0.0;
    double normal_x = 1.0;
    double normal_y = 0.0;
    double tangent_x = 0.0;
    double tangent_y = 1.0;
};

struct CrossCandidate
{
    bool valid = false;
    int line_id = -1;
    int road_id = -1;

    double raw_x = 0.0;
    double raw_y = 0.0;

    double center_x = 0.0;
    double center_y = 0.0;
    double normal_x = 1.0;
    double normal_y = 0.0;
    double tangent_x = 0.0;
    double tangent_y = 1.0;

    double lateral_from_raw = 0.0;
    double ratio = 0.0;
    double score = 0.0;

    std::string state = "unknown";
    ros::Time state_stamp;
    bool source_has_signal_control = false;
};

class BehaviorIdentification
{
private:
    struct VehicleState
    {
        uint32_t id = 0;
        std::vector<geometry_msgs::Pose> pose;
        std::vector<geometry_msgs::Twist> twist;
        std::vector<geometry_msgs::Accel> accel;
        std::vector<double> speed;
    };

public:
    explicit BehaviorIdentification(ros::NodeHandle& nh);

    void vehStateSeqCallback(const behavior_identification::VehStateSequenceArray::ConstPtr& msg);
    void scenarioStatusCallback(const std_msgs::String::ConstPtr& msg);
    void trafficLightCallback(const road_side_system::TrafficLightPhaseArray::ConstPtr& msg);

    void STChannelIdentify();
    void SLChannelIdentify();
    void ATChannelIdentify();
    void IdentificationOutput();

    void DrawPlot(std::vector<VehicleState> history_states);
    void PublishTextMarker(uint32_t veh_id,
                           const std::string& behavior_text,
                           double center_x,
                           double center_y,
                           double center_z);

private:
    void LoadConfig(ros::NodeHandle& nh);

    double Deg2Rad(double deg);
    double Rad2Deg(double rad);
    double NormalizeDeg(double deg);
    double NormalizeRad(double rad);
    double AngleDiffDeg(double a, double b);
    double AngleDiffRad(double a, double b);

    double QuaternionToYaw(const geometry_msgs::Quaternion& q);
    bool IsQuaternionValid(const geometry_msgs::Quaternion& q);

    bool MatchTrafficLightDirection(double prev_x,
                                    double prev_y,
                                    double curr_x,
                                    double curr_y,
                                    double stop_yaw_deg,
                                    double& matched_yaw_deg,
                                    double& heading_diff_deg);

    void ProjectToStopLineFrame(double x,
                                double y,
                                double stop_x,
                                double stop_y,
                                double matched_yaw_deg,
                                double& longitudinal,
                                double& lateral);

    bool CrossedStopLineByProjection(double prev_x,
                                     double prev_y,
                                     double curr_x,
                                     double curr_y,
                                     double stop_x,
                                     double stop_y,
                                     double matched_yaw_deg,
                                     double& curr_longitudinal,
                                     double& curr_lateral);

    bool CrossedLightPointAlongMotion(double prev_x,
                                      double prev_y,
                                      double curr_x,
                                      double curr_y,
                                      double light_x,
                                      double light_y);

    bool IsTurningTrajectory(const VehicleState& veh);

private:
    ros::Subscriber object_array_sub_;
    ros::Subscriber scenario_status_sub_;
    ros::Subscriber traffic_light_sub_;
    ros::Publisher marker_pub_;

    std::string object_array_topic_ = "/veh_state_sequences";
    double speed_threshold_ = 0.0;

    mutable std::mutex state_mutex_;
    derived_object_msgs::ObjectArray latest_object_array_;
    std::vector<VehicleState> all_vehicle_states_;
    std::vector<VehicleState> last_valid_states_;

    std::vector<road_side_system::TrafficLightPhase> current_traffic_lights_;
    mutable std::mutex light_mutex_;
};

}  // namespace behavior_identification

#endif  // BEHAVIOR_IDENTIFICATION_IDENTIFICATION_HPP_
 