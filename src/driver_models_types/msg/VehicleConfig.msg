# VehicleConfig.msg
int32 total_vehicles
string role_name
uint32 carla_id
string driver_model
string scene_type
string scene_class
int32 path_id
int32 road_option # 0: straight, 1: left_turn, 2: right_turn
bool is_random_behavior_vehicle
string spawn_key
geometry_msgs/Pose stop_line_pose
geometry_msgs/Pose exit_intersection_pose
geometry_msgs/PoseStamped spawn_point
geometry_msgs/PoseStamped goal_point
std_msgs/Float64 cruise_speed
driver_models_types/RandomBehavior random_behavior


