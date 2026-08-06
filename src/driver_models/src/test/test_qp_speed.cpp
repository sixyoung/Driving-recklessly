#include <ros/ros.h>
#include "em_planner.h"
#include "trajectoryPoint.h"
#include "path_data.h"
#include "speed_data.h"
#include "reference_line.h"
#include "speed_limit_decider.h"
#include "path_time_heuristic_optimizer.h"
#include "speed_decider/speed_decider.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>  
namespace fs = std::filesystem;

std::vector<Obstacle> AllObstacle; 
Param_Configs Config_;

void ComputeInitFrenetState(const ReferencePoint &matched_point,
                            const TrajectoryPoint &cartesian_state,
                            std::array<double, 3> *ptr_s,
                            std::array<double, 3> *ptr_d)
{
CartesianFrenetConverter::cartesian_to_frenet(
    matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
    matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
    cartesian_state.path_point().x, cartesian_state.path_point().y,
    cartesian_state.v, cartesian_state.a,
    cartesian_state.path_point().theta,
    cartesian_state.path_point().kappa, ptr_s, ptr_d);
}

// ====== 适配函数，将 InitialConditions 转成 TrajectoryPoint ======
void CopyFrom(TrajectoryPoint &traj, const InitialConditions &Initial) {
  traj.CopyFrom(Initial);
  traj.x = Initial.x_init;
  traj.y = Initial.y_init;
  traj.z = Initial.z_init;
  traj.theta = Initial.theta_init;
  traj.kappa = Initial.kappa_init;
  traj.dkappa = Initial.dkappa_init;
  traj.v = Initial.v_init;
  traj.a = Initial.a_init;
  traj.relative_time = Initial.init_relative_time;
  traj.s = Initial.s0;

  traj.d = Initial.d0;
  traj.d_d = Initial.dd0;
  traj.d_dd = Initial.ddd0;
  traj.s_d = Initial.ds0;
  traj.s_dd = Initial.dds0;
  traj.s_ddd = 0;
  traj.d_ddd = 0;
}

void createTestObstacles(Obstacle_avoid& oba, std::vector<Obstacle>& AllObstacle) {
    AllObstacle.clear();

    // ====== 障碍物 1：矩形车 ======
    {
        planner::Obstacle obs(Config_);

        obs.timestamp_ =  ros::Time::now();

        obs.centerpoint.position.x = 40.0;
        obs.centerpoint.position.y = 0.0;
        obs.centerpoint.position.z = 0.0;

        // 长和宽（假设）
        obs.obstacle_length = 4.5;
        obs.obstacle_width = 2.0;
        obs.obstacle_height = 1.5;

        obs.obstacle_id = "0";
        obs.obstacle_type = 1;
        obs.obstacle_shape = 2;  // 矩形

        obs.obstacle_threa = 0 * M_PI / 180.0; // 朝向 (yaw)
        obs.obstacle_velocity = 1;
        obs.obstacle_acc = 0.0;

        if (obs.obstacle_velocity > 0.1) // 动态障碍物
        {
            oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                    Config_.dynamic_obs_predict_time, obs.obstacle_threa, 
                                    obs.obstacle_velocity, obs.obstacle_acc);
            for (const auto& pt : obs.trajectory_prediction.trajectory_point()) {
                std::cout << "t=" << pt.relative_time
                        << "  x=" << pt.x
                        << "  y=" << pt.y
                        << "  theta=" << pt.theta
                        << "  v=" << pt.v
                        << "  a=" << pt.a
                        << "  s=" << pt.s
                        << "  d=" << pt.d
                        << std::endl;
            }
        }

        PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        PPoint left_front;
        PPoint left_buttom;
        PPoint right_front;
        PPoint right_buttom;

        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, center, obs.obstacle_threa, left_front, left_buttom,
                                        right_buttom, right_front);

        std::vector<PPoint> corners = {left_front, right_front, right_buttom, left_buttom};                                
        if (obs.obstacle_shape == 2) 
        {
            // 顶点
            obs.pinnacle.poses.clear();
            for (size_t j = 0; j < corners.size(); j++)
            {
                geometry_msgs::Pose sds;
                sds.position.x = corners[j].x;
                sds.position.y = corners[j].y;
                sds.position.z = 0; // 压缩二维
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(corners[j].x, corners[j].y));
            }
            // 半径为0
            obs.obstacle_radius = 0;

        }
        AllObstacle.push_back(obs);
    }
    // ====== 障碍物 1：矩形车 ======
    {
        planner::Obstacle obs(Config_);

        obs.timestamp_ =  ros::Time::now();

        obs.centerpoint.position.x = 15.0;
        obs.centerpoint.position.y = 0.0;
        obs.centerpoint.position.z = 0.0;

        // 长和宽（假设）
        obs.obstacle_length = 2.0;
        obs.obstacle_width = 2.0;
        obs.obstacle_height = 1.5;

        obs.obstacle_id = "1";
        obs.obstacle_type = 1;   // 假设1=车辆
        obs.obstacle_shape = 2;  // 矩形

        obs.obstacle_threa = 0.0; // 朝向 (yaw)
        obs.obstacle_velocity = 0.0;
        obs.obstacle_acc = 0.0;

        if (obs.obstacle_velocity > 0.1) // 动态障碍物
        {
            oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                    Config_.dynamic_obs_predict_time, obs.obstacle_threa, 
                                    obs.obstacle_velocity, obs.obstacle_acc);
        }

        PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        PPoint left_front;
        PPoint left_buttom;
        PPoint right_front;
        PPoint right_buttom;

        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, center, obs.obstacle_threa, left_front, left_buttom,
                                        right_buttom, right_front);

        std::vector<PPoint> corners = {left_front, right_front, right_buttom, left_buttom};                                
        if (obs.obstacle_shape == 2) 
        {
            // 顶点
            obs.pinnacle.poses.clear();
            for (size_t j = 0; j < corners.size(); j++)
            {
                geometry_msgs::Pose sds;
                sds.position.x = corners[j].x;
                sds.position.y = corners[j].y;
                sds.position.z = 0; // 压缩二维
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(corners[j].x, corners[j].y));
            }
            // 半径为0
            obs.obstacle_radius = 0;

        }
        obs.start_time = 0.0;
        obs.end_time = 3;
        AllObstacle.push_back(obs);
    }

    // ====== 障碍物 1：矩形车 ======
    {
        planner::Obstacle obs(Config_);

        obs.timestamp_ =  ros::Time::now();

        obs.centerpoint.position.x = 15.0;
        obs.centerpoint.position.y = 0.0;
        obs.centerpoint.position.z = 0.0;

        // 长和宽（假设）
        obs.obstacle_length = 2.0;
        obs.obstacle_width = 2.0;
        obs.obstacle_height = 1.5;

        obs.obstacle_id = "2";
        obs.obstacle_type = 1;   // 假设1=车辆
        obs.obstacle_shape = 2;  // 矩形

        obs.obstacle_threa = 0.0; // 朝向 (yaw)
        obs.obstacle_velocity = 0.0;
        obs.obstacle_acc = 0.0;

        if (obs.obstacle_velocity > 0.1) // 动态障碍物
        {
            oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                    Config_.dynamic_obs_predict_time, obs.obstacle_threa, 
                                    obs.obstacle_velocity, obs.obstacle_acc);
        }

        PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        PPoint left_front;
        PPoint left_buttom;
        PPoint right_front;
        PPoint right_buttom;

        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, center, obs.obstacle_threa, left_front, left_buttom,
                                        right_buttom, right_front);

        std::vector<PPoint> corners = {left_front, right_front, right_buttom, left_buttom};                                
        if (obs.obstacle_shape == 2) 
        {
            // 顶点
            obs.pinnacle.poses.clear();
            for (size_t j = 0; j < corners.size(); j++)
            {
                geometry_msgs::Pose sds;
                sds.position.x = corners[j].x;
                sds.position.y = corners[j].y;
                sds.position.z = 0; // 压缩二维
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(corners[j].x, corners[j].y));
            }
            // 半径为0
            obs.obstacle_radius = 0;

        }
        obs.start_time = 5.0;
        obs.end_time = 8.0;
        AllObstacle.push_back(obs);
    }    
    // ====== 障碍物 1：矩形车 ======
    {
        planner::Obstacle obs(Config_);

        obs.timestamp_ =  ros::Time::now();

        obs.centerpoint.position.x = 20.0;
        obs.centerpoint.position.y = 0.0;
        obs.centerpoint.position.z = 0.0;

        // 长和宽（假设）
        obs.obstacle_length = 4.5;
        obs.obstacle_width = 2.0;
        obs.obstacle_height = 1.5;

        obs.obstacle_id = "3";
        obs.obstacle_type = 1;   // 假设1=车辆
        obs.obstacle_shape = 2;  // 矩形

        obs.obstacle_threa = 0.0; // 朝向 (yaw)
        obs.obstacle_velocity = 0.0;
        obs.obstacle_acc = 0.0;

        if (obs.obstacle_velocity > 0.1) // 动态障碍物
        {
            oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                    Config_.dynamic_obs_predict_time, obs.obstacle_threa, 
                                    obs.obstacle_velocity, obs.obstacle_acc);
        }

        PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        PPoint left_front;
        PPoint left_buttom;
        PPoint right_front;
        PPoint right_buttom;

        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, center, obs.obstacle_threa, left_front, left_buttom,
                                        right_buttom, right_front);

        std::vector<PPoint> corners = {left_front, right_front, right_buttom, left_buttom};                                
        if (obs.obstacle_shape == 2) 
        {
            // 顶点
            obs.pinnacle.poses.clear();
            for (size_t j = 0; j < corners.size(); j++)
            {
                geometry_msgs::Pose sds;
                sds.position.x = corners[j].x;
                sds.position.y = corners[j].y;
                sds.position.z = 0; // 压缩二维
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(corners[j].x, corners[j].y));
            }
            // 半径为0
            obs.obstacle_radius = 0;

        }
        obs.start_time = 0.0;
        obs.end_time = 2.8;
        AllObstacle.push_back(obs);
    }
    {
        planner::Obstacle obs(Config_);

        obs.timestamp_ =  ros::Time::now();

        obs.centerpoint.position.x = 20.0;
        obs.centerpoint.position.y = 0.0;
        obs.centerpoint.position.z = 0.0;

        // 长和宽（假设）
        obs.obstacle_length = 4.5;
        obs.obstacle_width = 2.0;
        obs.obstacle_height = 1.5;

        obs.obstacle_id = "4";
        obs.obstacle_type = 1;   // 假设1=车辆
        obs.obstacle_shape = 2;  // 矩形

        obs.obstacle_threa = 0.0; // 朝向 (yaw)
        obs.obstacle_velocity = 0.0;
        obs.obstacle_acc = 0.0;

        if (obs.obstacle_velocity > 0.1) // 动态障碍物
        {
            oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                    Config_.dynamic_obs_predict_time, obs.obstacle_threa, 
                                    obs.obstacle_velocity, obs.obstacle_acc);
        }

        PPoint center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        PPoint left_front;
        PPoint left_buttom;
        PPoint right_front;
        PPoint right_buttom;

        oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, center, obs.obstacle_threa, left_front, left_buttom,
                                        right_buttom, right_front);

        std::vector<PPoint> corners = {left_front, right_front, right_buttom, left_buttom};                                
        if (obs.obstacle_shape == 2) 
        {
            // 顶点
            obs.pinnacle.poses.clear();
            for (size_t j = 0; j < corners.size(); j++)
            {
                geometry_msgs::Pose sds;
                sds.position.x = corners[j].x;
                sds.position.y = corners[j].y;
                sds.position.z = 0; // 压缩二维
                obs.pinnacle.poses.push_back(sds);
                obs.polygon_points.push_back(Vec2d(corners[j].x, corners[j].y));
            }
            // 半径为0
            obs.obstacle_radius = 0;

        }
        obs.start_time = 6;
        obs.end_time = 8;
        AllObstacle.push_back(obs);
    }
    ROS_INFO("Test obstacles created, total: %zu", AllObstacle.size());
}

void ExportSTBoundaries(const StGraphData &st_graph_data, const std::string &filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "❌ Failed to open file: " << filename << std::endl;
        return;
    }

    const auto &boundaries = st_graph_data.st_boundaries();
    ofs << "# Exported ST Boundaries, total=" << boundaries.size() << "\n";

    for (size_t i = 0; i < boundaries.size(); ++i) {
        const auto &boundary = boundaries[i];
        ofs << "Obstacle " << boundary.id() << " index=" << i << "\n";

        ofs << "# Upper points (t, s)\n";
        for (const auto &p : boundary.getUpper_points()) {
            ofs << p.t() << " " << p.s() << "\n";
        }

        ofs << "# Lower points (t, s)\n";
        for (const auto &p : boundary.getLower_points()) {
            ofs << p.t() << " " << p.s() << "\n";
        }

        ofs << "\n"; // 每个障碍物之间空一行
    }

    ofs.close();
    std::cout << "✅ ST boundaries exported to " << filename << std::endl;
}

void ExportSpeedData(const SpeedData &speed_data, const std::string &filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "❌ Failed to open file: " << filename << std::endl;
        return;
    }

    ofs << "# Exported Speed Profile (t, s, v, a)\n";
    for (const auto &p : speed_data) {
        ofs << p.t << " " << p.s << " " << p.v << " " << p.a << "\n";
    }

    ofs.close();
    std::cout << "✅ Speed profile exported to " << filename << std::endl;
}


void ExportPathData(const PathData &path_data, const std::string &filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "❌ Failed to open file: " << filename << std::endl;
        return;
    }

    ofs << "# Exported DP Path (s, x, y, theta, kappa)\n";
    for (const auto &p : path_data.discretized_path()) {
        ofs << p.s << " " << p.x << " " << p.y
            << " " << p.theta << " " << p.kappa << "\n";
    }

    ofs.close();
    std::cout << "✅ DP Path exported to " << filename << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_dp_speed_node"); 
    ros::NodeHandle nh;
    std::vector<ReferencePoint> reference_points;
    std::vector<double> headings, kappas, dkappas, accumulated_s;
    std::vector<std::pair<double, double>> xy_points;
    std::pair<std::vector<double>, std::vector<double>> reference_path;
    TrajectoryPoint planning_init_point;
    SpeedData speed_data;
    Obstacle_avoid oba(Config_);
    VehicleStateProvider vehicle_state_provider_;

    // === 清理输出目录 ===
    std::string output_dir = "/home/bob/文档/备份/demo05/src/test/txt";
    try {
        for (const auto &entry : fs::directory_iterator(output_dir)) {
            if (fs::is_regular_file(entry)) {
                fs::remove(entry);
            }
        }
        std::cout << "🧹 Cleared old txt files in " << output_dir << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "⚠️ Failed to clear output dir: " << e.what() << std::endl;
    }

    // =============== 1. 构造初始点 =================
    nav_msgs::Odometry vehicle_pos;
    vehicle_pos.pose.pose.position.x = 0.0;
    vehicle_pos.pose.pose.position.y = 0.0;
    vehicle_pos.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    tf::Quaternion q;
    tf::quaternionMsgToTF(vehicle_pos.pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double vehicle_heading = yaw;
    InitialConditions init_point = {0.0, 0.0, 0.0, // d0, dd0, ddd0
                                    0.0, 5.0, 0.0, // s0, ds0, dds0
                                    0.0,           // relative_time
                                    vehicle_pos.pose.pose.position.x, // x
                                    vehicle_pos.pose.pose.position.y,// y
                                    0.0, // z
                                    5.0, 0.0, // v, a
                                    0.0, 0.0,
                                    0.0 // theta, kappa, dkappa
    };

    CopyFrom(planning_init_point, init_point);

    const auto& pos = vehicle_pos.pose.pose.position;
    const auto& ori = vehicle_pos.pose.pose.orientation;
    const auto& twist = vehicle_pos.twist.twist;
    double ts = vehicle_pos.header.stamp.toSec();
    double v = std::sqrt(
        twist.linear.x * twist.linear.x +
        twist.linear.y * twist.linear.y +
        twist.linear.z * twist.linear.z);
    double a = 0.0;
    double yaw_rate = twist.angular.z;
    vehicle_state_provider_.Update(
        pos.x, pos.y, pos.z,
        ori.w, ori.x, ori.y, ori.z,
        v, a, yaw_rate, ts);

    // =============== 2. 构造参考线（直线） =================
    nav_msgs::Path path_msg;
    for (double x = 0.0; x <= 100.0; x += 1.0) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = 0.0;
        path_msg.poses.push_back(pose);
    }
    for (const auto &pose : path_msg.poses) {
        xy_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        reference_path.first.push_back(pose.pose.position.x);
        reference_path.second.push_back(pose.pose.position.y);
    }

    if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s,
                                        &kappas, &dkappas)) {
        ROS_ERROR("Reference line generation failed!");
        return -1;
    }

    for (size_t i = 0; i < xy_points.size(); ++i) {
        ReferencePoint ref_point(kappas[i], dkappas[i], xy_points[i].first,
                                xy_points[i].second, headings[i],
                                accumulated_s[i]);
        reference_points.push_back(ref_point);
    }

    CubicSpline2D *csp = new CubicSpline2D(reference_path.first, reference_path.second,
                            accumulated_s);

    ReferenceLine reference_line(csp, reference_points, accumulated_s,
                                reference_path, Config_);
    // 创建参考线info对象
    ReferenceLineInfo reference_line_info(vehicle_state_provider_.vehicle_state(), reference_line);

    ROS_INFO("Reference line generated successfully with %zu points.", reference_points.size());
    

    // =============== 3. 构造TEST障碍物 =================
    createTestObstacles(oba, AllObstacle);
    // 只读视图：给 ST 图 / PathTimeGraph / SpeedLimitDecider 用
    std::vector<const Obstacle *> obstacles_const;
    obstacles_const.reserve(AllObstacle.size());

    // 可写视图：给 SpeedDecider 用
    std::vector<Obstacle *> obstacles_mutable;
    obstacles_mutable.reserve(AllObstacle.size());

    for (size_t i = 0; i < AllObstacle.size(); i++)
    {
        obstacles_const.emplace_back(&AllObstacle[i]);
        obstacles_mutable.emplace_back(&AllObstacle[i]);
    }

    // =============== 4. 构造自车的SL和障碍物的SL和ST =================
    //---------------------自主车的SL---------------------//
    SL_Boundary adc_sl_boundary;
    Vec2d vec_to_center(
        (Config_.front_edge_to_center - Config_.back_edge_to_center) / 2.0,
        (Config_.left_edge_to_center - Config_.right_edge_to_center) / 2.0);
    // realtime vehicle position
    Vec2d vehicle_position(vehicle_pos.pose.pose.position.x, vehicle_pos.pose.pose.position.y);

    Vec2d vehicle_center(vehicle_position + vec_to_center.rotate(vehicle_heading));
    Box2d vehicle_box(vehicle_center, vehicle_heading, Config_.FLAGS_vehicle_length, Config_.FLAGS_vehicle_width);
    if (!reference_line.GetSLBoundary(vehicle_box, &adc_sl_boundary))
    {
        ROS_ERROR("GetSLBoundary failed.");
        return -1;
    }
    //---------------------障碍物SL和ST---------------------//
    ReferencePoint matched_point = PathMatcher::MatchToPath(reference_line.path_reference(), planning_init_point.path_point().x,
                                                            planning_init_point.path_point().y);
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;

    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
    auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles_const, reference_line.path_reference(), init_s[0],
                                                                init_s[0] + 40, // 前瞻多少m
                                                                0.0, Config_.FLAGS_trajectory_time_length, init_d, Config_);
                                  
    std::cout << "ST obstacles: " << ptr_path_time_graph->get_path_time_obstacles().size() << std::endl;
    std::cout << "SL boundaries: " << ptr_path_time_graph->get_static_obs_sl_boundaries().size() << std::endl;
    std::vector<const Obstacle *> obstacles_;
    // 过滤地图障碍物
    for (size_t i = 0; i < ptr_path_time_graph->get_path_time_obstacles().size(); i++)
    {
        auto path_time_obstacle = ptr_path_time_graph->get_path_time_obstacles().at(i);

        for (auto &obstacle : AllObstacle)
        {
        //  如果说有ST图的障碍物，才有效
        if (path_time_obstacle.obstacle_id == obstacle.obstacle_id)
        {
            if (obstacle.IsStatic() && obstacle.obstacle_type == 1) // 静态或者低速的车辆障碍物
            {
            // 找到对应的 SL_Boundary，而不是直接用 i
            for (size_t i = 0; i < ptr_path_time_graph->get_static_obs_sl_boundaries().size(); i++) {
                if (ptr_path_time_graph->get_static_obs_sl_boundaries()[i].obstacle_id == obstacle.obstacle_id) {
                    auto static_obs_sl_boundary = ptr_path_time_graph->get_static_obs_sl_boundaries().at(i);
                    obstacle.SetSLBoundary(static_obs_sl_boundary);
                    break;
                }
            }
            obstacle.SetSTBoundary(path_time_obstacle);
            obstacle.SetLateralDecision();
            }
            else
            {
            obstacle.SetSTBoundary(path_time_obstacle);
            }
            obstacles_.push_back(&obstacle);
        }
        }
    }
    // =============== 5. 构造 PathData =================
    // 1.路径规划 DP
    PathData path_data;
    DpPolyPathOptimizer dp_optimizer(obstacles_const, vehicle_pos, Config_);
    if (!dp_optimizer.Process(speed_data, reference_line, planning_init_point,
                                &path_data)) {
        ROS_ERROR("DP path planning failed!");
        return -1;
    }
    double total_length_s_ = path_data.Length();
    ExportPathData(path_data, "/home/bob/文档/备份/demo05/src/test/txt/dp_path.txt");

    // 2.路径规划 QP
    QpSplinePathOptimizer PathOptimizer(Config_); // ActiveSetSpline1dSolver求解器
    if (!PathOptimizer.Process(speed_data, reference_line, planning_init_point, &path_data, obstacles_))
    {
        ROS_INFO("Failed to generate PathOptimizer");
        return {};
    }
    ExportPathData(path_data, "/home/bob/文档/备份/demo05/src/test/txt/qp_path.txt");

    // =============== 6.速度规划 =================
    // 1.创建速度限制类
    planner::SpeedLimit speed_limits;
    SpeedLimitDecider speed_limit_decider(adc_sl_boundary, reference_line, path_data, Config_);
    speed_limit_decider.GetSpeedLimits(obstacles_, &speed_limits);
    ROS_INFO("[SpeedLimit] total points: %zu", speed_limits.speed_limit_points().size());

    for (size_t i = 0; i < speed_limits.speed_limit_points().size(); ++i) {
        const auto &pt = speed_limits.speed_limit_points()[i];
        ROS_INFO("[SpeedLimit] idx=%zu  s=%.3f  v_limit=%.3f", i, pt.first, pt.second);
    }
    // 2.创建st图对象
    StGraphData st_graph_data(Config_);
    st_graph_data.LoadData(ptr_path_time_graph->get_path_time_obstacles(), 0.0, planning_init_point, speed_limits, 
    Config_.default_cruise_speed, total_length_s_, Config_.total_time);
    ExportSTBoundaries(st_graph_data, "/home/bob/文档/备份/demo05/src/test/txt/st_boundaries.txt");

    // 3.速度规划 DP
    PathTimeHeuristicOptimizer SpeedProfile(st_graph_data, obstacles_, Config_);
    if (!SpeedProfile.Process(path_data, planning_init_point, &speed_data))
    {
        ROS_INFO("Failed to generate SpeedProfile");
        return -1;
    }
    ExportSpeedData(speed_data, "/home/bob/文档/备份/demo05/src/test/txt/dp_speed.txt");

    // =============== SpeedDecider 决策 ===============
    SpeedDecider speed_decider(planning_init_point, adc_sl_boundary, &reference_line, Config_.role_name_, Config_);

    if (!speed_decider.Execute(speed_data, obstacles_mutable)) {
        ROS_ERROR("SpeedDecider execution failed.");
    } else {
        ROS_INFO("SpeedDecider executed successfully.");
        for (auto* obs : obstacles_mutable) {
            if (!obs) continue;
            std::cout << "Obstacle " << obs->obstacle_id
                    << " LongitudinalDecision = " << obs->LongitudinalDecision().DecisionTagName()
                    << std::endl;
        }
    }

    for (auto &obstacle : obstacles_mutable)
    {
        for(size_t i = 0; i < st_graph_data.st_boundaries().size(); i++){
            if(obstacle->obstacle_id == st_graph_data.st_boundaries().at(i).id()){
                if ( obstacle->longitudinal_decision_.tag == DecisionTag::IGNORE){
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::KEEP_CLEAR;
                }else if(obstacle->longitudinal_decision_.tag == DecisionTag::STOP){
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::STOP;
                }else if(obstacle->longitudinal_decision_.tag == DecisionTag::FOLLOW){
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::FOLLOW;
                }else if(obstacle->longitudinal_decision_.tag == DecisionTag::YIELD){
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::YIELD;
                }else if(obstacle->longitudinal_decision_.tag == DecisionTag::OVERTAKE){
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::OVERTAKE;
                }else{
                    st_graph_data.st_boundaries_.at(i).boundary_type_ = ST_Boundary::BoundaryType::UNKNOWN;
                }
            }
        }
    }

    // 4.速度规划 QP
    QpSplineStSpeedOptimizer SpeedOptimizer(Config_);
    if (!SpeedOptimizer.Process(adc_sl_boundary, st_graph_data, path_data, planning_init_point, reference_line_info, &speed_data))
    {
        ROS_INFO("Failed to generate SpeedOptimizer");
        return -1;
    }
    ExportSpeedData(speed_data, "/home/bob/文档/备份/demo05/src/test/txt/qp_speed.txt");
    
    // =============== 7. 输出结果 =================
    std::cout << "QP Speed result has " << speed_data.size() << " points." << std::endl;
    for (const auto &p : speed_data) {
        std::cout << "t=" << p.t << " s=" << p.s << " v=" << p.v << " a=" << p.a << std::endl;
    }

    return 0;
}
