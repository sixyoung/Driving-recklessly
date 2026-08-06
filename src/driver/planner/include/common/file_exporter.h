#pragma once
#include <vector>
#include <string>
#include "em_planner.h"
#include "dp_road_graph.h"

namespace planner {

class FileExporter {
public:
    static void ExportPathWaypointsSample(
        const std::vector<std::vector<SLPoint>>& path_waypoints,
        const ReferenceLine& reference_line,
        const std::string& filename,
        size_t frame_id,
        bool append_mode = true,
        bool split_mode = false,
        size_t frames_per_file = 300);

    static void ExportPathData(const PathData &path_data,
                            const std::string &filename,
                            size_t frame_id,
                            bool append_mode = true,
                            bool split_mode = false,       // ✅ 是否分文件
                            size_t frames_per_file = 300,  // ✅ 每个文件帧数
                            bool csv_mode = false          // ✅ CSV 模式
    );

    static void ExportFrenetPath(const PathData &path_data,
                                const std::string &filename,
                                size_t frame_id,
                                bool append_mode = true,
                                bool split_mode = false,
                                size_t frames_per_file = 300,
                                bool csv_mode = false);

    static void ExportSTBoundaries(const StGraphData &st_graph_data,
                                const std::string &filename,
                                const int frame_id,
                                bool append_mode = true,
                                bool split_mode = false,      // ✅ 是否分文件
                                size_t frames_per_file = 300, // ✅ 每个文件保存多少帧
                                bool csv_mode = false         // ✅ CSV 模式
    );

    static void ExportOptimTrajectory(const DiscretizedTrajectory &trajectory,
                                    const std::string &filename,
                                    size_t frame_id,
                                    bool append_mode = true,
                                    bool split_mode = false,       // ✅ 是否分文件
                                    size_t frames_per_file = 300,  // ✅ 每个文件多少帧
                                    bool csv_mode = false          // ✅ CSV 输出模式
    );

    static void ExportSpeedData(const SpeedData &speed_data,
                                const std::string &filename,
                                size_t frame_id,
                                bool append_mode = true,
                                bool split_mode = false,       // ✅ 是否分文件保存
                                size_t frames_per_file = 300,  // ✅ 每文件帧数
                                bool csv_mode = false          // ✅ 新增：CSV 表格模式
    );

    static void ExportReferenceLine(const ReferenceLine& reference_line,
                                    const std::string& filename,
                                    size_t frame_id,
                                    bool append_mode = true,
                                    bool split_mode = false,       // ✅ 是否分文件保存
                                    size_t frames_per_file = 300   // ✅ 每个文件保存多少帧
    );

    static void ExportPlanningInitPoint(const TrajectoryPoint &planning_init_point,
                                        const std::string &filename,
                                        size_t frame_id,
                                        bool append_mode = true,
                                        bool split_mode = false,       // ✅ 是否分文件
                                        size_t frames_per_file = 300   // ✅ 每个文件多少帧
    );

    static void ExportVehicleOdom(const nav_msgs::Odometry &odom,
                                const std::string &filename,
                                size_t frame_id,
                                bool append_mode = true,
                                bool split_mode = false,       // ✅ 是否分文件保存
                                size_t frames_per_file = 300   // ✅ 每个文件保存多少帧
    );

    static void SaveSpeedLimits(const planner::SpeedLimit& speed_limits,
                                const std::string& filename,
                                size_t frame_id,
                                bool append_mode = true,
                                bool split_mode = false,       // ✅ 是否分文件保存
                                size_t frames_per_file = 300   // ✅ 每个文件多少帧
    );

    static void ExportObstacles(const std::vector<Obstacle> &obstacles,
                                const std::string &filename,
                                const int frame_id,
                                bool append_mode = true,
                                bool split_mode = false,       // ✅ 是否分文件
                                size_t frames_per_file = 300   // ✅ 每个文件保存多少帧
    );

    static void ExportSimulationData(const ReferenceLine& reference_line,
                                    const nav_msgs::Odometry &ego_odmo,
                                    const TrajectoryPoint &planning_init_point,
                                    const PublishableTrajectory &last_planned_trajectory,
                                    const DiscretizedTrajectory &current_planned_trajectory,
                                    const std::string &filename,
                                    double frame_header_time
    );

    static void ExportTrajectoryCostObstacles(
                                    const std::vector<SL_Boundary> &static_boundaries,
                                    const std::vector<std::vector<Box2d>> &dynamic_boxes,
                                    const std::string &filename,
                                    int frame_id,
                                    bool append_mode,
                                    bool split_mode,
                                    size_t frames_per_file,
                                    size_t num_obs);
    static void ExportLateralBoundaries(
                                    const std::vector<double>& evaluated_s,
                                    const std::vector<double>& boundary_low,
                                    const std::vector<double>& boundary_high,
                                    const std::string& filename,
                                    int frame_id,
                                    bool append_mode,
                                    bool split_mode,
                                    size_t frames_per_file);
    static void ExportStConstraints(
                                    const std::vector<double>& t_evaluated,
                                    const std::vector<double>& s_upper_bound,
                                    const std::vector<double>& s_lower_bound,
                                    const std::string& filename,
                                    int frame_id,
                                    bool append_mode,
                                    bool split_mode,
                                    size_t frames_per_file);
    static void ExportObstacleBox(
                                    const planner::Box2d &box,
                                    const std::string,
                                    double relative_time,
                                    const std::string &filename,
                                    int frame_id = 0,
                                    bool append_mode = true,
                                    bool split_mode = false,
                                    size_t frames_per_file = 300);
    static void ExportStCostTable(
                                    const std::vector<std::vector<StGraphPoint>>& cost_table,
                                    const std::vector<std::vector<bool>>& visited_table,
                                    const std::string& filename,
                                    int frame_id,
                                    bool append_mode,
                                    bool split_mode,
                                    size_t frames_per_file);
};
}  // namespace planner
