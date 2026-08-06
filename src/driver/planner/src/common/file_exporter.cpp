#include "file_exporter.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <chrono>

namespace planner {

inline void WriteUnifiedFrameHeader(std::ofstream &ofs, int frame_id) {
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    ofs << "# Frame ID: " << frame_id << "\n";
    ofs << "# Timestamp: " << std::put_time(std::localtime(&t_c), "%F %T") << "\n";
}

void FileExporter::ExportPathWaypointsSample(
    const std::vector<std::vector<SLPoint>>& path_waypoints,
    const ReferenceLine& reference_line,
    const std::string& filename,
    size_t frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        std::ios_base::openmode mode = std::ios::out | std::ios::app;
        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        auto now = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Path Waypoints Sample (SL + XY)\n";

        for (size_t i = 0; i < path_waypoints.size(); ++i) {
            for (size_t j = 0; j < path_waypoints[i].size(); ++j) {
                const auto& pt = path_waypoints[i][j];
                Vec2d xy_point;
                if (!reference_line.SLToXY(pt, &xy_point)) continue;
                ofs << i << " " << j << " "
                    << pt.s << " " << pt.l << " "
                    << xy_point.x() << " " << xy_point.y() << "\n";
            }
            ofs << "----\n";
        }

        ofs << "\n";
        ofs.close();
        // std::cout << "✅ Path waypoints (SL+XY) exported to " << target_file << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportPathWaypointsSample exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportPathData(const PathData &path_data,
                                  const std::string &filename,
                                  size_t frame_id,
                                  bool append_mode,
                                  bool split_mode,       // ✅ 是否分文件
                                  size_t frames_per_file, // ✅ 每个文件帧数
                                  bool csv_mode)          // ✅ CSV 模式
{
    try {
        std::string target_file = filename;

        // ==== 分文件模式 ====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // ==== 确保目录存在 ====
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // ==== 打开文件 ====
        std::ios_base::openmode mode = std::ios::out | std::ios::app;
        if (!append_mode && !split_mode) mode = std::ios::out | std::ios::trunc;
        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);
        auto now = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // ======================================================
        // =============== 非 CSV 模式文本输出 ==================
        // ======================================================
        if (!csv_mode) {
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);

            ofs << "# Discretized Path (s, x, y, theta, kappa)\n";
            for (const auto &p : path_data.discretized_path()) {
                ofs << p.s << " "
                    << p.x << " "
                    << p.y << " "
                    << p.theta << " "
                    << p.kappa << "\n";
            }
            ofs << "\n";

            ofs << "# Frenet Path (s, d)\n";
            for (const auto &p : path_data.frenet_frame_path()) {
                ofs << p.s << " " << p.d << "\n";
            }
            ofs << "\n";
        }
        // ======================================================
        // =============== CSV 模式输出 ==========================
        // ======================================================
        else {
            // 写表头（如果文件为空）
            if (ofs.tellp() == 0) {
                ofs << "frame_id,type,s,x,y,theta,kappa,d\n";
            }

            // 1. 笛卡尔路径
            for (const auto &p : path_data.discretized_path()) {
                ofs << frame_id << ",cartesian,"
                    << p.s << ","
                    << p.x << ","
                    << p.y << ","
                    << p.theta << ","
                    << p.kappa << ","
                    << 0.0   // d 无意义填 0
                    << "\n";
            }

            // 2. Frenet 路径
            for (const auto &p : path_data.frenet_frame_path()) {
                ofs << frame_id << ",frenet,"
                    << p.s << ","
                    << 0.0 << ","  // x
                    << 0.0 << ","  // y
                    << 0.0 << ","  // theta
                    << 0.0 << ","  // kappa
                    << p.d << "\n";
            }
        }

        ofs.close();
    } catch (const std::exception &e) {
        std::cerr << "❌ ExportPathData exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportFrenetPath(const PathData &path_data,
                             const std::string &filename,
                             size_t frame_id,
                             bool append_mode,
                             bool split_mode,
                             size_t frames_per_file,
                             bool csv_mode) {
    try {
        std::string target_file = filename;

        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) mode |= std::ios::app;
        else mode |= std::ios::app;

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        if (!csv_mode) {
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);
            ofs << "# Format: (s, l, dl, ddl)\n";

            for (const auto &p : path_data.frenet_frame_path()) {
                ofs << p.s    << " "
                    << p.d    << " "
                    << p.d_dd << " "
                    << p.d_ddd<< "\n";
            }
            ofs << "\n";
        } else {
            if (ofs.tellp() == 0) ofs << "frame_id,s,l,dl,ddl\n";
            for (const auto &p : path_data.frenet_frame_path()) {
                ofs << frame_id << ","
                    << p.s << ","
                    << p.d << ","
                    << p.d_dd << ","
                    << p.d_ddd << "\n";
            }
        }
        ofs.close();
    } catch (const std::exception &e) {
        std::cerr << "❌ ExportFrenetPath exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportSTBoundaries(const StGraphData &st_graph_data,
                               const std::string &filename,
                               const int frame_id,
                               bool append_mode,
                               bool split_mode,      // ✅ 是否分文件
                               size_t frames_per_file, // ✅ 每个文件保存多少帧
                               bool csv_mode         // ✅ CSV 模式
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含"."）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件叠加模式
        } else {
            mode |= std::ios::app;  // 分文件时每个文件也叠加写
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        auto now   = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        if (!csv_mode) {
            // ============= 普通模式 =============
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);

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
                ofs << "\n";
            }
            ofs << "\n";

        } else {
            // ============= CSV 模式 =============
            // 如果是新文件，写表头
            if (ofs.tellp() == 0) {
                ofs << "frame_id,obstacle_id,t,s,type\n";
            }

            const auto &boundaries = st_graph_data.st_boundaries();
            for (const auto &boundary : boundaries) {
                for (const auto &p : boundary.getUpper_points()) {
                    ofs << frame_id << ","
                        << boundary.id() << ","
                        << p.t() << ","
                        << p.s() << ",upper\n";
                }
                for (const auto &p : boundary.getLower_points()) {
                    ofs << frame_id << ","
                        << boundary.id() << ","
                        << p.t() << ","
                        << p.s() << ",lower\n";
                }
            }
        }

        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportSTBoundaries exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportOptimTrajectory(const DiscretizedTrajectory &trajectory,
                                  const std::string &filename,
                                  size_t frame_id,
                                  bool append_mode,
                                  bool split_mode,       // ✅ 是否分文件
                                  size_t frames_per_file,  // ✅ 每个文件多少帧
                                  bool csv_mode          // ✅ CSV 输出模式
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件追加
        } else {
            mode |= std::ios::app;  // 分文件时，每个文件独立累加
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // 系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        if (!csv_mode) {
            // ===== 普通模式 =====
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);
            ofs << "# Optimized Trajectory (t, s, x, y, theta, v, a)\n";

            for (const auto &p : trajectory) {
                ofs << p.relative_time     << " "
                    << p.path_point().s    << " "
                    << p.path_point().x    << " "
                    << p.path_point().y    << " "
                    << p.path_point().theta<< " "
                    << p.v                 << " "
                    << p.a                 << "\n";
            }
            ofs << "\n";
        } else {
            // ===== CSV 模式 =====
            // 如果是新文件，还需要写表头
            if (ofs.tellp() == 0) {
                ofs << "frame_id,t,s,x,y,theta,v,a\n";
            }
            for (const auto &p : trajectory) {
                ofs << frame_id << ","
                    << p.relative_time     << ","
                    << p.path_point().s    << ","
                    << p.path_point().x    << ","
                    << p.path_point().y    << ","
                    << p.path_point().theta<< ","
                    << p.v                 << ","
                    << p.a                 << "\n";
            }
        }

        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportOptimTrajectory exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportSpeedData(const SpeedData &speed_data,
                            const std::string &filename,
                            size_t frame_id,
                            bool append_mode,
                            bool split_mode,       // ✅ 是否分文件保存
                            size_t frames_per_file,  // ✅ 每文件帧数
                            bool csv_mode          // ✅ 新增：CSV 表格模式
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含.）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件追加
        } else {
            mode |= std::ios::app;  // 分文件时，每个文件独立累加
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // 获取系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        if (!csv_mode) {
            // ===== 普通文本模式 =====
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);
            ofs << "# Format: (t, s, v, a)\n";

            for (const auto &p : speed_data) {
                ofs << p.t << " "
                    << p.s << " "
                    << p.v << " "
                    << p.a << "\n";
            }
            ofs << "\n";
        } else {
            // ===== CSV 表格模式 =====
            // 如果是新文件，还要写表头
            if (ofs.tellp() == 0) {
                ofs << "frame_id,t,s,v,a\n";
            }
            for (const auto &p : speed_data) {
                ofs << frame_id << ","
                    << p.t << ","
                    << p.s << ","
                    << p.v << ","
                    << p.a << "\n";
            }
        }

        ofs.close();
        // std::cout << "✅ SpeedData saved to " << target_file << " (frame " << frame_id << ")\n";

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportSpeedData exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportReferenceLine(const ReferenceLine& reference_line,
                                const std::string& filename,
                                size_t frame_id,
                                bool append_mode,
                                bool split_mode,       // ✅ 是否分文件保存
                                size_t frames_per_file   // ✅ 每个文件保存多少帧
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件追加模式
        } else {
            mode |= std::ios::app;  // 分文件时每个文件也追加
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // 获取当前系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // 帧头
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Exported Reference Line (s, x, y, theta, kappa, dkappa)\n";

        // 写入参考线点
        const auto& ref_points = reference_line.reference_points();
        for (const auto& p : ref_points) {
            ofs << p.accumulated_s_ << " "
                << p.x_ << " "
                << p.y_ << " "
                << p.heading_ << " "
                << p.kappa_ << " "
                << p.dkappa_ << "\n";
        }
        ofs << "\n";

        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportReferenceLine exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportPlanningInitPoint(const TrajectoryPoint &planning_init_point,
                                    const std::string &filename,
                                    size_t frame_id,
                                    bool append_mode,
                                    bool split_mode,       // ✅ 是否分文件
                                    size_t frames_per_file   // ✅ 每个文件多少帧
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件叠加
        } else {
            mode |= std::ios::app;  // 分文件时也追加写
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // 系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // 帧头
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Planning Init Point\n";

        // 写数据
        ofs << "x " << planning_init_point.path_point().x << "\n";
        ofs << "y " << planning_init_point.path_point().y << "\n";
        ofs << "theta " << planning_init_point.path_point().theta << "\n";
        ofs << "kappa " << planning_init_point.path_point().kappa << "\n";
        ofs << "s " << planning_init_point.path_point().s << "\n";
        ofs << "d " << planning_init_point.d << "\n";
        ofs << "v " << planning_init_point.v << "\n";
        ofs << "a " << planning_init_point.a << "\n";
        ofs << "relative_time " << planning_init_point.relative_time << "\n\n";

        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportPlanningInitPoint exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportVehicleOdom(const nav_msgs::Odometry &odom,
                              const std::string &filename,
                              size_t frame_id,
                              bool append_mode,
                              bool split_mode,       // ✅ 是否分文件保存
                              size_t frames_per_file   // ✅ 每个文件保存多少帧
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在（即使目录已存在也不会报错）
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件叠加模式
        } else {
            mode |= std::ios::app;  // 分文件时每个文件也叠加写
        }

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // 获取当前系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // 帧头信息
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Vehicle Odom (pose + twist)\n";

        // 位姿
        ofs << "position "
            << odom.pose.pose.position.x << " "
            << odom.pose.pose.position.y << " "
            << odom.pose.pose.position.z << "\n";

        ofs << "orientation "
            << odom.pose.pose.orientation.x << " "
            << odom.pose.pose.orientation.y << " "
            << odom.pose.pose.orientation.z << " "
            << odom.pose.pose.orientation.w << "\n";

        // 速度
        ofs << "linear_velocity "
            << odom.twist.twist.linear.x << " "
            << odom.twist.twist.linear.y << " "
            << odom.twist.twist.linear.z << "\n";

        ofs << "angular_velocity "
            << odom.twist.twist.angular.x << " "
            << odom.twist.twist.angular.y << " "
            << odom.twist.twist.angular.z << "\n\n";

        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportVehicleOdom exception: " << e.what() << std::endl;
    }
}

void FileExporter::SaveSpeedLimits(const planner::SpeedLimit& speed_limits,
                            const std::string& filename,
                            size_t frame_id,
                            bool append_mode,
                            bool split_mode,       // ✅ 是否分文件保存
                            size_t frames_per_file   // ✅ 每个文件多少帧
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 自动创建目录
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件叠加模式
        } else {
            mode |= std::ios::app;  // 分文件时每个文件也叠加写
        }

        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(3);

        // 获取当前系统时间
        auto now   = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // 帧头信息
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Format: (s [m], limit_v [m/s])\n";

        // 写入数据
        for (size_t i = 0; i < speed_limits.speed_limit_points().size(); ++i) {
            const auto& p = speed_limits.speed_limit_points()[i];
            ofs << p.first   // s
                << " "
                << p.second  // v_limit
                << "\n";
        }
        ofs << "\n";

        ofs.close();
        // std::cout << "✅ Speed limits saved to " << target_file << " (frame " << frame_id << ")\n";

    } catch (const std::exception& e) {
        std::cerr << "❌ SaveSpeedLimits exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportObstacles(const std::vector<Obstacle> &obstacles,
                            const std::string &filename,
                            const int frame_id,
                            bool append_mode,
                            bool split_mode,       // ✅ 是否分文件
                            size_t frames_per_file   // ✅ 每个文件保存多少帧
) {
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含"."）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) {
            mode |= std::ios::app;  // 单文件叠加模式
        } else {
            mode |= std::ios::app;  // 分文件时每个文件也叠加写
        }

        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        // 获取系统时间戳
        auto now   = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);

        ofs << "# Exported Obstacles, total=" << obstacles.size() << "\n";

        // 遍历所有障碍物
        for (size_t i = 0; i < obstacles.size(); ++i) {
            const auto &obs = obstacles[i];
            ofs << "Obstacle index=" << i
                << " id=" << obs.obstacle_id << "\n";

            ofs << " position "
                << obs.centerpoint.position.x << " "
                << obs.centerpoint.position.y << " "
                << obs.centerpoint.position.z << "\n";

            ofs << " threa "
                << obs.obstacle_threa << " "
                << "\n";

            ofs << " size "
                << obs.obstacle_length << " "
                << obs.obstacle_width  << " "
                << obs.obstacle_height << "\n";

            ofs << " velocity "
                << obs.obstacle_velocity
                << "\n";

            ofs << "\n"; // 每个障碍物之间空一行
        }

        ofs << "\n"; // 每一帧之间空一行
        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportObstacles exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportSimulationData(const ReferenceLine& reference_line,
                                const nav_msgs::Odometry &ego_odmo,
                                const TrajectoryPoint &planning_init_point,
                                const PublishableTrajectory &last_planned_trajectory,
                                const DiscretizedTrajectory &current_planned_trajectory,
                                const std::string &filename,
                                double frame_header_time) {   // ✅ 新增参数
    std::ofstream ofs(filename, std::ios::app);  // 追加模式
    if (!ofs.is_open()) {
        std::cerr << "❌ Failed to open file: " << filename << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    
    // ===== 当前时间戳（ms） =====
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()).count();
    ofs << "time(ms): " << ms << "\n";

    // ===== 每帧 Header Time =====
    ofs << "# Frame Header Time\n";
    ofs << "frame_header_time " << frame_header_time << "\n";

    // ===== 每帧都保存对应参考线 =====
    ofs << "# Reference Line (s, x, y, theta, kappa, dkappa)\n";
    const auto& ref_points = reference_line.reference_points();
    for (const auto& p : ref_points) {
        ofs << p.accumulated_s_ << " "
            << p.x_ << " "
            << p.y_ << " "
            << p.heading_ << " "
            << p.kappa_ << " "
            << p.dkappa_ << "\n";
    }
    ofs << "===================================\n"; // 分隔符
    // ===== Ego Odom =====
    ofs << "# Ego Odom\n";
    ofs << "position "
        << ego_odmo.pose.pose.position.x << " "
        << ego_odmo.pose.pose.position.y << " "
        << ego_odmo.pose.pose.position.z << "\n";
    ofs << "orientation "
        << ego_odmo.pose.pose.orientation.x << " "
        << ego_odmo.pose.pose.orientation.y << " "
        << ego_odmo.pose.pose.orientation.z << " "
        << ego_odmo.pose.pose.orientation.w << "\n";
    ofs << "linear_velocity "
        << ego_odmo.twist.twist.linear.x << " "
        << ego_odmo.twist.twist.linear.y << " "
        << ego_odmo.twist.twist.linear.z << "\n";
    ofs << "angular_velocity "
        << ego_odmo.twist.twist.angular.x << " "
        << ego_odmo.twist.twist.angular.y << " "
        << ego_odmo.twist.twist.angular.z << "\n";

    // ===== Planning Init Point =====
    ofs << "# Planning Init Point\n";
    ofs << "x " << planning_init_point.path_point().x << "\n";
    ofs << "y " << planning_init_point.path_point().y << "\n";
    ofs << "theta " << planning_init_point.path_point().theta << "\n";
    ofs << "kappa " << planning_init_point.path_point().kappa << "\n";
    ofs << "s " << planning_init_point.path_point().s << "\n";
    ofs << "d " << planning_init_point.d << "\n";
    ofs << "v " << planning_init_point.v << "\n";
    ofs << "a " << planning_init_point.a << "\n";
    ofs << "relative_time " << planning_init_point.relative_time << "\n";

    // ===== Last Planned Trajectory =====
    ofs << "# Last Optimized Trajectory\n";
    ofs << "header_time " << last_planned_trajectory.header_time() << "\n";
    for (const auto &p : last_planned_trajectory) {
        ofs << p.path_point().x << " "
            << p.path_point().y << " "
            << p.s << " "
            << p.v << " "
            << p.a << " "
            << p.relative_time << "\n";
    }

    // ===== Current Planned Trajectory =====
    ofs << "# Current Optimized Trajectory\n";
    for (const auto &p : current_planned_trajectory) {
        ofs << p.path_point().x << " "
            << p.path_point().y << " "
            << p.s << " "
            << p.v << " "
            << p.a << " "
            << p.relative_time << "\n";
    }

    ofs << "-----------------------------------\n";
    ofs.close();

    std::cout << "✅ Simulation data exported to " << filename << std::endl;
}

void FileExporter::ExportTrajectoryCostObstacles(
    const std::vector<SL_Boundary> &static_boundaries,
    const std::vector<std::vector<Box2d>> &dynamic_boxes,
    const std::string &filename,
    int frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file,
    size_t num_obs)
{
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // 打开模式（和ExportObstacles一致）
        std::ios_base::openmode mode = std::ios::out;
        mode |= std::ios::app;

        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        // ===== 帧头信息 =====
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);

        ofs << "# Static Obstacles: " << static_boundaries.size()
            << " | Dynamic Obstacles: " << dynamic_boxes.size() 
            << " | Obstacles: " << num_obs 
            << "\n";

        // ===== 静态障碍物 =====
        ofs << "---- Static SL Boundaries ----\n";
        for (size_t i = 0; i < static_boundaries.size(); ++i) {
            const auto &b = static_boundaries[i];
            ofs << "Static index=" << i
                << " id=" << b.obstacle_id
                << " type=" << b.TypeName() << "\n";

            ofs << " SL-range s:[" << b.start_s_ << ", " << b.end_s_
                << "]  l:[" << b.start_l_ << ", " << b.end_l_ << "]\n";

            ofs << " size " << b.obstacle_length << " " << b.obstacle_width
                << " pos " << b.centerpoint.position.x << " "
                << b.centerpoint.position.y << " "
                << b.centerpoint.position.z << "\n";

            ofs << " velocity " << b.obstacle_velocity
                << "  static=" << std::boolalpha << b.is_static_obstacle << "\n\n";

        }

        // ===== 动态障碍物 =====
        ofs << "---- Dynamic Boxes ----\n";
        for (size_t i = 0; i < dynamic_boxes.size(); ++i) {
            ofs << "Dynamic index=" << i << "\n";
            const auto &boxes = dynamic_boxes[i];

            for (size_t j = 0; j < boxes.size(); ++j) {
                const auto &box = boxes[j];
                ofs << std::fixed << std::setprecision(3)
                    << "  t=" << std::setw(3) << j
                    << "  center=(" << std::setw(7) << box.center().x() << ", "
                    << std::setw(7) << box.center().y() << ")"
                    << "  size=(" << box.length() << "x" << box.width() << ")"
                    << "  heading=" << box.heading() << "\n";
            }
            ofs << "\n";
        }

        ofs << "\n"; // 每帧之间空行
        ofs.close();

    } catch (const std::exception &e) {
        std::cerr << "❌ ExportTrajectoryCostObstacles exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportLateralBoundaries(
    const std::vector<double>& evaluated_s,
    const std::vector<double>& boundary_low,
    const std::vector<double>& boundary_high,
    const std::string& filename,
    int frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // ===== 自动创建目标目录 =====
        std::filesystem::path dir_path = std::filesystem::path(target_file).parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            std::cout << "📁 Created directory: " << dir_path << std::endl;
        }

        // ===== 打开文件 =====
        std::ofstream ofs;
        if (append_mode) {
            ofs.open(target_file, std::ios::app);
        } else {
            ofs.open(target_file, std::ios::trunc);
        }

        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // ===== 帧头信息 =====
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# Lateral Boundaries (s, low, high)\n";

        // ===== 数据内容 =====
        for (size_t i = 0; i < evaluated_s.size(); ++i) {
            ofs << evaluated_s[i] << " "
                << boundary_low[i] << " "
                << boundary_high[i] << "\n";
        }

        ofs << "-----------------------------------\n";
        ofs.close();

        // std::cout << "✅ Frame " << frame_id
        //           << " lateral boundary exported to " << target_file << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception in ExportLateralBoundaries: " << e.what() << std::endl;
    }
}

void FileExporter::ExportStConstraints(
    const std::vector<double>& t_evaluated,
    const std::vector<double>& s_upper_bound,
    const std::vector<double>& s_lower_bound,
    const std::string& filename,
    int frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try {
        std::string target_file = filename;

        // ===== 分文件模式 =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();      // 文件名（无扩展名）
            std::string ext    = fpath.extension().string(); // 扩展名（含 .）
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // ===== 自动创建目标目录 =====
        std::filesystem::path dir_path = std::filesystem::path(target_file).parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            std::cout << "📁 Created directory: " << dir_path << std::endl;
        }

        // ===== 打开文件 =====
        std::ofstream ofs;
        if (append_mode) {
            ofs.open(target_file, std::ios::app);
        } else {
            ofs.open(target_file, std::ios::trunc);
        }

        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        // ===== 帧头信息 =====
        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# ST Constraints (t, s_upper, s_lower)\n";

        // ===== 数据内容 =====
        for (size_t i = 0; i < t_evaluated.size(); ++i) {
            ofs << t_evaluated[i] << " "
                << s_upper_bound[i] << " "
                << s_lower_bound[i] << "\n";
        }

        ofs << "-----------------------------------\n";
        ofs.close();

        // std::cout << "✅ Frame " << frame_id
        //           << " ST constraints exported to " << target_file << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception in ExportStConstraints: " << e.what() << std::endl;
    }
}

void FileExporter::ExportObstacleBox(
    const planner::Box2d &box,
    const std::string obstacle_id,
    double relative_time,
    const std::string &filename,
    int frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try {
        std::string target_file = filename;

        // ===== 分文件模式（可选） =====
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // ===== 自动创建目录 =====
        std::filesystem::path dir_path = std::filesystem::path(target_file).parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
        }

        // ===== 检查该帧是否已写入 =====
        static int last_frame_id = -1; // 上一帧号
        bool is_new_frame = (frame_id != last_frame_id);
        last_frame_id = frame_id;

        // ===== 打开文件 =====
        std::ofstream ofs(target_file, std::ios::app);
        if (!ofs.is_open()) {
            std::cerr << "[ExportObstacleBox] ❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        // ✅ 如果是新帧，写入分隔头
        if (is_new_frame) {
            ofs << "=============================\n";
            WriteUnifiedFrameHeader(ofs, frame_id);
            ofs << "# Obstacle Box (t=" << relative_time << ")\n";
        }

        // ✅ 写入单个障碍物的 box
        ofs << "Obstacle " << obstacle_id
            << "  t=" << relative_time << "\n";
        for (const auto &corner : box.GetAllCorners()) {
            ofs << corner.x() << " " << corner.y() << "\n";
        }
        ofs << "----\n";

        ofs.close();
    }
    catch (const std::exception &e) {
        std::cerr << "[ExportObstacleBox] Exception: " << e.what() << std::endl;
    }
}

void FileExporter::ExportStCostTable(
    const std::vector<std::vector<StGraphPoint>>& cost_table,
    const std::vector<std::vector<bool>>& visited_table,
    const std::string& filename,
    int frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try {
        std::string target_file = filename;

        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem = fpath.stem().string();
            std::string ext  = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        std::filesystem::path dir_path = std::filesystem::path(target_file).parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
        }

        std::ofstream ofs;
        ofs.open(target_file, append_mode ? std::ios::app : std::ios::trunc);

        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open file: " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        ofs << "=============================\n";
        WriteUnifiedFrameHeader(ofs, frame_id);
        ofs << "# ST Cost Table\n";
        ofs << "# Format: t_index s_index t_value s_value cost speed\n";

        for (size_t t = 0; t < cost_table.size(); ++t)
        {
            const auto& row = cost_table[t];
            for (size_t s = 0; s < row.size(); ++s)
            {
                if (!visited_table[t][s]) continue;  // ⭐ 只保存访问过的点

                const auto& point = row[s];

                ofs << t << " "
                    << s << " "
                    << point.point().t() << " "
                    << point.point().s() << " "
                    << point.total_cost() << " "
                    << point.GetOptimalSpeed() << "\n";
            }
            ofs << "\n";
        }

        ofs << "-----------------------------------\n";
        ofs.close();

    }
    catch (const std::exception& e) {
        std::cerr << "❌ Exception in ExportStCostTable: " << e.what() << std::endl;
    }
}



}  // namespace planner
