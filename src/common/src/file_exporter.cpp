#include "common/file_exporter.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <chrono>

namespace common {

void FileExporter::ExportEgoPath(
    const std::vector<carla::geom::Transform> &all_points,
    const std::vector<carla::geom::Transform> &path,
    const std::string &filename,
    size_t frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file,
    bool csv_mode) {
    try {
        std::string target_file = filename;

        // ✅ 分文件模式
        if (split_mode) {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem   = fpath.stem().string();
            std::string ext    = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }

        // ✅ 确保目录存在
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        // ✅ 打开文件
        std::ios_base::openmode mode = std::ios::out;
        if (append_mode && !split_mode) mode |= std::ios::app;
        else mode |= std::ios::app;

        std::ofstream ofs(target_file, mode);
        if (!ofs.is_open()) {
            std::cerr << "❌ Failed to open " << target_file << std::endl;
            return;
        }

        ofs << std::fixed << std::setprecision(6);

        auto now = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(now);

        // ✅ 普通文本模式
        if (!csv_mode) {
            ofs << "=============================\n";
            ofs << "# Frame: " << frame_id
                << "  Time: " << std::put_time(std::localtime(&t_c), "%F %T") << "\n";

            ofs << "# All Local Path Points (x, y, yaw)\n";
            for (size_t i = 0; i < all_points.size(); ++i) {
                double yaw = all_points[i].rotation.yaw * M_PI / 180.0;
                ofs << i << " "
                    << all_points[i].location.x << " "
                    << all_points[i].location.y << " "
                    << yaw << "\n";
            }

            ofs << "----\n";
            ofs << "# Final Selected Path (x, y, yaw)\n";
            for (size_t i = 0; i < path.size(); ++i) {
                double yaw = path[i].rotation.yaw * M_PI / 180.0;
                ofs << i << " "
                    << path[i].location.x << " "
                    << path[i].location.y << " "
                    << yaw << "\n";
            }
            ofs << "\n";
        }
        // ✅ CSV 模式
        else {
            if (ofs.tellp() == 0) {
                ofs << "frame_id,type,index,x,y,yaw\n";
            }
            // 所有点
            for (size_t i = 0; i < all_points.size(); ++i) {
                double yaw = all_points[i].rotation.yaw * M_PI / 180.0;
                ofs << frame_id << ","
                    << "all" << ","
                    << i << ","
                    << all_points[i].location.x << ","
                    << all_points[i].location.y << ","
                    << yaw << "\n";
            }
            // 最终路径
            for (size_t i = 0; i < path.size(); ++i) {
                double yaw = path[i].rotation.yaw * M_PI / 180.0;
                ofs << frame_id << ","
                    << "final" << ","
                    << i << ","
                    << path[i].location.x << ","
                    << path[i].location.y << ","
                    << yaw << "\n";
            }
        }

        ofs.close();
    } catch (const std::exception &e) {
        std::cerr << "❌ ExportEgoPath exception: " << e.what() << std::endl;
    }
}


}  // namespace planner
