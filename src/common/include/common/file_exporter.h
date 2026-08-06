#pragma once
#include <vector>
#include <string>
#include <carla/geom/Transform.h>  

namespace common {

class FileExporter {
public:
    static void ExportEgoPath(const std::vector<carla::geom::Transform> &all_points,
                                    const std::vector<carla::geom::Transform> &path,
                                    const std::string &filename,
                                    size_t frame_id,
                                    bool append_mode,
                                    bool split_mode,
                                    size_t frames_per_file,
                                    bool csv_mode);
};

}  // namespace planner
