#include<common/misc.h>
#include<common/transforms.h>


namespace common {

    bool is_within_distance_and_angle(
    const carla::geom::Transform& current_transform,
    const carla::geom::Transform& target_transform,
    double max_distance,
    double min_angle_deg, 
    double max_angle_deg){

        // 计算目标向量
        carla::geom::Vector3D target_vector(target_transform.location.x-current_transform.location.x, 
                                            target_transform.location.y-current_transform.location.y, 
                                            target_transform.location.z-current_transform.location.z);
        double norm_target = target_vector.Length();

        // 如果向量的长度非常小，认为目标就在前方
        if (norm_target < 0.001) {
            return true;
        }

        // 如果目标距离太远
        if (norm_target > max_distance) {
            return false;
        }

        // 获取参考物体的前进方向向量
        carla::geom::Vector3D ego_forward = current_transform.GetForwardVector().MakeUnitVector();

        carla::geom::Vector3D tar_vector = target_vector.MakeUnitVector();  // 将目标向量归一化为单位向量

        // 计算目标向量与前进向量的点积
        double dot = carla::geom::Math::Dot(ego_forward, tar_vector);

        // 计算夹角（弧度）
        double angle = std::acos(dot);  // 计算夹角（弧度）
        dot = std::clamp(dot, -1.0, 1.0);
        // 将夹角从弧度转换为角度
        double angle_in_deg = angle * 180.0 / M_PI;

        // 如果夹角小于 90 度（π/2 弧度），表示目标在自车前方
        return (min_angle_deg <= angle_in_deg) && (angle_in_deg <= max_angle_deg);
    }

    double calculate_angle(const carla::geom::Vector3D& vec1, const carla::geom::Vector3D& vec2) {
        tf2::Vector3 vec1_tf2(vec1.x, vec1.y, vec1.z);
        vec1_tf2.normalize();
        tf2::Vector3 vec2_tf2(vec2.x, vec2.y, vec2.z);
        vec2_tf2.normalize();

        // 计算目标向量与前进向量的点积
        double dot_product = vec1_tf2.dot(vec2_tf2);  // 点积计算
        // 计算夹角（弧度）
        dot_product = std::clamp(dot_product, -1.0, 1.0);
        double angle = std::acos(dot_product);  // 计算夹角（弧度）
        // 将夹角从弧度转换为角度
        double angle_in_deg = angle * 180.0 / M_PI;
        return angle_in_deg;  
    }

    // 判断目标车辆相对于自车的位置
    Side detect_vehicle_side(const carla::geom::Transform& ego_transform,
                            const carla::geom::Transform& target_transform,
                            double angle_threshold) {
        // 获取自车的前向和右向向量
        carla::geom::Vector3D ego_forward = ego_transform.GetForwardVector();
        carla::geom::Vector3D ego_right = ego_transform.GetRightVector();

        // 获取自车和目标车辆的位置
        carla::geom::Vector3D ego_position = ego_transform.location;
        carla::geom::Vector3D target_position = target_transform.location;

        // 计算目标车辆相对自车的向量
        carla::geom::Vector3D relative_position = target_position - ego_position;

        // 归一化向量，防止长度影响计算
        ego_forward = ego_forward.MakeUnitVector();
        ego_right = ego_right.MakeUnitVector();
        relative_position = relative_position.MakeUnitVector();

        // 计算目标车辆与自车的夹角
        double angle = calculate_angle(ego_forward, relative_position);

        // 使用 tf2 库计算目标车辆相对于自车的右向量投影
        tf2::Vector3 relative_position_tf2(relative_position.x, relative_position.y, relative_position.z);
        tf2::Vector3 ego_right_tf2(ego_right.x, ego_right.y, ego_right.z);

        double side_dot = relative_position_tf2.dot(ego_right_tf2);  // 点积判断左右位置

        // 判断目标车辆位置
        if (std::abs(angle) >= angle_threshold && std::abs(angle) <= 90.0) {
            // 目标车辆在自车前方一定角度内
            return (side_dot > 0) ? RIGHT : LEFT;
        } else if (angle > 90.0) {
            // 判断目标车辆在后方
            return BACK;
        }
        // 默认情况下目标车辆在前方
        return FRONT;
    }


    double distance_vehicle(const geometry_msgs::Point& position1, const geometry_msgs::Point& position2) {
        double dx = position1.x - position2.x;
        double dy = position1.y - position2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool is_polygon_intersect(const std::vector<std::vector<double>>& route_bb, 
                           const std::vector<std::vector<double>>& target_bb) {
        // 构建自车路径的 Polygon
        Polygon ego_polygon;
        for (size_t i = 0; i < route_bb.size(); i++) {
            bg::append(ego_polygon, bg::model::d2::point_xy<double>(route_bb[i][0], route_bb[i][1]));
        }
        bg::correct(ego_polygon);  // 确保多边形是有效的

        // 构建目标车路径的 Polygon
        Polygon target_polygon;
        for (size_t i = 0; i < target_bb.size(); i++) {
            bg::append(target_polygon, bg::model::d2::point_xy<double>(target_bb[i][0], target_bb[i][1]));
        }
        bg::correct(target_polygon);  // 确保多边形是有效的

        // 判断是否相交
        return bg::intersects(ego_polygon, target_polygon);
    }

    /**
     * @brief 检查两个多边形是否相交，并提取交集轮廓点（去重）。
     * 
     * 此函数使用 Boost.Geometry 提供的多边形布尔操作功能，对输入的两个多边形执行交集计算，
     * 并返回是否相交的布尔值，以及所有交集区域的边界点（去重后）。边界点使用 std::set 自动去重，
     * 对于共享边界或多个交叉区域，重复点将被忽略。
     * 
     * @param poly1 输入多边形 1（应为闭合合法多边形）
     * @param poly2 输入多边形 2（应为闭合合法多边形）
     * @return std::pair<bool, std::vector<Point>> 
     *         - first: 是否存在交集
     *         - second: 去重后的交集区域边界点集合（可能为空）
     */
    std::pair<bool, std::vector<Point>> CheckPolygonIntersection(const Polygon& poly1, const Polygon& poly2) {
        std::pair<bool, std::vector<Point>> result;
        result.first = false;

        if (poly1.outer().empty() || poly2.outer().empty()) {
            return result;
        }

        // 复制并修正多边形
        Polygon corrected1 = poly1;
        Polygon corrected2 = poly2;
        bg::correct(corrected1);
        bg::correct(corrected2);

        // 计算交集
        std::vector<Polygon> output;
        bg::intersection(corrected1, corrected2, output);

        if (output.empty()) {
            return result;  // 没有交集
        }

        // 自定义比较器用于点去重
        struct PointXYLess {
            bool operator()(const Point& a, const Point& b) const {
                return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
            }
        };

        std::set<Point, PointXYLess> unique_points;

        // 提取所有交集区域的外圈点
        for (const auto& poly : output) {
            for (const auto& pt : poly.outer()) {
                unique_points.insert(pt);  // 自动去重
            }
        }

        // 填充结果
        result.first = true;
        result.second.assign(unique_points.begin(), unique_points.end());
        return result;
    }

    std::pair<bool, bool> is_vehicle_intersect( const carla::geom::Transform& current_transform,
    const carla::geom::Transform& target_transform, const carla::geom::Transform& ego_tar_wp_transform){
        // 自车位置
        carla::geom::Location current_location = current_transform.location;
        // 目标车辆位置
        carla::geom::Location target_location = target_transform.location;

        // 计算自车位置指向目标车辆位置的向量
        carla::geom::Location direction_to_target(target_location.x - current_location.x,
                                              target_location.y - current_location.y,
                                              target_location.z - current_location.z);
        // 计算目标位置指向自车辆位置的向量
        carla::geom::Location direction_to_ego(current_location.x - target_location.x,
                                              current_location.y - target_location.y,
                                              current_location.z - target_location.z);

        carla::geom::Vector3D ego_forward_vector = current_transform.GetForwardVector();
        carla::geom::Vector3D ego_tar_forward_vector = ego_tar_wp_transform.GetForwardVector();

        carla::geom::Vector3D tar_forward_vector = target_transform.GetForwardVector();

        // 计算点积，判断目标车辆是否在自车前方
        float dot_product1 = ego_forward_vector.x * direction_to_target.x + ego_forward_vector.y * direction_to_target.y;
        float dot_product2 = tar_forward_vector.x * direction_to_ego.x + tar_forward_vector.y * direction_to_ego.y;

        // 目标车必须在自车前方
        bool is_target_ahead = dot_product1 > 0.0;
        bool is_ego_ahead = dot_product2 > 0.0;


        if (!is_target_ahead || !is_ego_ahead) {
            return {false, false};  // 目标车辆不在前方，直接返回 false
        }

        // 计算叉积（在二维平面上）
        float cross_product1 = ego_tar_forward_vector.x * direction_to_target.y - ego_tar_forward_vector.y * direction_to_target.x;
        // 计算叉积（在二维平面上）
        float cross_product2 = ego_tar_forward_vector.x * tar_forward_vector.y - ego_tar_forward_vector.y * tar_forward_vector.x;     
        
        // 计算叉积（在二维平面上）
        float cross_product3 = tar_forward_vector.x * direction_to_ego.y - tar_forward_vector.y * direction_to_ego.x;
        // 计算叉积（在二维平面上）
        float cross_product4 = tar_forward_vector.x * ego_tar_forward_vector.y - tar_forward_vector.y * ego_tar_forward_vector.x;  
    
        return {cross_product1*cross_product2 < 0.0, cross_product3*cross_product4 < 0.0};
    }

    // // 函数：计算矩形的四个顶点
    // std::vector<carla::geom::Location> GetRectangleCorners(const carla::geom::Transform& transform, float length_x, float length_y) {
    //     carla::geom::Location center = transform.location;
    //     carla::geom::Vector3D forward = transform.GetForwardVector();
    //     carla::geom::Vector3D right = transform.GetRightVector();

    //     std::vector<carla::geom::Location> corners;

    //     // 使用 Location 类型的显式转换
    //     corners.push_back(center + static_cast<carla::geom::Location>(forward * (length_x / 2.0f) + right * (length_y / 2.0f)));//右上
    //     corners.push_back(center - static_cast<carla::geom::Location>(forward * (length_x / 2.0f) - right * (length_y / 2.0f)));//左下
    //     corners.push_back(center - static_cast<carla::geom::Location>(forward * (length_x / 2.0f) + right * (length_y / 2.0f)));//右下
    //     corners.push_back(center + static_cast<carla::geom::Location>(forward * (length_x / 2.0f) - right * (length_y / 2.0f)));//左上

    //     return corners;
    // }

    /**
     * @brief 计算车辆包络框的 4 个角点 + 4 个边中点
     * 
     * @param transform 车辆的 Transform
     * @param length_x  车辆长度（前后方向）
     * @param length_y  车辆宽度（左右方向）
     * @return std::vector<carla::geom::Location> 顺序：[右上, 右下, 左下, 左上, 上边中点, 下边中点, 左边中点, 右边中点]
     */
    std::vector<carla::geom::Location> GetRectangleCorners(
        const carla::geom::Transform& transform, float length_x, float length_y)
    {
        using carla::geom::Location;
        using carla::geom::Vector3D;

        Location center = transform.location;
        Vector3D forward = transform.GetForwardVector().MakeUnitVector();
        Vector3D right = transform.GetRightVector().MakeUnitVector();

        Vector3D half_forward = forward * (length_x * 0.5f);
        Vector3D half_right   = right * (length_y * 0.5f);

        // 四个角点
        Location right_top = center + static_cast<Location>(half_forward + half_right);
        Location right_bottom = center - static_cast<Location>(half_forward - half_right);
        Location left_bottom = center - static_cast<Location>(half_forward + half_right);
        Location left_top = center + static_cast<Location>(half_forward - half_right);

        // 四个边中点
        Location top_mid = (right_top + left_top) * 0.5f;
        Location bottom_mid = (right_bottom + left_bottom) * 0.5f;
        Location left_mid = (left_top + left_bottom) * 0.5f;
        Location right_mid = (right_top + right_bottom) * 0.5f;

        std::vector<Location> points = {
            top_mid,         // 4 顶边中点
            right_top,       // 0 右上角
            right_mid,       // 7 右边中点
            right_bottom,    // 1 右下角
            bottom_mid,      // 5 底边中点
            left_bottom,     // 2 左下角
            left_mid,        // 6 左边中点
            left_top        // 3 左上角
        };


        return points;
    }

    // 打印顶点的坐标
    void PrintRectangleCorners(const std::vector<carla::geom::Location>& corners) {
        for (size_t i = 0; i < corners.size(); ++i) {
            const auto& corner = corners[i];
            std::cout << "Corner " << i + 1 << ": "
                    << "x = " << corner.x << ", "
                    << "y = " << corner.y << ", "
                    << "z = " << corner.z << std::endl;
        }
    }

    // 函数：计算投影的最小值和最大值
    void GetProjectionRange(const std::vector<carla::geom::Location>& corners, const carla::geom::Vector3D& edge, float& min_proj, float& max_proj) {
        min_proj = max_proj = edge.x * (corners[0].x) + edge.y * (corners[0].y) + edge.z * (corners[0].z);  // 点积计算

        for (const auto& corner : corners) {
            float proj = edge.x * (corner.x) + edge.y * (corner.y) + edge.z * (corner.z);  // 点积计算
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }
    }

        // 函数：检查投影是否有交集不是两个box！！！！，可以修改
    bool CheckProjectionOverlap(const carla::geom::Transform& box1_transform, float box1_length_x, float box1_length_y,
                                const carla::geom::Transform& box2_transform, float box2_length_x, float box2_length_y) {
        // 获取矩形1和矩形2的四个顶点
        std::vector<carla::geom::Location> box1_corners = GetRectangleCorners(box1_transform, box1_length_x, box1_length_y);
        std::vector<carla::geom::Location> box2_corners = GetRectangleCorners(box2_transform, box2_length_x, box2_length_y);

        // 获取矩形1和矩形2的边向量
        carla::geom::Vector3D box1_edges[4] = {
            box1_corners[1] - box1_corners[0],  // 上边
            box1_corners[2] - box1_corners[1],  // 右边
            box1_corners[3] - box1_corners[2],  // 下边
            box1_corners[0] - box1_corners[3]   // 左边
        };

        carla::geom::Vector3D box2_edges[4] = {
            box2_corners[1] - box2_corners[0],  // 上边
            box2_corners[2] - box2_corners[1],  // 右边
            box2_corners[3] - box2_corners[2],  // 下边
            box2_corners[0] - box2_corners[3]   // 左边
        };
        carla::geom::Vector3D right = box1_transform.GetRightVector();
        // 对 box1 的每一条边，计算投影范围
        float box1_min_proj, box1_max_proj;
        GetProjectionRange(box1_corners, right, box1_min_proj, box1_max_proj);

        // 对 box2 的每一条边，计算投影范围
        float box2_min_proj, box2_max_proj;
        GetProjectionRange(box2_corners, right, box2_min_proj, box2_max_proj);

        // 判断是否有投影重叠
        if (box1_max_proj < box2_min_proj || box2_max_proj < box1_min_proj) {
            // 如果没有重叠，返回 false
            return false;
        }
        // 如果所有的投影都有重叠，返回 true
        return true;

        // // 遍历 box1 和 box2 的所有边，检查是否有交集
        // for (int i = 0; i < 4; ++i) {
        //     // 对 box1 的每一条边，计算投影范围
        //     float box1_min_proj, box1_max_proj;
        //     GetProjectionRange(box1_corners, box1_edges[i], box1_min_proj, box1_max_proj);

        //     // 对 box2 的每一条边，计算投影范围
        //     float box2_min_proj, box2_max_proj;
        //     GetProjectionRange(box2_corners, box1_edges[i], box2_min_proj, box2_max_proj);

        //     // 判断是否有投影重叠
        //     if (box1_max_proj < box2_min_proj || box2_max_proj < box1_min_proj) {
        //         // 如果没有重叠，返回 false
        //         return false;
        //     }
        // }

        // // 如果所有的投影都有重叠，返回 true
        // return true;
    }


    std::shared_ptr<carla::geom::Location> CalculateLineSegmentIntersection(
        const carla::geom::Location& ray_start,
        const carla::geom::Vector3D& ray_direction,
        const carla::geom::Location& segment_start,
        const carla::geom::Location& segment_end) {
        // 将向量表示为2D点 (忽略z轴)
        double x1 = ray_start.x;
        double y1 = ray_start.y;
        double x2 = ray_start.x + ray_direction.x;
        double y2 = ray_start.y + ray_direction.y;
        
        double x3 = segment_start.x;
        double y3 = segment_start.y;
        double x4 = segment_end.x;
        double y4 = segment_end.y;

        // 计算行列式
        double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // 如果行列式为0，说明直线平行或重合，无交点
        if (std::abs(denominator) < 1e-6) {
            return nullptr;
        }

        // 计算交点
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

        // 如果t >= 0（射线的方向）且0 <= u <= 1（在线段上），则交点有效
        if (t >= 0 && u >= 0 && u <= 1) {
            double px = x1 + t * (x2 - x1);
            double py = y1 + t * (y2 - y1);

            return std::make_shared<carla::geom::Location>(static_cast<float>(px), static_cast<float>(py), ray_start.z);
        }

        return nullptr;  // 没有有效交点
    }

    std::shared_ptr<carla::geom::Location> FindIntersection(
        const carla::geom::Location& start,
        const carla::geom::Vector3D& direction,
        const std::vector<std::pair<carla::geom::Location, carla::geom::Location>>& edges) {
        carla::geom::Location closest_intersection;
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto& edge : edges) {
            auto intersection = CalculateLineSegmentIntersection(start, direction, edge.first, edge.second);
            if (intersection) {
                double distance = start.Distance(*intersection);
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_intersection = *intersection;
                    found = true;
                }
            }
        }

        if (found) {
            return std::make_shared<carla::geom::Location>(closest_intersection);
        }
        return nullptr;
    }

    // 计算 AABB（用于快速排除）
    AABB compute_AABB(const std::vector<std::vector<double>>& polygon) {
        AABB box;
        box.min_x = box.min_y = std::numeric_limits<double>::max();
        box.max_x = box.max_y = std::numeric_limits<double>::lowest();

        for (const auto& point : polygon) {
            box.min_x = std::min(box.min_x, point[0]);
            box.max_x = std::max(box.max_x, point[0]);
            box.min_y = std::min(box.min_y, point[1]);
            box.max_y = std::max(box.max_y, point[1]);
        }
        return box;
    }

    // 计算 AABB 是否相交（快速排除）
    bool is_AABB_overlapping(const AABB& a, const AABB& b) {
        return !(a.max_x < b.min_x || a.min_x > b.max_x ||
                a.max_y < b.min_y || a.min_y > b.max_y);
    }
  
    // 🚀 主要函数：检查 route_bb 和 tar_corners 是否相交
    std::pair<bool, uint32_t> isBoxIntersectingPath(const std::vector<std::vector<double>>& route_bb, 
                                                    const std::vector<std::vector<double>>& tar_bb) {
        if (route_bb.size() < 3 || tar_bb.size() < 3) {
            return {false, 0}; // 无效输入
        }

        // 先进行 AABB 快速排除**
        AABB aabb_route = compute_AABB(route_bb);
        AABB aabb_tar = compute_AABB(tar_bb);
        if (!is_AABB_overlapping(aabb_route, aabb_tar)) {
            return {false, 0};  // 直接排除
        }

        // 再使用 SAT 进行精确检测**
        bool result = is_polygon_intersect(route_bb, tar_bb);
        return {result, 0};  // 返回结果
    }

    Polygon GetPolygon(const std::vector<carla::geom::Location>& boundary) {
        Polygon polygon;
        for (const auto& point : boundary) {
            bg::append(polygon.outer(), Point(point.x, point.y));
        }
        bg::correct(polygon);  // 确保多边形的正确性（闭合、顺时针检查）
        return polygon;
    }
    /**
     * @brief 将路径转换为多边形
     * @param path 路径点列表
     * @param width 路径宽度（米）
     * @return 表示路径的多边形
     */
    Polygon PathToPolygon(const std::vector<carla::geom::Transform>& path, double width) {
        std::vector<Point> left_side, right_side;

        for (const auto& tf : path) {
            auto forward = tf.GetForwardVector().MakeSafeUnitVector(1e-6f);
            auto right = tf.GetRightVector().MakeSafeUnitVector(1e-6f);

            carla::geom::Location loc = tf.location;
            carla::geom::Location left_pt = loc + carla::geom::Location(-width * right.x, -width * right.y, 0.0);
            carla::geom::Location right_pt = loc + carla::geom::Location(width * right.x, width * right.y, 0.0);

            left_side.emplace_back(left_pt.x, left_pt.y);
            right_side.emplace_back(right_pt.x, right_pt.y);
        }

        // 构建闭合 polygon（顺时针）
        Polygon poly;
        for (const auto& p : left_side)
            bg::append(poly.outer(), p);
        for (auto it = right_side.rbegin(); it != right_side.rend(); ++it)
            bg::append(poly.outer(), *it);

        // 闭合 polygon
        if (!poly.outer().empty())
            poly.outer().push_back(poly.outer().front());
        bg::correct(poly);
        return poly;
    }

    void DrawPolygon(const Polygon& poly,
                    carla::client::DebugHelper debug_helper,
                    float z_offset,
                    float thickness,
                    float duration,
                    const carla::client::DebugHelper::Color& color) 
    {
        const auto& points = poly.outer();

        if (points.size() < 2) return;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            carla::geom::Location start(points[i].get<0>(), points[i].get<1>(), z_offset);
            carla::geom::Location end(points[i + 1].get<0>(), points[i + 1].get<1>(), z_offset);

            debug_helper.DrawLine(start, end, thickness, color, duration, false);
        }
    }

    bool ShouldObeyRedLight(double probability) {
        std::uniform_real_distribution<> dis(0.0, 1.0);
        return dis(gen) < probability;  // 根据传入概率决定是否闯红灯
    }
    carla::geom::Location FindBestMidPoint(
        const std::vector<carla::geom::Transform> &ego_path,
        const std::vector<carla::geom::Transform> &target_path,
        double threshold = 1.0)
    {
        double min_distance = std::numeric_limits<double>::max();
        carla::geom::Location best_mid_point;

        for (const auto &ego_tf : ego_path) {
            const auto &ego_loc = ego_tf.location;
            for (const auto &tar_tf : target_path) {
                const auto &tar_loc = tar_tf.location;

                double dist = carla::geom::Math::Distance(ego_loc, tar_loc);

                // 如果满足距离阈值，直接返回这对的中点
                if (dist <= threshold) {
                    return carla::geom::Location(
                        0.5 * (ego_loc.x + tar_loc.x),
                        0.5 * (ego_loc.y + tar_loc.y),
                        0.5 * (ego_loc.z + tar_loc.z)
                    );
                }

                // 否则持续更新最近点对
                if (dist < min_distance) {
                    min_distance = dist;
                    best_mid_point = carla::geom::Location(
                        0.5 * (ego_loc.x + tar_loc.x),
                        0.5 * (ego_loc.y + tar_loc.y),
                        0.5 * (ego_loc.z + tar_loc.z)
                    );
                }
            }
        }

        // 没有任何点对在阈值内，返回最近的中点
        return best_mid_point;
    }
}