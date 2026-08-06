#ifndef MISC_H
#define MISC_H

#include <geometry_msgs/Pose.h>
#include <carla/client/DebugHelper.h>
#include <carla/geom/Transform.h> 
#include <carla/geom/Location.h>
#include <carla/geom/Vector3D.h> 
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>
#include <random> 
#include <chrono>  
#include <cmath>
#include <limits>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point> Polygon;

namespace common {
    enum Side {
        LEFT,
        RIGHT,
        FRONT,
        BACK
    };
    // 初始化随机数种子
    inline std::mt19937 gen(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

    // 计算轴对齐包围盒 (AABB)
    struct AABB {
        double min_x, max_x, min_y, max_y;
    };
    AABB compute_AABB(const std::vector<std::vector<double>>& polygon);

    bool is_AABB_overlapping(const AABB& a, const AABB& b);

    bool is_polygon_intersect(const std::vector<std::vector<double>>& route_bb, 
                           const std::vector<std::vector<double>>& target_bb);

    std::pair<bool, std::vector<Point>> CheckPolygonIntersection(
        const Polygon& poly1, const Polygon& poly2);

    std::pair<bool, uint32_t> isBoxIntersectingPath(const std::vector<std::vector<double>>& route_bb, 
                                                    const std::vector<std::vector<double>>& tar_corners);
    bool is_within_distance_and_angle(
            const carla::geom::Transform& current_transform,
            const carla::geom::Transform& target_transform,
            const double max_distance,
            const double min_angle_deg,  // 最小角度（度）
            const double max_angle_deg);  // 最大角度（度）
    

    double calculate_angle(const carla::geom::Vector3D& vec1, const carla::geom::Vector3D& vec2);
    Side detect_vehicle_side(const carla::geom::Transform& ego_transform, const carla::geom::Transform& target_transform, double angle_threshold = 45.0);

    // 绕z轴旋转函数
    template <typename T>
    carla::geom::Vector3D RotatePoint(const carla::geom::Vector3D& point, const T& radians) {
        // 使用传入的角度类型（T）进行旋转
        T rotated_x = cos(radians) * point.x - sin(radians) * point.y;
        T rotated_y = sin(radians) * point.x + cos(radians) * point.y;
        return carla::geom::Vector3D(rotated_x, rotated_y, point.z);
    }

    double distance_vehicle(const geometry_msgs::Point& position1, const geometry_msgs::Point& position2);

    std::pair<bool, bool> is_vehicle_intersect( const carla::geom::Transform& current_transform,
    const carla::geom::Transform& target_transform, const carla::geom::Transform& ego_tar_wp_transform);
    
    std::vector<carla::geom::Location> GetRectangleCorners(const carla::geom::Transform& transform, float length_x, float length_y);
    void PrintRectangleCorners(const std::vector<carla::geom::Location>& corners);
    void GetProjectionRange(const std::vector<carla::geom::Location>& corners, const carla::geom::Vector3D& edge, float& min_proj, float& max_proj);
    bool CheckProjectionOverlap(const carla::geom::Transform& box1_transform, float box1_length_x, float box1_length_y,
                                const carla::geom::Transform& box2_transform, float box2_length_x, float box2_length_y);
    std::shared_ptr<carla::geom::Location> CalculateLineSegmentIntersection(
            const carla::geom::Location& ray_start,
            const carla::geom::Vector3D& ray_direction,
            const carla::geom::Location& segment_start,
            const carla::geom::Location& segment_end);

    std::shared_ptr<carla::geom::Location> FindIntersection(
            const carla::geom::Location& start,
            const carla::geom::Vector3D& direction,
            const std::vector<std::pair<carla::geom::Location, carla::geom::Location>>& edges);

    Polygon GetPolygon(const std::vector<carla::geom::Location>& boundary);

    Polygon PathToPolygon(const std::vector<carla::geom::Transform>& path, double width);

    void DrawPolygon(const Polygon& poly,
                        carla::client::DebugHelper debug_helper,
                        float z_offset = 0.5f,
                        float thickness = 0.1f,
                        float duration = 5.0f,
                        const carla::client::DebugHelper::Color& color = {0, 255, 0});

    template<typename T1, typename T2>
    double GetMinBoxDistance(const T1& box1, const T2& box2) {
        Polygon poly1, poly2;

        if constexpr (std::is_same<T1, std::vector<carla::geom::Location>>::value) {
            poly1 = GetPolygon(box1);
        } else {
            poly1 = box1;
        }

        if constexpr (std::is_same<T2, std::vector<carla::geom::Location>>::value) {
            poly2 = GetPolygon(box2);
        } else {
            poly2 = box2;
        }

        return bg::distance(poly1, poly2);
    }
        
    bool ShouldObeyRedLight(double probability);
    carla::geom::Location FindBestMidPoint(
        const std::vector<carla::geom::Transform> &ego_path,
        const std::vector<carla::geom::Transform> &target_path,
        double threshold);

}

#endif //MISC_H