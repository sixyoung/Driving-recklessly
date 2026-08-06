#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include "CubicSpline2D.h"   // 你自己的 CubicSpline2D 类头文件
#include "path_matcher.h"   // 你自己的 PathMatcher 类头文件

int main() {
    // === 1. 随机生成离散数据点 ===
    std::vector<double> xs, ys;
    std::random_device rd;
    std::mt19937 gen(rd());  
    std::uniform_real_distribution<> dist(0.1, 1.0); // 间距范围 [0.1, 1.0] m

    double x = 0.0;
    while (x <= 10.0) { // 总长大约 10m
        double y = std::sin(x * 0.5) * 2.0; // 加一点波动
        xs.push_back(x);
        ys.push_back(y);

        double dx = dist(gen); // 生成一个随机间距
        x += dx;
    }

    // === 2. 计算弧长参数 s ===
    CubicSpline2D spline;
    std::vector<double> s = spline.calc_s(xs, ys);

    // === 3. 构造样条对象 ===
    CubicSpline2D csp(xs, ys, s);

    // === 4. 保存原始点 ===
    std::ofstream ofs_raw("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/raw_path.txt");
    if (!ofs_raw.is_open()) {
        std::cerr << "❌ Failed to open raw_path.txt" << std::endl;
        return -1;
    }
    for (size_t i = 0; i < xs.size(); ++i) {
        ofs_raw << s[i] << " " << xs[i] << " " << ys[i] << "\n";
    }
    ofs_raw.close();
    std::cout << "✅ Raw path exported to raw_path.txt" << std::endl;

    // === 5. 平滑插值，采样更多点 ===
    std::ofstream ofs_smooth("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/smoothed_path.txt");
    if (!ofs_smooth.is_open()) {
        std::cerr << "❌ Failed to open smoothed_path.txt" << std::endl;
        return -1;
    }
    std::vector<std::pair<double,double>> xy_points;

    double ds = 0.1; // 插值采样间隔
    for (double si = s.front(); si <= s.back(); si += ds) {
        double x = csp.calc_x(si);
        double y = csp.calc_y(si);
        xy_points.emplace_back(x, y);

        double yaw = csp.calc_yaw(si);         // 如果 CubicSpline2D 提供
        double kappa = csp.calc_curvature(si); // 如果 CubicSpline2D 提供

        ofs_smooth << si << " " << x << " " << y
                   << " " << yaw << " " << kappa << "\n";
    }

    ofs_smooth.close();
    std::cout << "✅ Smoothed path exported to smoothed_path.txt" << std::endl;
    
    // === 6. 用 PathMatcher 计算 heading / kappa 等 ===
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulated_s;

    if (!PathMatcher::ComputePathProfile(xy_points, &headings,
                                        &accumulated_s, &kappas, &dkappas))
    {
        std::cerr << "❌ ReferenceLine generate failed in ComputePathProfile!" << std::endl;
        return -1;
    }

    // === 7. 保存 profile 数据 ===
    std::ofstream ofs_profile("/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/profile_path.txt");
    if (!ofs_profile.is_open()) {
        std::cerr << "❌ Failed to open profile_path.txt" << std::endl;
        return -1;
    }
    for (size_t i = 0; i < accumulated_s.size(); ++i) {
        ofs_profile << accumulated_s[i] << " " 
                    << xy_points[i].first << " " 
                    << xy_points[i].second << " "
                    << headings[i] << " "
                    << kappas[i] << "\n";
    }
    ofs_profile.close();
    std::cout << "✅ Profile path exported to profile_path.txt" << std::endl;

    return 0;
}
