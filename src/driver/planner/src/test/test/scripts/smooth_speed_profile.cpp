#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

// ===============================
// CubicSpline1D 类：一维三次样条
// ===============================
class CubicSpline1D {
public:
    std::vector<double> x, a, b, c, d;  // 系数
    bool is_valid = false;

    CubicSpline1D() = default;

    CubicSpline1D(const std::vector<double>& x_in, const std::vector<double>& y_in) {
        compute(x_in, y_in);
    }

    void compute(const std::vector<double>& x_in, const std::vector<double>& y_in) {
        int n = x_in.size();
        if (n < 3) return;
        x = x_in;
        a = y_in;
        b.resize(n);
        c.resize(n);
        d.resize(n);

        std::vector<double> h(n - 1);
        for (int i = 0; i < n - 1; ++i)
            h[i] = x[i + 1] - x[i];

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
        Eigen::VectorXd B = Eigen::VectorXd::Zero(n);

        A(0, 0) = 1.0;
        A(n - 1, n - 1) = 1.0;
        for (int i = 1; i < n - 1; ++i) {
            A(i, i - 1) = h[i - 1];
            A(i, i) = 2.0 * (h[i - 1] + h[i]);
            A(i, i + 1) = h[i];
            B(i) = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
        }

        Eigen::VectorXd c_vec = A.colPivHouseholderQr().solve(B);
        for (int i = 0; i < n; ++i) c[i] = c_vec[i];

        for (int i = 0; i < n - 1; ++i) {
            d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
            b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
        }
        is_valid = true;
    }

    // 求 y(t)
    double calc_y(double t) const {
        if (!is_valid) return 0.0;
        int i = search_segment(t);
        double dx = t - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

private:
    int search_segment(double t) const {
        if (t <= x.front()) return 0;
        if (t >= x.back()) return x.size() - 2;
        auto it = std::upper_bound(x.begin(), x.end(), t);
        return std::max(int(it - x.begin()) - 1, 0);
    }
};

// ===============================
// 主函数：样条平滑 + 数值导数
// ===============================
int main() {
    std::string input_file  = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_modified.txt";
    std::string output_file = "/home/bob/文档/备份/demo05/src/test/txt/qp_speed_smoothed.txt";

    std::ifstream ifs(input_file);
    if (!ifs.is_open()) {
        std::cerr << "❌ Failed to open " << input_file << std::endl;
        return -1;
    }

    std::vector<double> t_raw, s_raw;
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        double ti, si, vi, ai;
        if (iss >> ti >> si >> vi >> ai) {
            t_raw.push_back(ti);
            s_raw.push_back(si);
        }
    }
    ifs.close();

    if (t_raw.size() < 5) {
        std::cerr << "❌ Too few data points (" << t_raw.size() << ")" << std::endl;
        return -1;
    }

    // === 1️⃣ 从起点开始每隔 0.5s 取一个点 ===
    std::vector<double> t_sparse, s_sparse;
    double next_t = t_raw.front();
    for (size_t i = 0; i < t_raw.size(); ++i) {
        if (t_raw[i] >= next_t || i == t_raw.size() - 1) {
            t_sparse.push_back(t_raw[i]);
            s_sparse.push_back(s_raw[i]);
            next_t += 0.1;
        }
    }

    // === 2️⃣ 样条平滑 s=f(t) ===
    CubicSpline1D spline(t_sparse, s_sparse);
    if (!spline.is_valid) {
        std::cerr << "❌ Failed to create spline." << std::endl;
        return -1;
    }

    // === 3️⃣ 生成平滑后的 s(t) 每隔 0.02s ===
    std::vector<double> t_dense, s_dense;
    for (double ti = t_sparse.front(); ti <= t_sparse.back(); ti += 0.02) {
        t_dense.push_back(ti);
        s_dense.push_back(spline.calc_y(ti));
    }

    // === 4️⃣ 用数值差分计算 v(t), a(t) ===
    std::vector<double> v_dense(s_dense.size(), 0.0);
    std::vector<double> a_dense(s_dense.size(), 0.0);

    for (size_t i = 1; i < s_dense.size() - 1; ++i) {
        double dt = t_dense[i + 1] - t_dense[i - 1];
        v_dense[i] = (s_dense[i + 1] - s_dense[i - 1]) / dt;
    }
    // 边界使用前向/后向差分
    v_dense[0] = (s_dense[1] - s_dense[0]) / (t_dense[1] - t_dense[0]);
    v_dense.back() = (s_dense.back() - s_dense[s_dense.size() - 2]) /
                     (t_dense.back() - t_dense[t_dense.size() - 2]);

    for (size_t i = 1; i < v_dense.size() - 1; ++i) {
        double dt = t_dense[i + 1] - t_dense[i - 1];
        a_dense[i] = (v_dense[i + 1] - v_dense[i - 1]) / dt;
    }
    a_dense[0] = (v_dense[1] - v_dense[0]) / (t_dense[1] - t_dense[0]);
    a_dense.back() = (v_dense.back() - v_dense[v_dense.size() - 2]) /
                     (t_dense.back() - t_dense[t_dense.size() - 2]);

    // === 5️⃣ 保存结果 ===
    std::ofstream ofs(output_file);
    if (!ofs.is_open()) {
        std::cerr << "❌ Failed to open " << output_file << std::endl;
        return -1;
    }

    ofs << "# Cubic Spline Smoothed Speed Profile (t, s, v, a)\n";
    for (size_t i = 0; i < t_dense.size(); ++i) {
        ofs << t_dense[i] << " " << s_dense[i] << " " << v_dense[i] << " " << a_dense[i] << "\n";
    }
    ofs.close();

    std::cout << "✅ Spline-smoothed profile exported to " << output_file << std::endl;
    std::cout << "   Total points: " << t_dense.size() << " (Δt = 0.02s)\n";
    return 0;
}
