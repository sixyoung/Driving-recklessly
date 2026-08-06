#include "auxiliary_function.h"


//返回：最近的点
std::pair<double, double>
Auxiliary::Find_nearest_piont(std::pair<double, double> ObastaclePoint,
                              std::pair<std::vector<double>, std::vector<double>> reference_path_, int& index_ob)
{
  std::pair<double, double> nearest;
  int index = 0;
  double min_val = 1000;
  double distance;
  for (size_t i = 0; i < reference_path_.first.size(); i++)
  {
    distance = sqrt(pow(ObastaclePoint.first - reference_path_.first[i], 2) +
                    pow(ObastaclePoint.second - reference_path_.second[i], 2));
    if (distance < min_val)
    {
      min_val = distance;  //更新
      index = i;
    }
  }
  index_ob = index;
  nearest.first = reference_path_.first[index];
  nearest.second = reference_path_.second[index];
  return nearest;
}

// 根据障碍物点 (x, y)、参考点 (x_ref, y_ref, rtheta, s_ref) 计算障碍物的 s
double Auxiliary::find_s(const std::pair<double, double>& obstacle_point,
                            const std::pair<double, double>& rxy,
                            double rtheta,
                            double s_ref) {
    // Δx, Δy
    double dx = obstacle_point.first - rxy.first;
    double dy = obstacle_point.second - rxy.second;

    // 切向
    double tx = std::cos(rtheta);
    double ty = std::sin(rtheta);

    // Δs = Δ · t
    double delta_s = dx * tx + dy * ty;

    // 返回目标点的 s
    return s_ref + delta_s;
}


//传入：障碍物中心点ObastaclePoint， 最近的参考点rxy
//输出：横向偏移
double Auxiliary::cartesian_to_frenet(std::pair<double, double> obstacle_point,
                                      std::pair<double, double> rxy,
                                      double rtheta) {
  // Δx, Δy
  const double dx = obstacle_point.first - rxy.first;
  const double dy = obstacle_point.second - rxy.second;

  // 参考点的法向量 (左法向)
  const double nx = -std::sin(rtheta);
  const double ny =  std::cos(rtheta);

  // 横向偏移量 = 点到参考点向量在法向上的投影
  double l = dx * nx + dy * ny;

  return l;
}