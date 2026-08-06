/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * Modified function input and used only some functions
 **/
#include "dp_road_graph.h"
#include <fstream>
#include "file_exporter.h"
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>
DpRoadGraph::DpRoadGraph(const SpeedData &speed_data, const ReferenceLine &reference_line,
                         const nav_msgs::Odometry vehicle_pos, const Param_Configs &cfg)
    : speed_data_(speed_data), reference_line_(reference_line), vehicle_pos_(vehicle_pos), Config_(&cfg)

{
  // 调节车的长宽
  vehicle_width_ = Config_->FLAGS_vehicle_width;
  vehicle_length_ = Config_->FLAGS_vehicle_length;
}
DpRoadGraph::DpRoadGraph()
{
}
DpRoadGraph::~DpRoadGraph()
{
}

// 传入：起点，障碍物，要获取的路径信息
/*
  函数FindPathTunnel：
 * 主要分为3部分：
 * 1、设置相关前提条件，
 * 2、查找代价最小路径，
 * 3、最后对每段最小代价curve离散化，结果存入path_data中
 */
bool DpRoadGraph::FindPathTunnel(const TrajectoryPoint &init_point, const std::vector<const Obstacle *> &obstacles,
                                 PathData *const path_data)
{
  // 1.找起始点所对应的在参考线上的点(s,l)
  init_point_ = init_point;
  if (!reference_line_.XYToSL(init_point_.path_point(), &init_sl_point_))
  {
    std::cout << "Fail to create init_sl_point";
    return false;
  }
  init_frenet_frame_point_ = reference_line_.GetFrenetPoint(init_point_.path_point());

  // 2.查找代价最小路径的核心在于GenerateMinCostPath()，也是分为3部分：
  // 先采样，然后构造graph，最后查找从起点（自车当前位置）到终点（尽可能远的某个采样点）的代价最小路径。
  VehicleConfig vehicle_config(vehicle_width_, vehicle_length_, init_point_.v, vehicle_pos_);

  waypoint_sampler_->Init(vehicle_config, &reference_line_, init_sl_point_, init_frenet_frame_point_);

  std::vector<DpRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path))
  {
    ROS_INFO("[%s]:Fail to generate graph!", Config_->role_name_.c_str());
    return false;
  }

  // 3.将最优前进路线封装成path_data

  std::vector<FrenetFramePoint> frenet_path;
  double accumulated_s = min_cost_path.front().sl_point.s; // 第一个点是规划起点

  //  离散化存储DP点,每两个点之间都是五次多项式曲线
  for (size_t i = 1; i < min_cost_path.size(); ++i) // 第一个点是规划起点
  {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

    const double path_length = cur_node.sl_point.s - prev_node.sl_point.s;
    double current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (current_s + Config_->path_resolution / 2.0 < path_length)
    {
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
      FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += Config_->path_resolution;
    }
    if (i == min_cost_path.size() - 1)
    {
      accumulated_s += current_s;
    }
    else
    {
      accumulated_s += path_length;
    }
  }
  // std::cout << "frenet_path:" << frenet_path.size() << "\n";
  // 旧版本:  path_data->Clear();
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(FrenetFramePath(std::move(frenet_path)));

  return true;
}

void DpRoadGraph::ExportDpNodeCost(
    const std::list<std::list<DpRoadGraphNode>> &graph_nodes,
    const std::string &filename,
    size_t frame_id,
    bool append_mode,
    bool split_mode,
    size_t frames_per_file)
{
    try
    {
        std::string target_file = filename;

        // ✅ 分帧保存
        if (split_mode)
        {
            size_t file_index = frame_id / frames_per_file;
            std::filesystem::path fpath(filename);
            std::string stem = fpath.stem().string();
            std::string ext = fpath.extension().string();
            std::string parent = fpath.has_parent_path() ? fpath.parent_path().string() : ".";
            target_file = parent + "/" + stem + "_" + std::to_string(file_index) + ext;
        }
        std::filesystem::path fpath(target_file);
        if (fpath.has_parent_path()) {
            std::filesystem::create_directories(fpath.parent_path());
        }

        std::ofstream ofs(target_file, append_mode ? std::ios::app : std::ios::out);
        if (!ofs.is_open())
        {
            std::cerr << "❌ Failed to open DpNodeCost file: " << target_file << std::endl;
            return;
        }

        // 当前时间
        std::time_t t = std::time(nullptr);
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        ofs << "==================== Frame " << frame_id
            << " @ " << time_buf << " ====================\n";

        size_t level = 0;
        for (const auto &level_nodes : graph_nodes)
        {
            ofs << "# Level " << level << " | Node count: " << level_nodes.size() << "\n";
            ofs << "s\tl\tsafety_cost\tsmoothness_cost\n";

            for (const auto &node : level_nodes)
            {
                ofs << std::fixed << std::setprecision(6)
                    << node.sl_point.s << "\t"
                    << node.sl_point.l << "\t"
                    << node.min_cost.safety_cost << "\t"
                    << node.min_cost.smoothness_cost<< "\n";
            }

            ofs << "--------------------------------------\n";
            ++level;
        }

        ofs << "\n";
        ofs.close();
        // std::cout << "✅ DpNodeCost exported to: " << target_file << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Exception in ExportDpNodeCost: " << e.what() << std::endl;
    }
}

/*
  函数GenerateMinCostPath：
  * 主要分为3部分：
  *1、先采样，撒点得到path_waypoints，调用SamplePathWaypoints()方法完成
  *2、然后构造graph，即forward过程
  *3、backward过程，顺藤摸瓜得到cost最小路径
 */
bool DpRoadGraph::GenerateMinCostPath(const std::vector<const Obstacle *> &obstacles,
                                      std::vector<DpRoadGraphNode> *min_cost_path)
{
  std::vector<std::vector<SLPoint>> path_waypoints;
  // 1.先采样，撒点得到path_waypoints，调用SamplePathWaypoints()方法完成
  if (!waypoint_sampler_->SamplePathWaypoints(init_point_, &path_waypoints) || path_waypoints.size() < 1)
  {
    ROS_INFO("Fail to sample path waypoints! reference_line_length");
    return false;
  }

  // ============================== 保存到文件 ==============================
  // 采样完 path_waypoints 后导出
  if(Config_->role_name_ == "hero0" && 0){
    planner::FileExporter::ExportPathWaypointsSample(
        path_waypoints,
        reference_line_,
        "/home/bob/文档/备份/demo05/src/test/txt/path_waypoints_samp/path_waypoints_samp.txt",
        Config_->frame_id,   // frame_id
        true,   // append
        true,   // split by frame
        300     // 每300帧一个文件
    );
  }
  // ============================== 保存到文件 ==============================

  // 2.计算代价
  VehicleConfig vehicle_config(vehicle_width_, vehicle_length_, init_point_.v, vehicle_pos_);

  // 这里的speed_data_就是计算 DynamicObstacleCost时需要的 heuristic_speed_data_
  TrajectoryCost trajectory_cost(vehicle_config, reference_line_, false, obstacles, speed_data_, init_sl_point_, *Config_);

  // 3.存储构建的graph，速度DP过程用的数据结构是vector<vector<>>，路径这里是list<list<>>
  std::list<std::list<DpRoadGraphNode>> graph_nodes;

  // 从撒点得到的path_waypoints的第一纵列中，找到nearest_i
  const auto &first_row = path_waypoints.front();
  size_t nearest_i = 0;

  // 在第一行（横向）中寻找最靠近起始点的node，忽略了其他点
  // 如果以当前所在点为起始点的话，就没必要在所在横向上采样其他点了
  for (size_t i = 1; i < first_row.size(); ++i)
  {
    if (std::fabs(first_row[i].l - init_sl_point_.l) < std::fabs(first_row[nearest_i].l - init_sl_point_.l))
    {
      nearest_i = i;
    }
  }
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(first_row[nearest_i], nullptr, ComparableCost()); // first_row[nearest_i]就是起点
  auto &front = graph_nodes.front().front();                                        // 规划起始点：init_point
  size_t total_level = path_waypoints.size();
  // 构建 graph_nodes
  // 两层循环:
  //        外循环 -- 撒点的列数；
  //        内循环 -- 列中的每个点；
  for (size_t level = 1; level < path_waypoints.size(); ++level)
  {
    const auto &prev_dp_nodes = graph_nodes.back();   // 前一层level
    const auto &level_points = path_waypoints[level]; // 当前层level中的所有横向采样点(类似神经元)

    graph_nodes.emplace_back();
    // std::vector<std::future<void>> results;
    for (size_t i = 0; i < level_points.size(); ++i) // 计算当前层level中与前一层所有计算的连接权值，也就是cost
    {
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);

      auto msg = std::make_shared<RoadGraphMessage>(prev_dp_nodes, level, total_level, &trajectory_cost, &front,
                                                    &(graph_nodes.back().back()));

      // 更新node的cost，更新后，该node保存了起点到该点的最小代价、prev node、
      // prev node ---> node间的5次polynomial curve
      UpdateNode(msg);
    }
  }
  if(Config_->role_name_ ==  "hero0" && 0){
    DpRoadGraph::ExportDpNodeCost(
        graph_nodes,
        "/home/bob/文档/备份/demo05/src/test/txt/dp_node_cost/dp_node_cost.txt",
        Config_->frame_id,  // 当前帧号
        true,               // 追加模式
        true,               // 分文件保存
        300                 // 每300帧一个文件
    );    
  }
  // 4. find best path
  DpRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) // 选择最后一个level中，具有最小cost的node最为规划终点
  {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve, cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  // Todo:min_cost_path没有把fake_head保存进去，fake_head->min_cost_prev_node才是真正的终点
  while (min_cost_node->min_cost_prev_node) // 当父亲节点存在
  {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);

  }
  if (min_cost_node != &graph_nodes.front().front())
  {
      ROS_WARN("[%s]:❌ 回溯失败: 最优路径的起点 != 规划起点", Config_->role_name_.c_str());
      const auto& start = graph_nodes.front().front();
      ROS_WARN("[%s]:规划起点(s=%.3f, l=%.3f)", Config_->role_name_.c_str(), start.sl_point.s, start.sl_point.l);
      ROS_WARN("[%s]:回溯起点(s=%.3f, l=%.3f)", Config_->role_name_.c_str(), min_cost_node->sl_point.s, min_cost_node->sl_point.l);

      return false;
  }
  std::reverse(min_cost_path->begin(), min_cost_path->end());

  // std::cout << "min_cost_path:" << min_cost_path->size() << "\n";
  return true;
}
/*
  函数：UpdateNode
 * 更新当前节点的父节点prev node 和 cost
 * 1、在current node与任一prev node间
 * 2、以及current node与first node间，构造5次polynomial curve；
 * 3、计算这2个node间的cost，如果小于min_cost,则更新cur_node的cost并更新连接关系
 */
// void DpRoadGraph::UpdateNode(const std::shared_ptr<RoadGraphMessage> &msg)
// {
//   // if (msg == nullptr)
//   //   return;
//   // if (msg->trajectory_cost == nullptr)
//   //   return;
//   // if (msg->front == nullptr)
//   //   return;
//   // if (msg->cur_node == nullptr)
//   //   return;
//   for (const auto &prev_dp_node : msg->prev_nodes)
//   {
//     const auto &prev_sl_point = prev_dp_node.sl_point;
//     const auto &cur_point = msg->cur_node->sl_point;
//     double init_dl = 0.0;
//     double init_ddl = 0.0;
//     if (msg->level == 1)
//     {
//       // 仅自车当前姿态有dl（角度朝向）,ddl，其余点的dl,ddl都是0
//       init_dl = init_frenet_frame_point_.d_d;   // 导数
//       init_ddl = init_frenet_frame_point_.d_dd; // 导数的导数

//       // std::cout << "init_d:" << init_frenet_frame_point_.d << " "
//       //           << "init_dl:" << init_dl << " "
//       //           << "init_ddl:" << init_ddl << "\n";
//     }
//     // 1D quintic polynomial curve:
//     // (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
//     // BVP问题：已知两端点的 值、一阶导、二阶导，即可求得5次多项式
//     // 只有level==1时，用车辆的位姿信息设定 dl、ddl,其他情况都是0，因为每个level之间的Δs还是挺大的，近似为0问题也不大
//     QuinticPolynomialCurve1d curve(prev_sl_point.l, init_dl, init_ddl, cur_point.l, 0.0, 0.0,
//                                    cur_point.s - prev_sl_point.s);

//     if (!IsValidCurve(curve)) // 剪枝，单位s内，l变化太大即为非法
//     {
//       continue;
//     }
//     const auto cost =
//         msg->trajectory_cost->Calculate(curve, prev_sl_point.s, cur_point.s, msg->level, msg->total_level) +
//         prev_dp_node.min_cost;
//     // 如果与最新的prev_dp_node之间的cost更小，会更新连接关系
//     msg->cur_node->UpdateCost(&prev_dp_node, curve, cost);
//   }

//   // 有换道 且 level>2,尝试与起点连接
//   if (Config_->IsChangeLanePath && msg->level >= 2)
//   {
//     const double init_dl = init_frenet_frame_point_.d_d;   // 导数
//     const double init_ddl = init_frenet_frame_point_.d_dd; // 导数的导数
//     QuinticPolynomialCurve1d curve(init_sl_point_.l, init_dl, init_ddl, msg->cur_node->sl_point.l, 0.0, 0.0,
//                                    msg->cur_node->sl_point.s - init_sl_point_.s);
//     if (!IsValidCurve(curve))
//     {
//       return;
//     }
//     const auto cost = msg->trajectory_cost->Calculate(curve, init_sl_point_.s, msg->cur_node->sl_point.s, msg->level,
//                                                       msg->total_level);
//     msg->cur_node->UpdateCost(msg->front, curve, cost);
//   }
// }
void DpRoadGraph::UpdateNode(const std::shared_ptr<RoadGraphMessage> &msg)
{
  for (const auto &prev_dp_node : msg->prev_nodes)
  {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = msg->cur_node->sl_point;
    double init_dl = 0.0;
    double init_ddl = 0.0;
    if (msg->level == 1)
    {
      init_dl = init_frenet_frame_point_.d_d;   // 自车起点横向速度
      init_ddl = init_frenet_frame_point_.d_dd; // 自车起点横向加速度
    }

    double delta_s = cur_point.s - prev_sl_point.s;

    QuinticPolynomialCurve1d curve(prev_sl_point.l, init_dl, init_ddl,
                                   cur_point.l, 0.0, 0.0,
                                   delta_s);

    if (!IsValidCurve(curve)) // 剪枝
    {
      ROS_WARN("❌ Invalid curve: prev(s=%.3f,l=%.3f) -> cur(s=%.3f,l=%.3f), Δs=%.3f",
               prev_sl_point.s, prev_sl_point.l,
               cur_point.s, cur_point.l,
               delta_s);
      continue;
    }

    const auto cost =  msg->trajectory_cost->Calculate(curve,
                                        prev_sl_point.s,
                                        cur_point.s,
                                        msg->level,
                                        msg->total_level) +
        prev_dp_node.min_cost;
    msg->cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }

  // 有换道 且 level>2,尝试与起点连接
  if (Config_->IsChangeLanePath && msg->level >= 2)
  {
    const double init_dl = init_frenet_frame_point_.d_d;
    const double init_ddl = init_frenet_frame_point_.d_dd;
    double delta_s = msg->cur_node->sl_point.s - init_sl_point_.s;

    QuinticPolynomialCurve1d curve(init_sl_point_.l, init_dl, init_ddl,
                                   msg->cur_node->sl_point.l, 0.0, 0.0,
                                   delta_s);

    if (!IsValidCurve(curve))
    {
      ROS_WARN("❌ Invalid direct-connection curve: init(s=%.3f,l=%.3f) -> cur(s=%.3f,l=%.3f), Δs=%.3f",
               init_sl_point_.s, init_sl_point_.l,
               msg->cur_node->sl_point.s, msg->cur_node->sl_point.l,
               delta_s);
      return;
    }

    const auto cost = msg->trajectory_cost->Calculate(curve,
                                                      init_sl_point_.s,
                                                      msg->cur_node->sl_point.s,
                                                      msg->level,
                                                      msg->total_level);

    if (std::isnan(cost.safety_cost) || std::isnan(cost.smoothness_cost)) {
      ROS_WARN("⚠️ Cost=NaN/Inf! Direct: init(s=%.3f,l=%.3f) -> cur(s=%.3f,l=%.3f), Δs=%.3f, "
               "init_dl=%.6f, init_ddl=%.6f",
               init_sl_point_.s, init_sl_point_.l,
               msg->cur_node->sl_point.s, msg->cur_node->sl_point.l,
               delta_s, init_dl, init_ddl);
    } 
    msg->cur_node->UpdateCost(msg->front, curve, cost);
  }
}

// 单位S内，L变化太大即为非法
bool DpRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const
{
  static constexpr double kMaxLateralDistance = 20.0;
  for (double s = 0.0; s < curve.ParamLength(); s += 2.0)
  {
    const double l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance)
    {
      return false;
    }
  }
  return true;
}

void DpRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost, const QuinticPolynomialCurve1d &curve,
                               const double start_s, const double end_s, const uint32_t curr_level,
                               const uint32_t total_level, ComparableCost *cost)
{
  *cost = trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}