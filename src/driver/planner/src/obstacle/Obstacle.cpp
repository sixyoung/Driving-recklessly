#include "Obstacle.h"
#include "path_struct.h"
#include <string>

namespace planner
{
//在有效范围内的所有障碍物
PPoint front_left;
PPoint back_left;
PPoint back_right;
PPoint front_right;
// 求障碍物顶点
PPoint ob_left_front;
PPoint ob_left_buttom;
PPoint ob_right_front;
PPoint ob_right_buttom;

GetObstacle::GetObstacle(bool is_use_carla, std::string role_name, const Param_Configs& cfg) : Config_(&cfg), oba(cfg)
{
  ros::NodeHandle nh_;
  AllObstacle.clear();
  ros::param::get("Eff_length", eff_length);
  ros::param::get("Eff_width", eff_width);
  ros::param::get("Eff_dis", eff_dis);
  // 订阅感知的障碍物
  if (is_use_carla == false)
  {
    obstacle = nh_.subscribe<object_msgs::DynamicObjectArray>("/xsj/obstacle/obstacles", 10, &GetObstacle::setObstacles, this);
  }
  else
  {
    obstacle = nh_.subscribe<derived_object_msgs::ObjectArray>("/carla/" + role_name + "/objects", 10, &GetObstacle::setObstaclesFromCarla, this);
  }
}

// 回调，获得障碍物信息
/*
说明：这里的障碍物信息可能根你们感知模块传过来的不一样
*/
void GetObstacle::setObstacles(const object_msgs::DynamicObjectArray::ConstPtr &msgs)
{
  /*---------------------------------------获取障碍物信息----------------------------------------*/
  if (msgs->objects.size() > 0) // 如果感知有识别到障碍物
  {
    AllObstacle.clear(); // 每次接受障碍物的时候，清空，更新
    // 存到AllObstacle
    for (size_t i = 0; i < msgs->objects.size(); i++)
    {
      Obstacle obs(*Config_);
      /*-------------------------------------每种形状都有的基本信息----------------------------------------*/
      // 中心点
      obs.centerpoint.position.x = msgs->objects[i].state.pose_covariance.pose.position.x;
      obs.centerpoint.position.y = msgs->objects[i].state.pose_covariance.pose.position.y;
      obs.centerpoint.position.z = 0; // 压缩二维
      // std::cout << "position:" << obs.centerpoint.position.x << "," << obs.centerpoint.position.y<< "\n";
      obs.obstacle_id = msgs->objects[i].id;              // id
      obs.obstacle_type = msgs->objects[i].semantic.type; // 类型
      obs.obstacle_shape = msgs->objects[i].shape.type;   // 形状
      // 朝向(要根感知确认一下)
      obs.obstacle_threa = tf::getYaw(msgs->objects[i].state.pose_covariance.pose.orientation);
      // 时间戳
      obs.timestamp_ = msgs->header.stamp;
      // 线速度
      obs.obstacle_velocity = msgs->objects[i].state.twist_covariance.twist.linear.x;
      // 加速度
      obs.obstacle_acc = 0;
      // 动态障碍物预测轨迹
      if (obs.obstacle_velocity > 0.1) // 动态障碍物
      {
        oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                 Config_->dynamic_obs_predict_time, obs.obstacle_threa, obs.obstacle_velocity, obs.obstacle_acc);
      }
      /*--------------------------------------不同形状有差别的信息-----------------------------------------*/

      if (msgs->objects[i].shape.type == 0) // 方形
      {
        obs.pinnacle.poses.clear(); // 顶点暂时为空
        obs.obstacle_radius =
            sqrt(pow(msgs->objects[i].shape.dimensions.x, 2) + pow(msgs->objects[i].shape.dimensions.y, 2)) / 2;
        // 长和宽
        obs.obstacle_length = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      else if (msgs->objects[i].shape.type == 1) // 圆柱体
      {
        obs.obstacle_radius = msgs->objects[i].shape.dimensions.x; // 半径
        // 顶点
        obs.pinnacle.poses.clear();
        PPoint ob_center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        oba.CalculateCarBoundaryPoint(1, 1, ob_center, obs.obstacle_threa, ob_left_front, ob_left_buttom,
                                      ob_right_buttom, ob_right_front);
        // oba.visualization_points(ob_left_front, ob_left_buttom, ob_right_buttom, ob_right_front);//显示障碍物顶点

        std::vector<PPoint> ob_vector;
        ob_vector.emplace_back(ob_right_front);  // 右下角
        ob_vector.emplace_back(ob_left_buttom);  // 右上角
        ob_vector.emplace_back(ob_left_front);   // 左上角
        ob_vector.emplace_back(ob_right_buttom); // 左下角
        for (size_t j = 0; j < ob_vector.size(); j++)
        {
          geometry_msgs::Pose sds;
          sds.position.x = ob_vector[j].x;
          sds.position.y = ob_vector[j].y;
          sds.position.z = 0; // 压缩二维
          obs.pinnacle.poses.push_back(sds);
          obs.polygon_points.push_back(Vec2d(ob_vector[j].x, ob_vector[j].y));
        }
        // 长和宽（假设）
        obs.obstacle_length = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      else if (msgs->objects[i].shape.type == 2) // 多边形:未知
      {
        // 顶点
        obs.pinnacle.poses.clear();
        for (size_t j = 0; j < msgs->objects[i].shape.footprint.points.size(); j++)
        {
          geometry_msgs::Pose sds;
          sds.position.x = msgs->objects[i].shape.footprint.points[j].x;
          sds.position.y = msgs->objects[i].shape.footprint.points[j].y;
          sds.position.z = 0; // 压缩二维
          obs.pinnacle.poses.push_back(sds);
          obs.polygon_points.push_back(Vec2d(msgs->objects[i].shape.footprint.points[j].x, msgs->objects[i].shape.footprint.points[j].y));
        }
        // 半径为0
        obs.obstacle_radius = 0;
        // 长和宽（假设）
        obs.obstacle_length = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      AllObstacle.push_back(obs);     
      // std::cout << "AllObstacle_size:" << AllObstacle.size() << "\n";
    }
  }
}

void GetObstacle::setObstaclesFromCarla(const derived_object_msgs::ObjectArray::ConstPtr &msgs)
{
  if (msgs->objects.size() > 0) // 如果感知有识别到障碍物
  {
    AllObstacle.clear(); // 每次接受障碍物的时候，清空，更新
    // 存到AllObstacle
    for (size_t i = 0; i < msgs->objects.size(); i++)
    {
      Obstacle obs(*Config_);
      /*-------------------------------------每种形状都有的基本信息----------------------------------------*/
      // 中心点
      obs.centerpoint.position.x = msgs->objects[i].pose.position.x;
      obs.centerpoint.position.y = msgs->objects[i].pose.position.y;
      obs.centerpoint.position.z = msgs->objects[i].pose.position.z;
      obs.obstacle_id = std::to_string(msgs->objects[i].id);
      obs.obstacle_type = msgs->objects[i].CLASSIFICATION_CAR; // 类型
      obs.obstacle_shape = msgs->objects[i].shape.BOX;         // 形状
      // 朝向(要根感知确认一下)
      obs.obstacle_threa = tf::getYaw(msgs->objects[i].pose.orientation);
      // 时间戳
      obs.timestamp_ = msgs->header.stamp;
      // 线速度,只考虑车的正前方方向的速度
      // obs.obstacle_velocity = 10*msgs->objects[i].twist.linear.x;
      obs.obstacle_velocity = std::sqrt(msgs->objects[i].twist.linear.x * msgs->objects[i].twist.linear.x + 
                                        msgs->objects[i].twist.linear.y * msgs->objects[i].twist.linear.y);
      // 加速度,只考虑车的正前方方向的加速度
      obs.obstacle_acc = msgs->objects[i].accel.linear.x;
      // std::cout << "x[" <<  obs.obstacle_id  << "]" << obs.obstacle_velocity << "\n";

      // /*--------------------------------------有效区域的构建--------------------------------------*/
      // //在有效范围内的所有障碍物
      // PPoint center(ego_pos.first + eff_dis * cos(ego_head), ego_pos.second + eff_dis * sin(ego_head));
      // oba.CalculateCarBoundaryPoint(eff_length, eff_width, center, ego_head, front_left, back_left, back_right,
      //                               front_right);
      // oba.visualization(center, ego_head, eff_length, eff_width); //显示有效区域
      // /*---------------------------------------过滤不在有效区域的障碍物-------------------------------------------*/
      // PPoint centerpoint(obs.centerpoint.position.x, obs.centerpoint.position.y);
      // if (oba.Inside_rectangle(front_left, back_left, back_right, front_right, centerpoint) == false)
      // {
      //   continue; //跳过这个障碍物，进入下一个
      // }

      // 动态障碍物预测轨迹
      if (obs.obstacle_velocity > 0.01) // 动态障碍物
      {
        // ROS_WARN("666666");
        oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                 Config_->dynamic_obs_predict_time, obs.obstacle_threa, obs.obstacle_velocity, obs.obstacle_acc);
      }
      /*-------------------------------------只考虑BOX的车----------------------------------------*/
      // 半径为0
      obs.obstacle_radius = 0;
      // 长,宽,高
      if (msgs->objects[i].shape.dimensions.size() == 3)
      {
        obs.obstacle_length = msgs->objects[i].shape.dimensions[0];//5
        obs.obstacle_width = msgs->objects[i].shape.dimensions[1];//2
        obs.obstacle_height = msgs->objects[i].shape.dimensions[2];//1
      }
      else
      {
        obs.obstacle_length = 0.1;
        obs.obstacle_width = 0.1;
        obs.obstacle_height = 0;
      }
      // 顶点
      obs.pinnacle.poses.clear();

      PPoint ob_center(obs.centerpoint.position.x, obs.centerpoint.position.y);
      oba.CalculateCarBoundaryPoint(obs.obstacle_length, obs.obstacle_width, ob_center, obs.obstacle_threa,
                                    ob_left_front, ob_left_buttom, ob_right_buttom, ob_right_front);
      // oba.visualization_points(ob_left_front, ob_left_buttom, ob_right_buttom, ob_right_front);

      std::vector<PPoint> ob_vector;
      ob_vector.emplace_back(ob_right_front);  // 右下角
      ob_vector.emplace_back(ob_left_front);   // 左上角
      ob_vector.emplace_back(ob_left_buttom);  // 右上角
      ob_vector.emplace_back(ob_right_buttom); // 左下角

      for (size_t j = 0; j < ob_vector.size(); j++)
      {
        geometry_msgs::Pose sds;
        sds.position.x = ob_vector[j].x;
        sds.position.y = ob_vector[j].y;
        sds.position.z = 0; // 压缩二维
        obs.pinnacle.poses.push_back(sds);
        obs.polygon_points.push_back(Vec2d(ob_vector[j].x, ob_vector[j].y));
      }
      AllObstacle.push_back(obs);
    }
  }
}

void Obstacle::SetSLBoundary(SL_Boundary &sl_boundary) // Lattice专用
{
  sl_boundary_ = std::move(sl_boundary);
}
void Obstacle::SetSTBoundary(ST_Boundary &st_boundary) // Lattice专用
{
  path_st_boundary_ = std::move(st_boundary);
}
const SL_Boundary &Obstacle::PerceptionSLBoundary() const
{
  return sl_boundary_;
}
const ST_Boundary &Obstacle::path_st_boundary() const
{
  return path_st_boundary_;
}

bool Obstacle::IsStatic() const
{
  return is_static == true;
}
bool Obstacle::IsVirtual() const
{
  return is_virtual == true;
}

bool Obstacle::HasTrajectory() const
{
  return !(trajectory_prediction.trajectory_point_size() == 0); // 没有预测轨迹就是静态障碍物
}

const ObjectDecisionType &Obstacle::LongitudinalDecision() const
{
  return longitudinal_decision_;
}

void Obstacle::SetLongitudinalDecision(ObjectDecisionType type)
{
  longitudinal_decision_ = type;
}

void Obstacle::SetLateralDecision()
{
  if (sl_boundary_.start_l_ > 0) // 障碍物在左边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::RIGHT_NUDGE;
  }
  if (sl_boundary_.end_l_ < 0) // 障碍物在右边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::LEFT_NUDGE;
  }
  if (sl_boundary_.end_l_ > -Config_->FLAGS_numerical_epsilon && sl_boundary_.start_l_ < Config_->FLAGS_numerical_epsilon)
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::NO_NUDGE;
  }
}

const ObjectDecisionType &Obstacle::LateralDecision() const
{
  return lateral_decision_;
}
bool Obstacle::HasLateralDecision() const
{
  return lateral_decision_.tag != DecisionTag::NOSET;
}

bool Obstacle::HasLongitudinalDecision() const
{
  return longitudinal_decision_.tag != DecisionTag::NOSET;
}

Box2d Obstacle::PerceptionBoundingBox() const
{
  // 必须在订阅之后调用
  return Box2d({centerpoint.position.x, centerpoint.position.y}, obstacle_threa, obstacle_length, obstacle_width);
}

Box2d Obstacle::GetBoundingBox(const TrajectoryPoint &point) const
{
  return Box2d({point.x, point.y}, point.theta, obstacle_length,
               obstacle_width);
}

// 获得障碍物在当前时刻的TrajectoryPoint,为了求GetBoundingBox
TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const
{
  const auto &points = trajectory_prediction.trajectory_point();
  // std::cout<<"points.size():"<<points.size()<<"\n";
  if (points.size() < 2) // 认为是静态障碍物
  {
    TrajectoryPoint point;
    point.set_x(centerpoint.position.x);
    point.set_y(centerpoint.position.y);
    point.set_z(0);
    point.set_theta(obstacle_threa);
    point.set_s(0.0);
    point.set_kappa(0.0);
    point.set_dkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }
  else // 认为是一个运动的障碍物
  {
    for (size_t i = 0; i < points.size(); i++)
    {
      if (points[i].has_path_point() == false)
      {
        std::cout << "has_path_point_:" << i << ","
                  << "x:" << points[i].x << ","
                  << "y:" << points[i].y << "\n";
      }
    }
    if (relative_time >= points.back().relative_time)
    {
      return points.back();
    }

    auto comp = [](const TrajectoryPoint &p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);
    if (it_lower == points.begin())
    {
      return points.front();
    }
    else if (it_lower == points.end())
    {
      return points.back();
    }
    return math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
  }
}

const math::Polygon2d &Obstacle::PerceptionPolygon() const
{
  return perception_polygon_;
}
}