#pragma once
#include <string>

// dodge the obstacle in lateral direction when driving
class ObjectNudge
{
public:
  enum class Type
  {
    LEFT_NUDGE = 1,  //  障碍物标签为向左微调ObjectNudge::LEFT_NUDGE，并且无人车确实被障碍物阻挡
    RIGHT_NUDGE = 2, // drive from the right side of the obstacle
    NO_NUDGE = 3,    // No nudge is set.
  };
  Type type = Type::NO_NUDGE;

  std::string TypeName() const
  {
    if (type == Type::LEFT_NUDGE)
    {
      return "LEFT_NUDGE";
    }
    else if (type == Type::RIGHT_NUDGE)
    {
      return "RIGHT_NUDGE";
    }
    else if (type == Type::NO_NUDGE)
    {
      return "NO_NUDGE";
    }
    return "";
  }

  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l = 2;
};

class ObjectSidePass
{
public:
  enum class Type
  {
    LEFT = 1,
    RIGHT = 2,
  };
  Type type = Type::LEFT;
};

// 停车原因枚举（可扩展）
enum class StopReasonCode {
  STOP_REASON_UNKNOWN = 0,
  STOP_REASON_OBSTACLE = 1,
  STOP_REASON_DESTINATION = 2,
  STOP_REASON_PREVIOUS_STOP = 3,
  STOP_REASON_SIGNAL = 4,
  STOP_REASON_PEDESTRIAN = 5,
  STOP_REASON_CROSSWALK = 6,
  STOP_REASON_CLEAR_ZONE = 7,
  STOP_REASON_EMERGENCY = 8,
};

struct PointENU {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// 障碍物决策标签枚举
enum class DecisionTag {
  IGNORE,
  STOP,
  FOLLOW,
  YIELD,
  OVERTAKE,
  NUDGE,
  SIDE_PASS,
  AVOID,
  NOSET
};


// ObjectStop 决策
class ObjectStop {
public:
  StopReasonCode reason_code = StopReasonCode::STOP_REASON_UNKNOWN;

  // 安全停车距离（米）
  double distance_s = 0.0;

  // 停车点位置
  PointENU stop_point;

  // 期望航向角（弧度）
  double stop_heading = 0.0;

  // 等待的障碍物 ID
  std::vector<std::string> wait_for_obstacle;

  // 工具函数
  bool HasWaitObstacle() const { return !wait_for_obstacle.empty(); }

  std::string ReasonCodeName() const {
    switch (reason_code) {
      case StopReasonCode::STOP_REASON_OBSTACLE: return "OBSTACLE";
      case StopReasonCode::STOP_REASON_DESTINATION: return "DESTINATION";
      case StopReasonCode::STOP_REASON_SIGNAL: return "SIGNAL";
      case StopReasonCode::STOP_REASON_PEDESTRIAN: return "PEDESTRIAN";
      default: return "UNKNOWN";
    }
  }
};

class ObjectYield
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
  double time_buffer = 4; // minimum time buffer required after the obstacle reaches the intersect point.
};

class ObjectFollow
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
};

class ObjectOvertake
{
public:
  double distance_s = 1; // minimum longitudinal distance in meters
  double fence_point = 2;
  double fence_heading = 3;
  double time_buffer = 4; // minimum time buffer required before the obstacle reaches the intersect point.
};

class ObjectIgnore {
public:
  // 这里没有额外字段，单纯作为一个“忽略标签”
  std::string Reason() const {
    return "IGNORE";
  }
};

//-----------------------------每个障碍物标签的类，先列机种要用到的--------------------------//
class ObjectDecisionType
{
public:
  enum class Type
  {
    OBJECT_TAG_NOT_SET = 1,
    OBJECT_TAG_HAS_SET = 2,
  };

  bool has_nudge() const
  {
    return nudge_.type != ObjectNudge::Type::NO_NUDGE;
  }
  ObjectNudge nudge() const
  {
    return nudge_;
  }

  Type object_tag_case() const
  {
    return type;
  }

  std::string TypeName() const
  {
    if (type == Type::OBJECT_TAG_NOT_SET)
    {
      return "OBJECT_TAG_NOT_SET";
    }
    else if (type == Type::OBJECT_TAG_HAS_SET)
    {
      return "OBJECT_TAG_HAS_SET";
    }
    return "";
  }

public:
  Type type = Type::OBJECT_TAG_NOT_SET;
  DecisionTag tag = DecisionTag::NOSET;

  // 决策内容
  ObjectNudge nudge_;
  ObjectFollow follow;
  ObjectYield yield;
  ObjectOvertake overtake;
  ObjectStop stop;
  ObjectIgnore ignore;

  bool has_follow() const { return tag == DecisionTag::FOLLOW; }
  bool has_yield()  const { return tag == DecisionTag::YIELD; }
  bool has_overtake() const { return tag == DecisionTag::OVERTAKE; }
  bool has_stop() const { return tag == DecisionTag::STOP; }
  bool has_ignore() const { return tag == DecisionTag::IGNORE; }

  std::string DecisionTagName() const {
    switch (tag) {
      case DecisionTag::IGNORE:   return "IGNORE";
      case DecisionTag::STOP:     return "STOP";
      case DecisionTag::FOLLOW:   return "FOLLOW";
      case DecisionTag::YIELD:    return "YIELD";
      case DecisionTag::OVERTAKE: return "OVERTAKE";
      case DecisionTag::NUDGE:    return "NUDGE";
      default:                    return "UNKNOWN";
    }
  } 
};