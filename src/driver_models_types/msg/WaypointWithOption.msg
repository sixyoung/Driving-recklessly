geometry_msgs/Pose pose  # 路径点的位姿
int32 road_option        # 路径点的路况类型（RoadOption 枚举值）  
  
# enum class RoadOption {
#         VOID = -1,             // 无效选项
#         LEFT = 1,              // 左转
#         RIGHT = 2,             // 右转
#         STRAIGHT = 3,          // 直行
#         LANEFOLLOW = 4,        // 跟随车道
#         CHANGELANELEFT = 5,    // 左换道
#         CHANGELANERIGHT = 6    // 右换道
#     };
