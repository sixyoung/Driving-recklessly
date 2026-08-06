#include <ros/ros.h>
#include <road_side_system_type/VehicleInfoOutarray.h>
#include <road_side_system_type/VehicleInfoOut.h>
#include <road_side_system_type/PoseWithTimeWindow.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scheduler_test_node");
    ros::NodeHandle nh;

    // 发布到集中话题 /road_side_system_out
    ros::Publisher test_pub = nh.advertise<road_side_system_type::VehicleInfoOutarray>(
        "/road_side_system_out", 1);

    ros::Rate loop_rate(1.0);  // 每秒发布一次

    while (ros::ok())
    {
        road_side_system_type::VehicleInfoOutarray out_array_msg;

        // ===== 模拟 3 辆车 =====
        for (int vid = 1; vid <= 3; ++vid)
        {
            road_side_system_type::VehicleInfoOut vinfo;
            vinfo.vehicle_id = vid;

            // ===== 模拟 3 个带时间窗的路径点 =====
            for (int i = 0; i < 3; ++i)
            {
                road_side_system_type::PoseWithTimeWindow pt;

                // 位姿
                pt.pose.position.x = 10.0 * vid + i;
                pt.pose.position.y = 5.0 * vid + i;
                pt.pose.orientation.w = 1.0;

                // 冲突区域四个角点
                geometry_msgs::Pose corner;
                for (int j = 0; j < 4; ++j) {
                    corner.position.x = pt.pose.position.x + 0.5 * std::cos(j * M_PI_2);
                    corner.position.y = pt.pose.position.y + 0.5 * std::sin(j * M_PI_2);
                    corner.orientation.w = 1.0;
                    pt.ConflictpPoint_corners.push_back(corner);
                }

                // 时间窗（开始时间 + 结束时间）
                pt.time_window.push_back(ros::Time::now());
                pt.time_window.push_back(ros::Time::now() + ros::Duration(2.0));

                vinfo.PointsWithTimeWindow.push_back(pt);
            }

            out_array_msg.VehicleInfos.push_back(vinfo);
        }

        ROS_INFO("[Test] 发布 VehicleInfoOutarray，共 %lu 辆车", out_array_msg.VehicleInfos.size());
        test_pub.publish(out_array_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
