#include <ros/ros.h>
#include "behavior_identification/identification.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "identification_node");
    ros::NodeHandle nh;

    behavior_identification::BehaviorIdentification node(nh);

    ros::spin();

    // 当 ros::spin() 退出（比如收到 SIGINT，或 ROS 停止）时，生成最后的图表
    std_msgs::String msg;
    msg.data = "STOP";
    node.scenarioStatusCallback(boost::make_shared<std_msgs::String>(msg));

    return 0;
}
