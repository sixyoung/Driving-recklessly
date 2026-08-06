#include <ros/ros.h>
#include "behavior_identification/car_state_classify.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_state_classify_node");
    ros::NodeHandle nh;

    CarStateClassify::CarStateClassifyNode node(nh);

    ros::spin();

    return 0;
}
