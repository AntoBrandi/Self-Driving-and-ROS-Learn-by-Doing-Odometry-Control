#include "bumperbot_examples/turtlesim_kinematics.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_kinematics_node");
    TurtlesimKinematics turtlesim_kinematics;
    ros::spin();

    return 0;
}