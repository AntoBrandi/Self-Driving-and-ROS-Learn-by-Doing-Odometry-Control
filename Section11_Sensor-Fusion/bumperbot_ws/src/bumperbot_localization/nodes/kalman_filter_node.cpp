#include "bumperbot_localization/kalman_filter.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nh;
    KalmanFilter filter(nh);
    ros::spin();

    return 0;
}