#include <ros/ros.h>


void timerCallback(const ros::TimerEvent& event)
{
    ROS_INFO("Called timerCallback Function");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_timer_cpp");
    ros::NodeHandle nh;

    ros::Duration timer_duration(1);
    ros::Timer timer = nh.createTimer(timer_duration, timerCallback);

    ros::spin();

    return 0;
}