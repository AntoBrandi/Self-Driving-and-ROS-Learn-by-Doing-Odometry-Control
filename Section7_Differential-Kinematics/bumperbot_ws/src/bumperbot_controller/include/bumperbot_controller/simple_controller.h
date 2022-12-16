#ifndef SIMPLE_CONTROLLER_H
#define SIMPLE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>


class SimpleController
{
public:
    SimpleController(const ros::NodeHandle &, double radius, double separation);

private:
    void velCallback(const geometry_msgs::Twist &);

    ros::NodeHandle nh_;
    ros::Subscriber vel_sub_;
    ros::Publisher right_cmd_pub_;
    ros::Publisher left_cmd_pub_;

    // Differential Kinematics
    Eigen::Matrix2d speed_conversion_;
};
#endif 
