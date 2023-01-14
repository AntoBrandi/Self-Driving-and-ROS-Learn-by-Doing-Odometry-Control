#ifndef TURTLESIM_KINEMATICS_H
#define TURTLESIM_KINEMATICS_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>

class TurtlesimKinematics
{
public:
    TurtlesimKinematics();

    void turtle1PoseCallback(const turtlesim::Pose& pose);

    void turtle2PoseCallback(const turtlesim::Pose& pose);

private:
    ros::NodeHandle nh_;
    ros::Subscriber turtle1_pose_sub_;
    ros::Subscriber turtle2_pose_sub_;

    turtlesim::Pose last_turtle1_pose_;
    turtlesim::Pose last_turtle2_pose_;
};

#endif // TURTLESIM_KINEMATICS_H