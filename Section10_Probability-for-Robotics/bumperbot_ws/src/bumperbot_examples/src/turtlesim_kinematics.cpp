#include "bumperbot_examples/turtlesim_kinematics.h"


TurtlesimKinematics::TurtlesimKinematics()
{
    turtle1_pose_sub_ = nh_.subscribe("/turtle1/pose", 1000, 
        &TurtlesimKinematics::turtle1PoseCallback, this);
    turtle2_pose_sub_ = nh_.subscribe("/turtle2/pose", 1000, 
        &TurtlesimKinematics::turtle2PoseCallback, this);
}


void TurtlesimKinematics::turtle1PoseCallback(const turtlesim::Pose& pose)
{
    last_turtle1_pose_ = pose;
}


void TurtlesimKinematics::turtle2PoseCallback(const turtlesim::Pose& pose)
{
    last_turtle2_pose_ = pose;
    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;
    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = 180 * theta_rad / 3.14;
    ROS_INFO_STREAM("------------------------------------\n" <<
                    "Translation Vector turtle1 -> turtle2 \n" <<
                    "Tx: " << Tx << "\n" <<
                    "Ty: " << Ty << "\n" <<
                    "Rotation Matrix turtle1 -> turtle2 \n" << 
                    "theta (rad): " << theta_rad << "\n" <<
                    "theta (deg): " << theta_deg << "\n" <<
                    "|R11   R12|:  |" << std::cos(theta_rad) << "\t" << -std::sin(theta_rad) << "|\n" <<
                    "|R21   R22|   |" << std::sin(theta_rad) << "\t\t" << std::cos(theta_rad) << "|\n");
}