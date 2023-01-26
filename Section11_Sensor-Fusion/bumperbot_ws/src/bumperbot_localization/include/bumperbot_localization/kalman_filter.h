#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>


class KalmanFilter
{
public:
    KalmanFilter(const ros::NodeHandle &);

    void statePrediction();

    void measurementUpdate();


private:
    void odomCallback(const nav_msgs::Odometry &);

    void imuCallback(const sensor_msgs::Imu &);


    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    double mean_;
    double variance_;
    double motion_variance_;
    double measurement_variance_;
    double motion_;
    bool is_first_odom_;
    double last_angular_z_;
    double imu_angular_z_;
    nav_msgs::Odometry kalman_odom_;
};
