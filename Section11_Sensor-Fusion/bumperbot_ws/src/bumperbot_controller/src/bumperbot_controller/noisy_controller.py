#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf_conversions
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class NoisyController(object):

    def __init__(self, wheel_radius, wheel_separation):
        rospy.loginfo("Using wheel radius %d" % wheel_radius)
        rospy.loginfo("Using wheel separation %d" % wheel_separation)
        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.joint_sub_ = rospy.Subscriber("joint_states", JointState, self.jointCallback)        
        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom_noisy", Odometry, queue_size=10)

        # Fill the Odometry Noisy message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF Noisy message
        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = rospy.Time.now()


    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF

        # Add noise to wheel readings
        wheel_encoder_left = msg.position[0] + np.random.normal(0, 0.05)
        wheel_encoder_right = msg.position[1] + np.random.normal(0, 0.05)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = (msg.header.stamp - self.prev_time_).to_sec()

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = wheel_encoder_left
        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.prev_time_ = msg.header.stamp

        # Calculate the rotational speed of each wheel
        fi_left = dp_left / dt
        fi_right = dp_right / dt

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Compose and publish the odom message
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()
        self.br_.sendTransform(self.transform_stamped_)