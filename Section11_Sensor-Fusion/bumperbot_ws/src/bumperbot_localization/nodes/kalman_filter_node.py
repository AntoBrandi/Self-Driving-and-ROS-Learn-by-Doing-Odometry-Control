#!/usr/bin/env python3
import rospy
from bumperbot_localization.kalman_filter import KalmanFilter


if __name__== '__main__':
    rospy.init_node('kalman_filter_node')
    filter = KalmanFilter()
    
    rospy.spin()