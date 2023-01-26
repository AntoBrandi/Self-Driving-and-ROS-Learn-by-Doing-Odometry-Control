#!/usr/bin/env python3
import rospy
from bumperbot_examples.tf_examples import TfExamples


if __name__== '__main__':
    rospy.init_node('tf_examples')
    tfExamples = TfExamples()

    rospy.spin()