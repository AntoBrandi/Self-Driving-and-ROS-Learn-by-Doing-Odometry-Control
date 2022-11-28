#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(msg):
    # This function is called each time a new message is published on the topic /chatter
    # The message that has been published is then passed as input to this function
    rospy.loginfo("New message received: %s", msg.data)
   
    
if __name__ == "__main__":
    # Inizialize a ROS node called listener
    rospy.init_node("simple_subscriber_py", anonymous=True)

    # register a subscriber on the topic /chatter that will listen for String messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("chatter", String, callback)

    # keeps the node up and running
    rospy.spin()