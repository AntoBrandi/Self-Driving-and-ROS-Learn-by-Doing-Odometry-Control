#!/usr/bin/env python3
import rospy
from bumperbot_examples.srv import GetTransform, GetTransformResponse
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse


class TfExamples(object):

    def __init__(self):
        self.x_increment_ = 0.05
        self.last_x_ = 0.0
        self.rotations_counter_ = 0

        # TF Broadcaster
        self.static_broadcaster_ = StaticTransformBroadcaster()
        self.dynamic_broadcaster_ = TransformBroadcaster()
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        # TF Listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_)

        self.static_transform_stamped_.header.stamp = rospy.Time.now()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0
        self.static_transform_stamped_.transform.rotation.y = 0
        self.static_transform_stamped_.transform.rotation.z = 0
        self.static_transform_stamped_.transform.rotation.w = 1

        self.static_broadcaster_.sendTransform(self.static_transform_stamped_)
        rospy.loginfo("Publishing static transform between %s and %s", 
                      self.static_transform_stamped_.header.frame_id,
                      self.static_transform_stamped_.child_frame_id)

        # Timer
        self.timer_ = rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        
        # Service Server
        self.get_transform_srv_ = rospy.Service("get_transform", GetTransform, self.getTransformCallback)

        # Quaternion
        self.last_orientation_ = quaternion_from_euler(0, 0, 0)
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05)
    

    def timerCallback(self, event):
        self.dynamic_transform_stamped_.header.stamp = rospy.Time.now()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        # self.dynamic_transform_stamped_.transform.rotation.x = 0
        # self.dynamic_transform_stamped_.transform.rotation.y = 0
        # self.dynamic_transform_stamped_.transform.rotation.z = 0
        # self.dynamic_transform_stamped_.transform.rotation.w = 1

        # Euler to Quaternion
        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        self.dynamic_broadcaster_.sendTransform(self.dynamic_transform_stamped_)
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x
        self.last_orientation_ = q
        self.rotations_counter_ += 1
        if self.rotations_counter_ >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotations_counter_ = 0

    
    def getTransformCallback(self, req):
        rospy.loginfo("Requested Transform between %s and %s", req.frame_id, req.child_frame_id)
        res = GetTransformResponse()
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rospy.Time())
        except Exception as e:
            rospy.logerr("An error occurred while transforming %s and %s: %s",
                         req.frame_id, req.child_frame_id, e)
            res.success = False
            return res

        rospy.loginfo("The requested transform is: %s", requested_transform)
        res.transform = requested_transform
        res.success = True
        return res
