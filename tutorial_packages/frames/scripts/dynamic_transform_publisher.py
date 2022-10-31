#!/usr/bin/python3

import numpy as np

import rospy

import tf_conversions

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')

    tf_broadcaster = tf2_ros.TransformBroadcaster() # The broadcaster object used to publish the tf
    transform = geometry_msgs.msg.TransformStamped() # The tf

    start = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        time_since_start = rospy.Time.now().to_sec() - start

        theta = time_since_start / 2.0

        transform.header.stamp = rospy.Time.now()

        # TODO: populate the transform.header.frame_id and transform.child_frame_id according to spec

        # TODO: populate the transform.transform.translation. fields (x, y, z)

        # TODO: use the tf_conversions euler to quaternion function to generate a quaternion according to spec

        # TODO: populate the transform.transform.rotation fields (x, y, z, w) with the quaternion from last step

        # TODO: publish transform

        rospy.sleep(0.1)

