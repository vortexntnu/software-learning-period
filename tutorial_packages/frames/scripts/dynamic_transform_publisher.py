#!/usr/bin/python3

import numpy as np


import rospy

import tf_conversions

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    start = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        time_since_start = rospy.Time.now().to_sec() - start

        theta = time_since_start / 2.0

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        
        transform.transform.translation.x = np.cos(theta)
        transform.transform.translation.y = np.sin(theta)
        transform.transform.translation.z = 0.0

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)

        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        tf_broadcaster.sendTransform(transform)
        rospy.sleep(0.1)

