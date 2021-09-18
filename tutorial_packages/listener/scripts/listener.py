#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import String

class Listener:
    
    def __init__(self):

        self.py_pose_listener = rospy.Subscriber("pose_py", Pose, self.py_pose_cb, queue_size=10)
        self.cpp_pose_listener = rospy.Subscriber("pose_cpp", Pose, self.cpp_pose_cb, queue_size=10)

    
    def py_pose_cb(self, pose_msg):
        pass

    def cpp_pose_cb(self, pose_msg):
        pass

if __name__ == '__main__':

    rospy.init_node("asv_listener")

    listener = Listener()

    while not rospy.is_shutdown():
        rospy.spin()
