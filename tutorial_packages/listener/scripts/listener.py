#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose, PoseStamped

class Listener:
    
    def __init__(self):

        self.py_pose_listener = rospy.Subscriber("pose_py", PoseStamped, self.py_pose_cb, queue_size=10)
        self.cpp_pose_listener = rospy.Subscriber("pose_cpp", Pose, self.cpp_pose_cb, queue_size=10)

    
    def py_pose_cb(self, pose_msg):
        """
        This is a callback that has been attached to the self.py_pose_listener 
        subscriber. A callback is a function that is given its own thread, and is
        called whenever a message is published on the topic the Subscriber subscribes
        to, in this case "pose_py"

        All this function does is print the contents of the message.

        Args:
                pose_msg: A PoseStamped message
        """
        rospy.loginfo("Python talker sent the message: {}".format(pose_msg))


    def cpp_pose_cb(self, pose_msg):
        """
        This is a callback that has been attached to the self.cpp_pose_listener 
        subscriber. A callback is a function that is given its own thread, and is
        called whenever a message is published on the topic the Subscriber subscribes
        to, in this case "pose_cpp"

        All this function does is print the contents of the message.

        Args:
                pose_msg: A Pose message
        """
        rospy.loginfo("C++ talker sent the message: {}".format(pose_msg))

if __name__ == '__main__':

    rospy.init_node("asv_listener")

    listener = Listener()

    while not rospy.is_shutdown():
        rospy.spin()
