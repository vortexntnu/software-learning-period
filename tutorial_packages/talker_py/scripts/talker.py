#!/usr/bin/env python

import rospy

from std_msgs.msg import Int64, String
from geometry_msgs.msg  import Pose

class Talker:

    def __init__(self, frequency=1):
        self.seq = 0
        self.rate = rospy.Rate(frequency)

        self.seq_pub = rospy.Publisher("seq_py", Int64, queue_size=1)
        self.random_pose_pub = rospy.Publisher("pose_py", Pose, queue_size=10)

    def spin(self):
        while not rospy.is_shutdown():

            pose_msg = Pose()
            pose_msg.position.x = 10
            self.random_pose_pub.publish(pose_msg)

            seq_msg = Int64()
            seq_msg.data = self.seq
            self.seq_pub.publish(seq_msg)

            self.seq += 1
            self.rate.sleep()


if __name__ == '__main__':

    rospy.init_node("asv_talker_py")

    talker = Talker(2)
    talker.spin()
