#!/usr/bin/env python

import rospy

from std_msgs.msg import Int64, String
from geometry_msgs.msg  import Pose, PoseStamped

from tf.transformations import quaternion_from_euler

import numpy as np

class Talker:

    def __init__(self, frequency=1):
        self.seq = 0
        self.rate = rospy.Rate(frequency)
        self.h = 0.01*frequency

        self.pos = [0, 0, 0]
        self.q = [0, 0, 0, 1]

        self.seq_pub = rospy.Publisher("seq_py", Int64, queue_size=1)
        self.random_pose_pub = rospy.Publisher("pose_py", PoseStamped, queue_size=10)

    def random_walk(self):
        for i in range(len(self.pos)):
            self.pos[i] += np.random.uniform(-self.h, self.h)

        for i in range(len(self.q)):
            self.q[i] += np.random.uniform(-0.1*self.h, 0.3*self.h)

        self.q /= np.linalg.norm(self.q)

        

    def spin(self):
        while not rospy.is_shutdown():

            
            pose_msg = Pose()
            pose_msg.position.x = self.pos[0]
            pose_msg.position.y = self.pos[1]
            pose_msg.position.z = 0

            pose_msg.orientation.x = self.q[0]
            pose_msg.orientation.y = self.q[1]
            pose_msg.orientation.z = self.q[2]
            pose_msg.orientation.w = self.q[3]


            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.seq = self.seq
            pose_stamped_msg.header.frame_id = "map"
            pose_stamped_msg.header.stamp = rospy.Time.now()
            pose_stamped_msg.pose = pose_msg

            self.random_pose_pub.publish(pose_stamped_msg)

            seq_msg = Int64()
            seq_msg.data = self.seq
            self.seq_pub.publish(seq_msg)

            self.seq += 1
            self.random_walk()

            self.rate.sleep()



if __name__ == '__main__':

    rospy.init_node("asv_talker_py")

    talker = Talker(20)
    talker.spin()
