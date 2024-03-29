#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int64
from geometry_msgs.msg  import Pose, PoseStamped

import numpy as np

class Talker:
    """
    This is a class docstring, and should explain what the class is
    supposed to do/be used for, and any attributes it may have.

    This class implements a version of the ROS tutorial "talker", and
    implements a node that publishes a PoseStamped at a set rate.
    """

    def __init__(self, frequency=1):
        """
        This is a docstring, and should usually be included with functions, 
        especially if the contents is non-trivial. See below for the general
        structure of the docstring.

        Args:
                frequency: the frequency of which the node should publish data

        Note:
                self is not added as an argument in the Args list, since it is 
                required anyways by th
        """
        self.seq = 0
        self.rate = rospy.Rate(frequency)
        self.h = 0.01*frequency # Base update step size for the pose increments

        self.pos = [0, 0, 0]
        self.q = [0, 0, 0, 1]

        self.seq_pub = rospy.Publisher("seq_py", Int64, queue_size=1)
        self.random_pose_pub = rospy.Publisher("pose_py", PoseStamped, queue_size=10)

    def random_walk(self):
        """
        Increment the published pose data using a uniform distribution.
        Also normalizes the class quaternion.
        
        Note:
                Using a uniform dist. for random walk will not lead to natural motion!
                Don't use this for anything other than an example :)
        """
        for i in range(len(self.pos)):
            self.pos[i] += np.random.uniform(-self.h, self.h)

        for i in range(len(self.q)):
            self.q[i] += np.random.uniform(-0.1*self.h, 0.3*self.h)

        self.q /= np.linalg.norm(self.q)

        

    def spin(self):
        """
        Contains a while loop that takes the class seq and pos/q,
        and publishes them as an Int64 and PoseStamped, respectively.

        Note:
                Blocking call on not rospy.is_shutdown()
        """
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

    rospy.init_node("talker_py")

    talker_frequency = 20

    talker = Talker(talker_frequency)
    talker.spin()
