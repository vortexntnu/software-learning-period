#include "talker_cpp/talker.hpp"


Talker::Talker(ros::NodeHandle nh) {

    seq = 0;
    random_pose_pub = nh.advertise<geometry_msgs::Pose>("pose_cpp", 10);
    seq_pub = nh.advertise<std_msgs::Int64>("seq_cpp", 10);
}

void Talker::spin() {
    ros::Rate loop_rate(frequency);

    while(ros::ok()) {

        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = pos[0];
        pose_msg.position.y = pos[1];
        pose_msg.position.z = pos[2];

        pose_msg.orientation.x = q[0];
        pose_msg.orientation.y = q[1];
        pose_msg.orientation.z = q[2];
        pose_msg.orientation.w = q[3];
        random_pose_pub.publish(pose_msg);

        std_msgs::Int64 seq_msg;
        seq_msg.data = seq;
        seq_pub.publish(seq_msg);


        seq++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return;
}