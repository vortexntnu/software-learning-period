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
        pose_msg.position.x = 5;
        pose_msg.position.y = 1;
        pose_msg.position.z = 0;

        pose_msg.quaternion.x = 0;
        pose_msg.quaternion.y = 0;
        pose_msg.quaternion.z = 0;
        pose_msg.quaternion.w = 0;
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