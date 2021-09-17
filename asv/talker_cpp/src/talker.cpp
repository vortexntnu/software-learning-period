#include "talker_cpp/talker.hpp"


Talker::Talker(ros::NodeHandle nh) {

    seq_pub = nh.advertise<std_msgs::Int64>("talker_cpp_seq", 10);

    random_odom_pub = nh.advertise<nav_msgs::Odometry>("talker_cpp_odom", 10);

    odom_reply_sub = nh.subscribe("odom_reply", 10, &Talker::odom_reply_cb, this);

    seq = 0;
}

void Talker::odom_reply_cb(const nav_msgs::Odometry &odom_msg) {

}

void Talker::spin() {
    ros::Rate loop_rate(frequency);

    while(ros::ok) {


        std_msgs::Int64 seq_msg;
        seq_msg.data = seq;
        seq_pub.publish(seq_msg);

        seq++;

        ros::spinOnce();
        loop_rate.sleep();
    }
}