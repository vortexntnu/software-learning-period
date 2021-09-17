#include "talker_cpp/talker.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker_cpp");
    ros::NodeHandle nh;
    Talker talker(nh);

    talker.spin();
    
    return 0;
}