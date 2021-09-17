#include "dot_control/omni_control.hpp"
#include <ros/ros.h>
// Equations are taken from this repo -> https://github.com/GuiRitter/OpenBase


int main(int argc, char** argv){
    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n;
    omnidrive::drive controller(n);
    ros::Rate rate(100);
    while(ros::ok()){
        //std::cout << "SENDING DATA\n";
        //ROS_INFO_STREAM_THROTTLE(2.0,"Sending Data ROS INFO");
        controller.publishVel();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
