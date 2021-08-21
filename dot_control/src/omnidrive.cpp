#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iostream>

// Equations are taken from this repo -> https://github.com/GuiRitter/OpenBase
double vx = 1, vy = 0, wp = 0, L = 1;

void velocity_callback(const geometry_msgs::Twist& msg){
    vx = msg.linear.x;
    vy = msg.linear.y;
    wp = msg.angular.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "omnidrive");
    ros::NodeHandle n;

    ros::Publisher left_pub = n.advertise<std_msgs::Float64>("dot/left_joint_position_controller/command", 1000);
    ros::Publisher right_pub = n.advertise<std_msgs::Float64>("dot/right_joint_position_controller/command", 1000);
    ros::Publisher back_pub = n.advertise<std_msgs::Float64>("dot/back_joint_position_controller/command", 1000);

    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 1000, velocity_callback);
    
    ros::Rate rate(10);

    std_msgs::Float64 lpos, rpos, bpos;
    lpos.data = 0;
    rpos.data = 0;
    bpos.data = 0;

    while(ros::ok()){

        std::cout << "SENDING DATA\n";
        double v1, v2, v3;
        v1 = (L * wp - (vx / 2) - (sqrt(3) * vy / 2));
        v2 = vx + L * wp;
        v3 = L * wp - (vx / 2) + (sqrt(3) * vy / 2);

        lpos.data += v1;
        bpos.data += v2;
        rpos.data += v3;

        left_pub.publish(lpos);
        right_pub.publish(rpos);
        back_pub.publish(bpos);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
