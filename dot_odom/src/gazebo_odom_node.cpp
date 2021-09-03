#include <ros/ros.h>
#include "dot_odom/gazebo_odom.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_odom_publish");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1) ;
  gazebo_odom::GazeboOdomPublisher gazeboOdomPublisher(nodeHandle);
  while (ros::ok())
  {
    gazeboOdomPublisher.publishOdom();
    ros::spinOnce();
    loop_rate.sleep() ;
  }
  return 0;
}
