#include <ros/ros.h>
#include "dot_map/gazebo_map.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_map_publish");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1) ;
  gazebo_map::GazebomapPublisher gazebomapPublisher(nodeHandle);
  while (ros::ok())
  {
    gazebomapPublisher.publishmap();
    ros::spinOnce();
    loop_rate.sleep() ;
  }
  return 0;
}
