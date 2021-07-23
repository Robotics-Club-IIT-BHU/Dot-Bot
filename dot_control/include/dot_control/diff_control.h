/*

diff_control.h,
Main Class for differential drive of the robot

Author: Somnath Sendhil Kumar
Date  : 23 July 2021
*/

#ifndef __DIFF_CONTROL__
#define __DIFF_CONTROL__

#define LW 0
#define RW 1

#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <iostream.h>

namespace diffControl{

  class DiffControl(ros::NodeHandle* nh){
    public:
      DiffControl();
      void control_loop();
      void configure();
    private:
      std::vector<double> IMU;
      std::vector<double> encoder;
  }

  DiffControl::DiffControl(ros::NodeHandle* nh){
    
  }

};

#endif