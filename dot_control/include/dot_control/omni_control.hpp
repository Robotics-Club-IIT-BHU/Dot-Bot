#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <iostream>
#include <string>


#define L 0.04 // distance between body center and wheel center
#define r 0.01905 // wheel radius
#define piby30 0.1047197551196597705355242034774843062905347323976457118988037109375 // long double thirty = 30; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / thirty);
#define piby2 1.5707963267948965579989817342720925807952880859375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / two);
#define pi 3.141592653589793115997963468544185161590576171875 // long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne));
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));
#define sqrt3 1.732050807568877193176604123436845839023590087890625 // long double three = 3; printf("%1.70Lf\n", (long double) sqrt(three));
#define sqrt3by2 0.8660254037844385965883020617184229195117950439453125 // long double two = 2; long double three = 3; printf("%1.70Lf\n", (sqrt(three) / two));

namespace omnidrive{
  class drive{
   private:
    double dt_;
    double vx,vy,wp;
    ros::NodeHandle n;
    //dot controller
    ros::Publisher left_pub;
    ros::Publisher right_pub;
    ros::Publisher back_pub;
    std_msgs::Float64 lpos, rpos, bpos;
    std::ofstream file_storage;
    //map and vel subscribe
    ros::Subscriber vel_sub,map_sub, gaz_sub;
    ros::Subscriber imu_sub;
    ros::Time timePrevious;
    double ro,pit,yaw;

    //for odom
    double odom_theta,odom_x,odom_y;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher pub_;

    //tf listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener * tfListenerObj;

   public:
    drive(ros::NodeHandle& nodeHandle, double dt);
    void velocity_callback(const geometry_msgs::Twist& msg);
    void controlLoop();
    void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void onJointStateMessage(const sensor_msgs::JointState::ConstPtr& msg);
    void onGazeboMessage(const gazebo_msgs::ModelStates::ConstPtr& msg);
    ~drive();
    void updateOdom(const nav_msgs::Odometry& msg);
    void publishOdom();
  };
}
