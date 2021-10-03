#include "dot_control/omni_control.hpp"
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>



std_msgs::Float64 message;

ros::Publisher publisher;
ros::Subscriber joint_states;

long double theta_left_current;
long double theta_back_current;
long double theta_right_current;
long double theta_left_previous;
long double theta_back_previous;
long double theta_right_previous;
long double duration;
long double v_left;
long double v_back;
long double v_right;
long double x, y, theta;
long double wx, wy, wtheta;
double yaw_offset;
bool yaw_flag=true;
double xref, yref, thetaref;
using namespace std;



ros::Time timeCurrent;
ros::Time timePrevious;

namespace omnidrive{
  //instantiate various variables
  drive::drive(ros::NodeHandle& nodeHandle, double dt):n(nodeHandle){
    dt_ = dt;
    left_pub = n.advertise<std_msgs::Float64>("left_joint_position_controller/command", 1000);
    right_pub = n.advertise<std_msgs::Float64>("right_joint_position_controller/command", 1000);
    back_pub = n.advertise<std_msgs::Float64>("back_joint_position_controller/command", 1000);
    vel_sub = n.subscribe("cmd_vel", 1000,  &drive::velocity_callback,this);
    joint_states=n.subscribe("joint_states", 1, &drive::onJointStateMessage, this);
    imu_sub = n.subscribe("imu",1, &drive::imuCallBack, this);
    // gaz_sub = n.subscribe("/gazebo/model_states", 1, &drive::onGazeboMessage, this);
    // map_sub = n.subscribe("/gazebo_map/map", 1000,  &drive::updateOdom,this);
    pub_ = n.advertise<nav_msgs::Odometry>("odom", 50) ;
    // publisher = n.advertise<std_msgs::Velocity>("wheel_velocity", 1);

    lpos.data = 0;
    rpos.data = 0;
    bpos.data = 0;
    vx = 0; vy = 0; wp = 0;
    odom_theta=0;odom_x=0.25;odom_y=1;
    tfListenerObj=new tf2_ros::TransformListener(tfBuffer);
    // file_storage.open("state_log.csv", std::ios::app);
    // file_storage << "odom_x,odom_y,odom_theta,calc_x,calc_y,calc_theta" << std::endl;
  
  }
  void drive::imuCallBack(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(ro, pit, yaw);
    if(yaw_flag==true){
      yaw_flag=false;
      yaw_offset = yaw;
    }
    odom_theta = yaw - yaw_offset;
    //odom_theta = yaw;
  }
  void drive::onGazeboMessage(const gazebo_msgs::ModelStates::ConstPtr& msg){
    xref = msg->pose[msg->name.size()-1].position.x;
    yref = msg->pose[msg->name.size()-1].position.y;
     tf::Quaternion q(
        msg->pose[msg->name.size()-1].orientation.x,
        msg->pose[msg->name.size()-1].orientation.y,
        msg->pose[msg->name.size()-1].orientation.z,
        msg->pose[msg->name.size()-1].orientation.w);
    tf::Matrix3x3 m(q);
    double tp, tr;
    m.getRPY(tr, tp, thetaref);
    thetaref = fmod(thetaref,6.283);
  }

  void drive::onJointStateMessage(const sensor_msgs::JointState::ConstPtr& input){
    timeCurrent = ros::Time::now();
    double current_left_velocity, current_right_velocity, current_back_velocity;
    // for (int i = 0; i < input->name.size(); i++) {
        // if (input->name[i].c_str()[0] == 'l') {
            theta_left_current = input->position[3];
            current_left_velocity=input->velocity[3];
        // } else if (input->name[i].c_str()[0] == 'b') {
            theta_back_current = input->position[2];
            current_back_velocity=input->velocity[2];
        // } else if (input->name[i].c_str()[0] == 'r') {
            theta_right_current = input->position[4];
            current_right_velocity=input->velocity[4];
        // }
    // }
    lpos.data = theta_left_current;
    rpos.data = theta_right_current;
    bpos.data = theta_back_current;
    duration = (timeCurrent - timePrevious).toSec();
    v_left  = (theta_left_current  - theta_left_previous ) / duration;
    v_back  = (theta_back_current  - theta_back_previous ) / duration;
    v_right = (theta_right_current - theta_right_previous) / duration;
    // std::cout <<"vleft_original " <<current_left_velocity <<" vleft_calculated " << v_left<<std::endl;
    // std::cout <<"vright_original " <<current_right_velocity <<" vright_calculated " << v_right<<std::endl;
    // std::cout <<"vback_original " <<current_back_velocity <<" vback_calculated " << v_back<<std::endl;
    // message.v_left  = v_left ;
    // message.v_back  = v_back ;
    // message.v_right = v_right;
    // publisher.publish(message);
    timePrevious = timeCurrent;
    theta_left_previous  = theta_left_current ;
    theta_back_previous  = theta_back_current ;
    theta_right_previous = theta_right_current;

    long double v_left0  = v_left  * r;
    long double v_back0  = v_back  * r;
    long double v_right0 = v_right * r;
    x     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    y     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    //theta = (v_left0 + v_back0 + v_right0) / (3*L);
    
    double X = cos(odom_theta)*x - sin(odom_theta)*y;
    double Y = sin(odom_theta)*x + cos(odom_theta)*y;

    odom_x += X * duration;
    odom_y += Y * duration;
    //odom_theta += theta * duration;

    //file_storage<<xref<<","<<yref<<","<<thetaref<<","<<odom_x<<","<<odom_y<<","<<odom_theta<< std::endl;
    

    publishOdom();



}

  void drive::velocity_callback(const geometry_msgs::Twist& msg){
    //ROS_INFO_STREAM_THROTTLE(2.0,"Got new velocities from cmd_vel");
      
      vx = msg.linear.x;
      vy = msg.linear.y;
      wp = msg.angular.z;
      
  }
  //publish velocity as was stated in the /cmd_vel
  void drive::controlLoop(){

    double vmx=vx, vmy=vy, wmp=wp;

    double v1, v2, v3;
    v1 = (L * wmp - (vmx / 2) - (sqrt3by2 * vmy));
    v2 = (vmx + (L * wmp));
    v3 = (L * wmp - (vmx / 2) + (sqrt3by2 * vmy));

  
    lpos.data += 10*v1*dt_/r;
    bpos.data += 10*v2*dt_/r;
    rpos.data += 10*v3*dt_/r;

   
    left_pub.publish(lpos);
    right_pub.publish(rpos);
    back_pub.publish(bpos);


  }

  void drive::updateOdom(const nav_msgs::Odometry& msg){
    //set the current theta to zero
    odom_theta=0;odom_x=0;odom_y=0;
    //ROS_INFO_STREAM_THROTTLE(2.0,"Updated Odom");
  }

  void drive::publishOdom(){
    //based on velocity add values to theta , x , y,z and send to transform and publish odom
    // ros::Time timeCurrent = ros::Time::now();
//     //double duration = (timeCurrent - timePrevious).toNSec()/1e9;
//     float duration=0.1;
//     odom_x += vx * duration;
//     odom_y += vy * duration;
//     odom_theta += wp * duration;
    //ROS_INFO_STREAM_THROTTLE(2.0,"Odom Publish duration "<<duration<<" odom_x "<<odom_x<<" odom_y "<<odom_y<<" odom_theta "<<odom_theta);
    //transfrom using tf
//    //saving model variables
  //  tf::Quaternion tf_quat;
  //  tf_quat.setRPY(0, 0, odom_theta);
//    //tranform odom
//    geometry_msgs::TransformStamped odom_trans;
//    odom_trans.transform.translation.x = odom_x ;
//    odom_trans.transform.translation.y = odom_y ;
//    odom_trans.transform.translation.z = 0 ;
//
//    geometry_msgs::Quaternion geo_Quat ;
//    tf::quaternionTFToMsg(tf_quat, geo_Quat) ;
//    odom_trans.transform.rotation = geo_Quat ;
//    odom_trans.header.stamp = ros::Time::now() ;
//    odom_trans.header.frame_id = "odom" ;
//    odom_trans.child_frame_id = "base_link" ;
//
//    //send the transform
//    odom_broadcaster.sendTransform(odom_trans);

    //transform using tf2
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_trans;

    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, odom_theta);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "origin_link";
    odom_trans.transform.translation.x =odom_x;
    odom_trans.transform.translation.y =odom_y;
    odom_trans.transform.translation.z =0;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat=tf2::toMsg(tf_quat);
    odom_trans.transform.rotation = geo_Quat ;

    br.sendTransform(odom_trans);

    //publish odometry over ros
    nav_msgs::Odometry odom ;
    odom.header.stamp = odom_trans.header.stamp ;
    odom.header.frame_id = "odom" ;

    //set the position
    cout<<odom_trans.transform.translation.x<<" "
      <<odom_trans.transform.translation.y<<" "
      <<odom_trans.transform.translation.z<<"\n";
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = "origin_link";
    odom.twist.twist.linear.x = x ;
    odom.twist.twist.linear.y = y ;
    odom.twist.twist.linear.z = 0 ;

    odom.twist.twist.angular.x= 0 ;
    odom.twist.twist.angular.y= 0 ;
    odom.twist.twist.angular.z= theta ;

    pub_.publish(odom);
  }
  drive::~drive(){
  }
}
