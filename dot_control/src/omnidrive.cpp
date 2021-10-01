#include "dot_control/omni_control.hpp"
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

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
    gaz_sub = n.subscribe("/gazebo/model_states", 1, &drive::onGazeboMessage, this);
    pub_ = n.advertise<nav_msgs::Odometry>("odom", 50) ;
    // publisher = n.advertise<std_msgs::Velocity>("wheel_velocity", 1);

    lpos.data = 0;
    rpos.data = 0;
    bpos.data = 0;
    vx = 0; vy = 0; wp = 0;
    odom_theta=0;odom_x=0;odom_y=0;
    tfListenerObj=new tf2_ros::TransformListener(tfBuffer);

    //some extra variables
    ro=0;pit=0;yaw=0;

    theta_left_current=0;
    theta_back_current=0;
    theta_right_current=0;
    theta_left_previous=0;
    theta_back_previous=0;
    theta_right_previous=0;
    duration=0;
    w_left=0;
    w_back=0;
    w_right=0;
    x=0; y=0; theta=0;X=0;Y=0;
    wx=0; wy=0; wtheta=0;
    odom_yaw=0;

    xref=0; yref=0; thetaref=0;
    timeCurrent=timePrevious=ros::Time::now();

    file_storage.open("/home/state_log.csv", std::ios::app);
    file_storage << "odom_x,odom_y,odom_theta,calc_x,calc_y,calc_theta" << std::endl;
    if (!nodeHandle.getParam("enabled_localization", enabled_localization)) {
    //nodeHandle.getParam(parameter_name, variable)//value of param is trnasferred to variable
      enabled_localization=false;
    }
    enabled_localization=true;
    //ROS_INFO_STREAM("Enabled localization "<<enabled_localization);
  }
  void drive::imuCallBack(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(ro, pit, yaw);
    odom_yaw = yaw;
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
    w_left  = (theta_left_current  - theta_left_previous ) / duration;
    w_back  = (theta_back_current  - theta_back_previous ) / duration;
    w_right = (theta_right_current - theta_right_previous) / duration;
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
    //v1=>left , v2=>back , v3 => right , 10 is the scaling factor
    long double v_left0  = w_left  * r;
    long double v_back0  = w_back  * r;
    long double v_right0 = w_right * r;
    y     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    x     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    theta = (v_left0 + v_back0 + v_right0) / (3*0.04);
    ////replacing x with -ve X as compared to real formulas , needs explanation
    //X = cos(odom_yaw)*x - sin(odom_yaw)*y;
    //Y = sin(odom_yaw)*x + cos(odom_yaw)*y;
    double X = cos(odom_theta)*x + sin(odom_theta)*y;
    double Y = sin(odom_theta)*x - cos(odom_theta)*y;
    //cout<<"Before : odom_x "<<odom_x<<"odom_y "<<odom_y<<" X "<<X<<" Y "<<Y<<" x "<<x<<" y "<<y<<" theta "<<theta<<"\n";
    odom_x += X * duration;
    odom_y += Y * duration;
    odom_theta += theta * duration;
    odom_yaw += theta * duration;
    //cout<<"After : odom_x "<<odom_x<<"odom_y "<<odom_y<<"X "<<X<<"Y "<<Y<<"odom_theta "<<odom_theta<<"\n";
    file_storage<<xref<<","<<yref<<","<<thetaref<<","<<odom_x<<","<<odom_y<<","<<odom_yaw<< std::endl;
    
    publishOdom();



}

  void drive::velocity_callback(const geometry_msgs::Twist& msg){
    //ROS_INFO_STREAM_THROTTLE(2.0,"Got new velocities from cmd_vel");
      
      vx = 10*msg.linear.x;
      vy = 10*msg.linear.y;
      wp = 50*0.7143*msg.angular.z;
      //controlLoop();0
  }
  //publish velocity as was stated in the /cmd_vel
  void drive::controlLoop(){
    //double vmx=cos(yaw)*vx-sin(yaw)*vy;
    //double vmy=-sin(yaw)*vx-cos(yaw)*vy;
    //v1=>left , v2=>back , v3 => right
    
    //replacing x with -ve x as compared to real formulas , needs explanation
    //cout<<" vx "<<vx<<" vy "<<" wp "<<wp<<"\n";
    double vmx=-cos(odom_yaw)*vx+sin(odom_yaw)*vy;
    double vmy=+sin(odom_yaw)*vx+cos(odom_yaw)*vy;
    double wmp = wp ;//- yaw;
    //cout<<" Bot velocities : vmx "<<vmx<<" vmy: "<<vmy<<" wp: "<<wmp<<" odom_yaw "<<odom_yaw<<"\n";
    double v1, v2, v3;
    v1 = (L * wmp - (vmx / 2) - (sqrt3by2 * vmy));
    v2 = (vmx + L * wmp);
    v3 = (L * wmp - (vmx / 2) + (sqrt3by2 * vmy));
    //cout<<" Bot velocities : v1 "<<v1<<" v2: "<<v2<<" v3:"<<v3<<"\n";
    lpos.data += 10*v1*dt_/r;
    bpos.data += 10*v2*dt_/r;
    rpos.data += 10*v3*dt_/r;

   
    left_pub.publish(lpos);
    right_pub.publish(rpos);
    back_pub.publish(bpos);


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
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x =odom_x;
    odom_trans.transform.translation.y =odom_y;
    odom_trans.transform.translation.z =0;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat=tf2::toMsg(tf_quat);
    odom_trans.transform.rotation = geo_Quat ;

    //publish odometry over ros
    nav_msgs::Odometry odom ;
    odom.header.stamp = odom_trans.header.stamp ;
    odom.header.frame_id = "odom" ;

    //set the position
    //cout<<odom_trans.transform.translation.x<<" "<<odom_trans.transform.translation.y<<" "<<odom_trans.transform.translation.z<<"\n";
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = X ;
    odom.twist.twist.linear.y = Y ;
    odom.twist.twist.linear.z = 0 ;

    odom.twist.twist.angular.x= 0 ;
    odom.twist.twist.angular.y= 0 ;
    odom.twist.twist.angular.z= theta ;

    pub_.publish(odom);
    //if(enabled_localization)
    //   return;
    br.sendTransform(odom_trans);
  }
  drive::~drive(){
  }
}
