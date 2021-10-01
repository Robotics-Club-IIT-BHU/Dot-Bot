#include "dot_map/gazebo_map.hpp"
#include <cmath>
#include <algorithm>

namespace gazebo_map {
  GazebomapPublisher::GazebomapPublisher(ros::NodeHandle &nodeHandle):nodeHandle_(nodeHandle){
    pub_ = nodeHandle_.advertise<nav_msgs::Odometry>("map", 50) ;
    modelName_= (std::string)"dot" ;
    relativeEntityName_ = (std::string)"nosignal_ps1.world";
    client_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
    if (!nodeHandle.getParam("enabled_localization", enabled_localization)) {
      //nodeHandle.getParam(parameter_name, variable)//value of param is trnasferred to variable
      enabled_localization=false;
    }
    enabled_localization=true;
    tfListenerObj=new tf2_ros::TransformListener(tfBuffer);
  }
  void GazebomapPublisher::publishmap(){
    //get model state from gazebo
    getModelState_.request.model_name = modelName_ ;
    getModelState_.request.relative_entity_name = relativeEntityName_ ;
    client_.call(getModelState_) ;
    //saving model variables
    pp_ = getModelState_.response.pose.position ;
    qq_ = getModelState_.response.pose.orientation ;
    tf2::Quaternion tf_quat(qq_.x, qq_.y, qq_.z, qq_.w) ;
    current_Twist_ = getModelState_.response.twist ;

    //we get transform from base_link to map
    geometry_msgs::TransformStamped map_trans;

    map_trans.header.stamp = ros::Time::now();
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "base_link";

    map_trans.transform.translation.x =pp_.x;
    map_trans.transform.translation.y =pp_.y;
    map_trans.transform.translation.z =pp_.z;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat=tf2::toMsg(tf_quat) ;
    map_trans.transform.rotation = geo_Quat ;
    getMap2Odom(map_trans);


  }
  void GazebomapPublisher::getMap2Odom(geometry_msgs::TransformStamped Map2BaseLink){
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform("odom","base_link", ros::Time(0));

      tf2::Quaternion q1(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);
      tf2::Quaternion q2(Map2BaseLink.transform.rotation.x,Map2BaseLink.transform.rotation.y,Map2BaseLink.transform.rotation.z,Map2BaseLink.transform.rotation.w);

      geometry_msgs::TransformStamped map_trans;
      map_trans.header.stamp = Map2BaseLink.header.stamp;
      map_trans.header.frame_id = "map";
      map_trans.child_frame_id = "odom";

      map_trans.transform.translation.x =Map2BaseLink.transform.translation.x-transformStamped.transform.translation.x;
      map_trans.transform.translation.y =Map2BaseLink.transform.translation.y-transformStamped.transform.translation.y;
      map_trans.transform.translation.z =Map2BaseLink.transform.translation.z-transformStamped.transform.translation.z;

      tf2::Quaternion q3=q2*q1.inverse();
      map_trans.transform.rotation=tf2::toMsg(q3);
      sendTransform(map_trans);
      //ROS_INFO_STREAM_THROTTLE(2.0,"Sent transform");
    } catch (tf2::TransformException &exception) {
      //ROS_WARN("%s", exception.what());
      return;
    }
  }
  void GazebomapPublisher::sendTransform(geometry_msgs::TransformStamped Map2BaseLink){
    //transform using tf
//    //tranform map
//    geometry_msgs::TransformStamped map_trans;
//    map_trans.transform.translation.x = pp_.x ;
//    map_trans.transform.translation.y = pp_.y ;
//    map_trans.transform.translation.z = pp_.z ;
//
//    geometry_msgs::Quaternion geo_Quat ;
//    tf::quaternionTFToMsg(tf_quat, geo_Quat) ;
//    map_trans.transform.rotation = geo_Quat ;
//
//    map_trans.header.stamp = ros::Time::now() ;
//    map_trans.header.frame_id = "map" ;
//    map_trans.child_frame_id = "odom" ;
//
//    //send the transform
//    map_broadcaster_.sendTransform(map_trans);

    //publish mapetry over ros
    nav_msgs::Odometry map ;
    map.header.stamp = Map2BaseLink.header.stamp ;
    map.header.frame_id = "map" ;

    //set the position
    map.pose.pose.position.x = Map2BaseLink.transform.translation.x ;
    map.pose.pose.position.y = Map2BaseLink.transform.translation.y ;
    map.pose.pose.position.z = Map2BaseLink.transform.translation.z ;
    map.pose.pose.orientation = Map2BaseLink.transform.rotation ;

    //set the velocity - we cant determine the velocities from camera
    map.child_frame_id = "odom";
    //map.twist.twist.linear.x = current_Twist_.linear.x ;
    //map.twist.twist.linear.y = current_Twist_.linear.y ;
    //map.twist.twist.linear.z = current_Twist_.linear.z ;

    //map.twist.twist.angular.x= current_Twist_.angular.x ;
    //map.twist.twist.angular.y= current_Twist_.angular.y ;
    //map.twist.twist.angular.z= current_Twist_.angular.z ;
    
    map.twist.twist.linear.x = 0 ;
    map.twist.twist.linear.y = 0 ;
    map.twist.twist.linear.z = 0 ;

    map.twist.twist.angular.x= 0 ;
    map.twist.twist.angular.y= 0 ;
    map.twist.twist.angular.z= 0 ;

    //publish the message
    pub_.publish(map);

    //if(enabled_localization)
    //   return;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(Map2BaseLink);

  }
  GazebomapPublisher::~GazebomapPublisher(){
  }
}
