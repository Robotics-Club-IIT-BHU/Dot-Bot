#include "dot_map/gazebo_map.hpp"
#include <cmath>
#include <algorithm>

namespace gazebo_map {
  GazebomapPublisher::GazebomapPublisher(ros::NodeHandle &nodeHandle):nodeHandle_(nodeHandle){
    pub_ = nodeHandle_.advertise<nav_msgs::Odometry>("map", 50) ;
    modelName_= (std::string)"dot" ;
    relativeEntityName_ = (std::string)"nosignal_ps2.world";
    client_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
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


    //transform using tf2
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped map_trans;

    map_trans.header.stamp = ros::Time::now();
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "odom";
    map_trans.transform.translation.x =pp_.x;
    map_trans.transform.translation.y =pp_.y;
    map_trans.transform.translation.z =pp_.z;

    geometry_msgs::Quaternion geo_Quat ;
    geo_Quat=tf2::toMsg(tf_quat) ;
    map_trans.transform.rotation = geo_Quat ;

    br.sendTransform(map_trans);


    //publish mapetry over ros
    nav_msgs::Odometry map ;
    map.header.stamp = map_trans.header.stamp ;
    map.header.frame_id = "map" ;

    //set the position
    map.pose.pose.position.x = map_trans.transform.translation.x ;
    map.pose.pose.position.y = map_trans.transform.translation.y ;
    map.pose.pose.position.z = map_trans.transform.translation.z ;
    map.pose.pose.orientation = geo_Quat ;

    //set the velocity
    map.child_frame_id = "odom";
    map.twist.twist.linear.x = current_Twist_.linear.x ;
    map.twist.twist.linear.y = current_Twist_.linear.y ;
    map.twist.twist.linear.z = current_Twist_.linear.z ;

    map.twist.twist.angular.x= current_Twist_.angular.x ;
    map.twist.twist.angular.y= current_Twist_.angular.y ;
    map.twist.twist.angular.z= current_Twist_.angular.z ;

    //publish the message
    pub_.publish(map);

  }
  GazebomapPublisher::~GazebomapPublisher(){
  }
}
