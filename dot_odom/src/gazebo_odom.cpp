#include "dot_odom/gazebo_odom.hpp"
#include <cmath>
#include <algorithm>

namespace gazebo_odom {
  GazeboOdomPublisher::GazeboOdomPublisher(ros::NodeHandle &nodeHandle):nodeHandle_(nodeHandle){
    pub_ = nodeHandle_.advertise<nav_msgs::Odometry>("odom", 50) ;
    modelName_= (std::string)"dot" ;
    relativeEntityName_ = (std::string)"nosignal_ps1.world";
    client_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
  }
  void GazeboOdomPublisher::publishOdom(){
    //get model state from gazebo
    getModelState_.request.model_name = modelName_ ;
    getModelState_.request.relative_entity_name = relativeEntityName_ ;
    client_.call(getModelState_) ;

    //saving model variables
    pp_ = getModelState_.response.pose.position ;
    qq_ = getModelState_.response.pose.orientation ;
    tf::Quaternion tf_quat(qq_.x, qq_.y, qq_.z, qq_.w) ;
    current_Twist_ = getModelState_.response.twist ;

    //tranform odom
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.transform.translation.x = pp_.x ;
    odom_trans.transform.translation.y = pp_.y ;
    odom_trans.transform.translation.z = pp_.z ;

    geometry_msgs::Quaternion geo_Quat ;
    tf::quaternionTFToMsg(tf_quat, geo_Quat) ;
    odom_trans.transform.rotation = geo_Quat ;

    odom_trans.header.stamp = ros::Time::now() ;
    odom_trans.header.frame_id = "odom" ;
    odom_trans.child_frame_id = "base_link" ;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

    //publish odometry over ros
    nav_msgs::Odometry odom ;
    odom.header.stamp = odom_trans.header.stamp ;
    odom.header.frame_id = "odom" ;

    //set the position
    odom.pose.pose.position.x = odom_trans.transform.translation.x ;
    odom.pose.pose.position.y = odom_trans.transform.translation.y ;
    odom.pose.pose.position.z = odom_trans.transform.translation.z ;
    odom.pose.pose.orientation = geo_Quat ;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = current_Twist_.linear.x ;
    odom.twist.twist.linear.y = current_Twist_.linear.y ;
    odom.twist.twist.linear.z = current_Twist_.linear.z ;

    odom.twist.twist.angular.x= current_Twist_.angular.x ;
    odom.twist.twist.angular.y= current_Twist_.angular.y ;
    odom.twist.twist.angular.z= current_Twist_.angular.z ;

    //publish the message
    pub_.publish(odom);

  }
  GazeboOdomPublisher::~GazeboOdomPublisher(){
  }
}
