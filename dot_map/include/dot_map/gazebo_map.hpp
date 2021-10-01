#pragma once

#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
namespace gazebo_map {

/**
 * Class containing the SMB Highlevel Controller
 */
class GazebomapPublisher {
 public:
  /** Constructor */
  GazebomapPublisher(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~GazebomapPublisher();
  void publishmap();
  void getMap2Odom(geometry_msgs::TransformStamped Map2BaseLink);
  void sendTransform(geometry_msgs::TransformStamped Map2BaseLink);
 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher pub_;
  std::string modelName_;
  std::string relativeEntityName_;
  gazebo_msgs::GetModelState getModelState_ ;
  geometry_msgs::Point pp_ ;
  geometry_msgs::Quaternion qq_ ;
  geometry_msgs::Twist current_Twist_ ;
  ros::ServiceClient client_;
  tf::TransformBroadcaster map_broadcaster_;
  bool enabled_localization;

  //tf listener
  //tf listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener * tfListenerObj;

};
}
