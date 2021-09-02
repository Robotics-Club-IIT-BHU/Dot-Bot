#pragma once

#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

namespace gazebo_odom {

/**
 * Class containing the SMB Highlevel Controller
 */
class GazeboOdomPublisher {
 public:
  /** Constructor */
  GazeboOdomPublisher(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~GazeboOdomPublisher();
  void publishOdom();
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
  tf::TransformBroadcaster odom_broadcaster_;
};
}
