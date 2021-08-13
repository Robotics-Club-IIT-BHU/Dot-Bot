#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
 
  geometry_msgs::PointStamped base_link_p;
  base_link_p.header.frame_id = "base_link";

  base_link_p.header.stamp = ros::Time();

  base_link_p.point.x = 1.0;
  base_link_p.point.y = 1.0;
  base_link_p.point.z = 0.0;

  try{
    geometry_msgs::PointStamped camera1_p;
    listener.transformPoint("camera1", base_link_p, camera1_p); 
    geometry_msgs::PointStamped camera2_p;
    listener.transformPoint("camera2", base_link_p, camera2_p);    
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"wheel_frame\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}