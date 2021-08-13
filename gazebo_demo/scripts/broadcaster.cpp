#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0.707, 0, 0.707), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"world", "camera1"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0.707, 0, 0.707), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"world", "camera2"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0.707, 0, 0.707), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"world", "camera3"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0.707, 0, 0.707), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"world", "camera4"));    
        
    r.sleep();
  }
}
