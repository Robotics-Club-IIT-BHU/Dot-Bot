#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/dot/cmd_vel", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(int dir)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    double dist=0.1524;
    switch(dir){
      case 8:
        base_cmd.linear.y = 0;
        base_cmd.linear.x = 0.25;
        break;
      case 2:
        base_cmd.linear.y = 0;
        base_cmd.linear.x = -0.25;
        break;
      case 4:
        base_cmd.linear.x = 0;
        base_cmd.linear.y = 0.25;
        break;
      case 6:
        base_cmd.linear.x = 0;
        base_cmd.linear.y = -0.25;
        break;
      default:
        base_cmd.linear.y = 0;
        base_cmd.linear.x = 0;
        dist=-1;
        break;
    }
    base_cmd.angular.z = 0;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();
      
      if(dist_moved > dist) 
      {
      done = true;
      base_cmd.linear.x=0;
      base_cmd.linear.y=0;
      cmd_vel_pub_.publish(base_cmd);
      }
    }
    if (done) return true;
    return false;
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  int d;
  std::cin>>d;
  driver.driveForwardOdom(d);
}

