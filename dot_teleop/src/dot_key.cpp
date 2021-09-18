#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define KEYI 105
#define KEYK 107
#define KEYJ 106
#define KEYL 108
#define KEYO 111
#define KEYM 109
#define KEYU 117
#define KEY_COM 44
#define KEY_DOT 46

class DotTeleop
{
public:
  DotTeleop();
  void keyLoop();
  void watchdog();

private:

  
  ros::NodeHandle nh_,ph_;
  double linear_x_, linear_y_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  void publish(double, double, double);
  boost::mutex publish_mutex_;

};

DotTeleop::DotTeleop():
  ph_("~"),
  linear_x_(0),
  linear_y_(0),
  angular_(0),
  l_scale_(1),
  a_scale_(1)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/dot/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dot_teleop");
  DotTeleop Dot_teleop;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&DotTeleop::keyLoop, &Dot_teleop));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&DotTeleop::watchdog, &Dot_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void DotTeleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0, 0);
}

void DotTeleop::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Moving around:");
  puts("u    i    o");
  puts("j    k    l");
  puts("CTRL-C to quit");

  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_x_=linear_y_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYJ:
        ROS_DEBUG("LEFT");
        linear_y_ = 1;
        break;
      case KEYL:
        ROS_DEBUG("RIGHT");
        linear_y_ = -1;
        break;
      case KEYI:
        ROS_DEBUG("UP");
        linear_x_ = 1;
        break;
      case KEYK:
        ROS_DEBUG("DOWN");
        linear_x_ = -1;
        break;
      case KEYO:
        ROS_DEBUG("CLOCK");
        angular_ = -1;
      case KEYU:
        ROS_DEBUG("ANTICLOCK");
        angular_=1; 
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_x_, linear_y_);
  }

  return;
}

void DotTeleop::publish(double angular, double linear_x, double linear_y)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear_x;
    vel.linear.y = l_scale_*linear_y;
    vel_pub_.publish(vel);    


  return;
}