#!/usr/bin/env python3
from re import X
import rospy
import math
import tf
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry

def callback(msg):
    global x
    global y  
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y


if __name__ == '__main__':
  global  x 
  global y 

  xf=int(input("Enter x coordinate"))
  yf=int(input("Enter y coordinate"))
  a=Twist()
  rospy.init_node('dot_move', anonymous=True)
  pub=rospy.Publisher("dot/cmd_vel",Twist,queue_size=10)
  rospy.Subscriber("/dot/odom", Odometry, callback)
  rospy.sleep(1)
  while(not rospy.is_shutdown() and ((x-xf)**2+(y-yf)**2)**0.5>1):
   a.linear.x=(xf-x)/20
   a.linear.y=(yf-y)/20
   print(a.linear.x,a.linear.y)
   pub.publish(a)

  b=Twist()
  pub.publish(b) 
 
