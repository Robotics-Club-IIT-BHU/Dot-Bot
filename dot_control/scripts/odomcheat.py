#!/usr/bin/env python3
import rospy
import math
import tf
import numpy
import roslib
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

x,y,z=0.25,1,0
rx,ry,rz,rw=0,0,0,0
def p1_callback(data):
    global x,y,z,rx,ry,rz,rw
    # print(data.name)
    i=data.name.index("dot::origin_link")
    x=data.pose[i].position.x
    y=data.pose[i].position.y
    z=data.pose[i].position.z
    rx=data.pose[i].orientation.x
    ry=data.pose[i].orientation.y
    rz=data.pose[i].orientation.z
    rw=data.pose[i].orientation.w
#    print(x,y)
    
def update_odom():
  global x,y,z,rx,ry,rz,rw
  (trans1,rot1)=tfl.lookupTransform('origin_link',"odom",rospy.Time(0))
  br.sendTransform([x,y,z],[rx,ry,rz,rw],rospy.Time.now(),"dot1origin_link","map")
  br.sendTransform(trans1,rot1,rospy.Time.now(),"dot1odom","dot1origin_link")
  tfl.waitForTransform("map","dot1odom",rospy.Time(0),rospy.Duration(4.0))
  (trans2,rot2)=tfl.lookupTransform('map','dot1odom',rospy.Time(0))
  br.sendTransform(trans2,rot2,rospy.Time.now(),"odom","map")
  pose = PoseStamped()
  pose.header.frame_id='map'
  pose.header.stamp=rospy.Time.now()
  pose.pose.position.x = x
  pose.pose.position.y = y
  pose.pose.position.z=0
  pose.pose.orientation.x=0
  pose.pose.orientation.y=0
  pose.pose.orientation.z=0
  pose.pose.orientation.w=1
  pose_pub.publish(pose)

def main():
   m1=rospy.Subscriber('/gazebo/link_states',LinkStates,p1_callback,queue_size=1)
#    odom_pub=rospy.Publisher('/dot1/pose',Odometry,queue_size=20)
#    odom=Odometry()
   rate=rospy.Rate(10)
   tfl.waitForTransform('origin_link','odom',rospy.Time(),rospy.Duration(4.0))
   while not rospy.is_shutdown():
       update_odom()
       rate.sleep()
if __name__ == "__main__":
    rospy.init_node('marksub', anonymous=True)
    pose_pub=rospy.Publisher('/dot2/pose',PoseStamped,queue_size=10)
    br = tf.TransformBroadcaster()
    tfl=tf.TransformListener()
    main()
    rospy.spin()
