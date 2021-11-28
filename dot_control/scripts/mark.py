#!/usr/bin/env python3
import rospy
import math
import tf
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

mark1_pub=rospy.Publisher("/dot1/marker",Marker,queue_size=1)
mark2_pub=rospy.Publisher("/dot2/marker",Marker,queue_size=1)
mark3_pub=rospy.Publisher("/dot3/marker",Marker,queue_size=1)
mark4_pub=rospy.Publisher("/dot4/marker",Marker,queue_size=1)
mark5_pub=rospy.Publisher("/dot5/marker",Marker,queue_size=1)
mark6_pub=rospy.Publisher("/dot6/marker",Marker,queue_size=1)
recalc_pub=rospy.Publisher("/recalc",Int8,queue_size=1)
recalc=Int8()
recalc.data=0
def p1_callback(msg):
  global mark1_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark1_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
    
def p2_callback(msg):
  global mark2_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark2_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
  
def p3_callback(msg):
  global mark3_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark3_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
  
def p4_callback(msg):
  global mark4_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark4_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
  
def p5_callback(msg):
  global mark5_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark5_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
  
def p6_callback(msg):
  global mark6_pub,recalc_pub,recalc
  f=0
  for i in msg.poses:
    if recalc.data==1:
      f=1
      break
    x1=i.pose.position.x
    y1=i.pose.position.y
    print(x1,y1)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action=0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.position.x=x1
    marker.pose.position.y=y1
    marker.pose.position.z=0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    mark6_pub.publish(marker)
    rospy.sleep(1)
  if f==0:
    recalc.data=1
    recalc_pub.publish(recalc)
    recalc.data=0
  
def main():
   m1=rospy.Subscriber('/dot1/path',Path,p1_callback,queue_size=1)
   m2=rospy.Subscriber('/dot2/path',Path,p2_callback,queue_size=1)
   m3=rospy.Subscriber('/dot3/path',Path,p3_callback,queue_size=1)
   m4=rospy.Subscriber('/dot4/path',Path,p4_callback,queue_size=1)
   m5=rospy.Subscriber('/dot5/path',Path,p5_callback,queue_size=1)
   m6=rospy.Subscriber('/dot6/path',Path,p6_callback,queue_size=1)
 
if __name__ == "__main__":
    rospy.init_node('marksub', anonymous=True)
    main()
    rospy.spin()
