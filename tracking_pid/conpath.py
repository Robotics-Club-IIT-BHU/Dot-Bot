#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
def trajer(data):
  global f
  global cg
  cpx=data.pose.position.x
  cpy=data.pose.position.y
  dis=((cg[0]-cpx)**2+(cg[1]-cpy)**2)**0.5
  print(dis)
  if dis<0.1:
   f=1
def pather():
  global px,py,f,path_pub
  global cg
  path=[(-0.009,0.896),(0.000,0.303),(-0.295,0.305),(-0.302,0.600),(-0.307,0.9),(-0.009,0.896)]
  for i in range(len(path)-1):
    f=0
    cg=path[i+1]
    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=rospy.Time.now()
    pose.pose.position.x = path[i][0]
    pose.pose.position.y = path[i][1]
    pose.pose.position.z=0
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0
    pose.pose.orientation.w=1
    msg.poses.append(pose)
    pose = PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=rospy.Time.now()
    pose.pose.position.x = path[i+1][0]
    pose.pose.position.y = path[i+1][1]
    pose.pose.position.z=0
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0
    pose.pose.orientation.w=1
    msg.poses.append(pose)
    path_pub.publish(msg)
    # rospy.sleep(5)
    
    while f==0 and not rospy.is_shutdown():
         print(f)
    rospy.sleep(2)
def reached(data):
    global px,py
    px=data.pose.position.x
    py=data.pose.position.y
    # print(x,y)
def main():
    pather()
if __name__ == "__main__":
    global cg
    cg=[100,100]
    rospy.init_node('pathing', anonymous=True)
    # res_call=rospy.Subscriber('/dot2/pose',PoseStamped,reached,queue_size=1)
    traj_call=rospy.Subscriber('dot2/pose',PoseStamped,trajer,queue_size=10)
    path_pub=rospy.Publisher('dot2/path',Path,queue_size=10,latch=True)
    px,py=0,0
    f=0
    main()
    rospy.spin()
