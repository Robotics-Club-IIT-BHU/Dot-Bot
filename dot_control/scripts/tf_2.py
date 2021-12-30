#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
import tf
import math
import tf2_ros
import numpy
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class posmsg: 
    def __init__(self, trans, rot): 
        self.trans = trans
        self.rot = rot

pseudo_frame = {
  'dot4/base_link': 'tag_1',
  'dot2/base_link': 'tag_0',
  'dot1/base_link': 'tag_2',
  'dot3/base_link': 'tag_3'
}

stat=[]
for i in range(13,54):
   stat.append(i)
botgs=[(4,151),(0,150),(3,314),(1,243)]
botpos=[]

for i in range(6):
  botpos.append(posmsg([0,0,0],[0,0,0,0]))

def update_odom(i):
  #(trans1,rot1)=tfl.lookupTransform("dot"+str(i+1)+"/base_link","dot"+str(i+1)+"/odom",rospy.Time(0))
  br.sendTransform(botpos[i].trans,botpos[i].rot,rospy.Time.now(),"dot"+str(i+1)+"/center_link","map")
  # br.sendTransform(botpos[i].trans,botpos[i].rot,rospy.Time.now(),"dot"+str(i+1)+"origin_link","map")
  # br.sendTransform(trans1,rot1,rospy.Time.now(),"dot"+str(i+1)+"odom","dot"+str(i+1)+"origin_link")
  # tfl.waitForTransform("map","dot"+str(i+1)+"odom",rospy.Time(0),rospy.Duration(4.0))
  # (trans2,rot2)=tfl.lookupTransform('map','dot'+str(i+1)+'odom',rospy.Time(0))
  # br.sendTransform(trans2,rot2,rospy.Time.now(),"dot"+str(i+1)+"/odom","map")

def myf(x):
    return x[1]
def tags_callback(msg):
    detections = msg.detections  
    global dista,distti,camtag
    dot_pub=[dot1_pub,dot2_pub,dot3_pub,dot4_pub]
#    print(detections)
#    print("hi")
    l1=[]
    for detection in detections:
     tagid=detection.id[0]
     if detection.id[0] in stat:
        #  print("heelo")
        #  print(detection.id[0],detection.pose.header.frame_id)
         (trant,rott)=tfl.lookupTransform(detection.pose.header.frame_id+'_link/tag_'+str(tagid),detection.pose.header.frame_id+'_link/parent',rospy.Time(0))
         dis2=math.sqrt(pow(trant[0],2)+pow(trant[1],2))
         if not detection.pose.header.frame_id in camtag:
          l1=[]
         l1.append((detection.id[0],dis2))
         camtag[detection.pose.header.frame_id]=l1
    for x in camtag:
        camtag[x].sort(key=myf)
    print("camtag=",camtag)
    for detection in detections:
      for i in range(len(botgs)):
        if detection.id[0] in botgs[i]:
            botid=detection.id[0]
            # print(detection.id[0])
#            print(detection.pose.header.frame_id+'link/tag'+str(detection.id[0]), 
#            detection.pose.header.frame_id+'link/tag'+str(bot_temp))
            (transx,rotx)=tfl.lookupTransform(detection.pose.header.frame_id+'_link/tag_'+str(botid),detection.pose.header.frame_id+'_link/parent',rospy.Time(0))
            dis=math.sqrt(pow(transx[0],2)+pow(transx[1],2))
            dista[i][detection.pose.header.frame_id]=dis
            distti[i][detection.pose.header.frame_id]=rospy.Time.now()
    minf=[0,0,0,0]
    for i in range(len(botgs)):
      mini=100
      for x in dista[i]:
          if dista[i][x]<mini:
              mini=dista[i][x]
              minf[i]=x
      print(minf[i])
    for detection in detections:  
      for i in range(len(botgs)):
        if detection.id[0] in botgs[i]:
            botid=detection.id[0]
#            print(detection.pose.header.frame_id+'link/tag'+str(detection.id[0]), 
#            detection.pose.header.frame_id+'link/tag'+str(bot_temp))
            st_temp=camtag[minf[i]][0][0]
            print(st_temp)
            (trans,rot) = tfl.lookupTransform(minf[i]+'_link/tag_'+str(st_temp),minf[i]+'_link/tag_'+str(detection.id[0]), 
             rospy.Time(0))   
            print('thigirans')
            print(detection.id[0])
            br.sendTransform(trans,rot,rospy.Time.now(),"tag_"+str(detection.id[0]),"tag_"+str(st_temp))
            transt=[0,0,0]
            rott=[0,0,0,0]
            if tfl.canTransform('tag_'+str(botgs[i][0]),'map',rospy.Time(0)) and tfl.canTransform('tag_'+str(botgs[i][1]),'map',rospy.Time(0)):
         
             (trans2,rot2)=tfl.lookupTransform('map','tag_'+str(botgs[i][0]),rospy.Time(0))
             (trans3,rot3)=tfl.lookupTransform('map','tag_'+str(botgs[i][1]),rospy.Time(0))
             transt[0]=(trans2[0]+trans3[0])/2
             rott[0]=(rot2[0]+rot3[0])/2
             transt[1]=(trans2[1]+trans3[1])/2
             rott[1]=(rot2[1]+rot3[1])/2
             transt[2]=(trans2[2]+trans3[2])/2
             rott[2]=(rot2[2]+rot3[2])/2
             rott[3]=(rot2[3]+rot3[3])/2
            #  print("hiii")
            elif tfl.canTransform('tag_'+str(botgs[i][0]),'map',rospy.Time(0)): 
              print(i)
              if i==1:
                to,ro= tfl.lookupTransform('tag_'+str(botgs[i][0]),"map",rospy.Time(0))
                print(to[1])
                to[1]=to[1]+0
                print(to[1])
                print("chutiydddddddddddddddddddddddua4")
                transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(to), tf.transformations.quaternion_matrix(ro))
                inversed_transform1 = tf.transformations.inverse_matrix(transform1)
                transt = tf.transformations.translation_from_matrix(inversed_transform1)
                rott = tf.transformations.quaternion_from_matrix(inversed_transform1) 
              elif i==0:
                to,ro= tfl.lookupTransform('tag_'+str(botgs[i][0]),"map",rospy.Time(0))
                print(to[1])
                to[1]=to[1]
                to[0]=to[0]+0.01
                print(to[1])
                print("chutiydddddddddddddddddddddddua4")
                transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(to), tf.transformations.quaternion_matrix(ro))
                inversed_transform1 = tf.transformations.inverse_matrix(transform1)
                transt = tf.transformations.translation_from_matrix(inversed_transform1)
                rott = tf.transformations.quaternion_from_matrix(inversed_transform1)               
              else:
                to,ro= tfl.lookupTransform('tag_'+str(botgs[i][0]),"map",rospy.Time(0))
                print(to[1])
                to[1]=to[1]+0.025
                print(to[1])
                print("chutiydddddddddddddddddddddddua2")
                transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(to), tf.transformations.quaternion_matrix(ro))
                inversed_transform1 = tf.transformations.inverse_matrix(transform1)
                transt = tf.transformations.translation_from_matrix(inversed_transform1)
                rott = tf.transformations.quaternion_from_matrix(inversed_transform1)

            elif tfl.canTransform('tag_'+str(botgs[i][1]),'map',rospy.Time(0)):
              transt,rott=tfl.lookupTransform('map','tag_'+str(botgs[i][1]),rospy.Time(0))
              g1=1

            if tfl.canTransform('tag_'+str(botgs[i][0]),'map',rospy.Time(0)) or tfl.canTransform('tag_'+str(botgs[i][1]),'map',rospy.Time(0))  :
             pose_st=PoseStamped()
             pose_st.header.frame_id='map'
             pose_st.header.stamp=rospy.Time.now()
             pose_st.pose.position.x=transt[0]
             pose_st.pose.position.y=transt[1]
             botpos[i].trans=transt
             botpos[i].rot=rott
            #  transform1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(transt), tf.transformations.quaternion_matrix(rott))
            #  inversed_transform1 = tf.transformations.inverse_matrix(transform1)
            #  translation1 = tf.transformations.translation_from_matrix(inversed_transform1)
            #  quaternion1 = tf.transformations.quaternion_from_matrix(inversed_transform1)
            #  br.sendTransform(translation1,quaternion1,rospy.Time.now(),"map","origin_link")
            # #  pose_st.pose.position.z=transt[2]
            # #  print(pose_st)

             dot_pub[i].publish(pose_st) 
             update_odom(i)

             
             print("H")    
    #   print(dista)
    for i in range(len(botgs)):  
      for x in dista[i]:
         if(rospy.Time.now()-distti[i][x]>rospy.Duration(1)):
              dista[i][x]=100      
            
  

if __name__ == '__main__':
    try:
        rospy.init_node('tf_apriltag')
        br = tf.TransformBroadcaster()
        tfl=tf.TransformListener()
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tags_callback)
        dot1_pub = rospy.Publisher('/dot1/pose', PoseStamped, queue_size=10)
        dot2_pub = rospy.Publisher('/dot2/pose', PoseStamped, queue_size=10)
        dot3_pub = rospy.Publisher('/dot3/pose', PoseStamped, queue_size=10)
        dot4_pub = rospy.Publisher('/dot4/pose', PoseStamped, queue_size=10)
        # for i in range(4):
        #   update_odom(i+1,[0,0,0])
        # rospy.Subscriber("/dot1/pose", PoseStamped, odom1_shift_callback)
        # rospy.Subscriber("/dot2/pose", PoseStamped, odom2_shift_callback)
        # rospy.Subscriber("/dot3/pose", PoseStamped, odom3_shift_callback)
        # rospy.Subscriber("/dot4/pose", PoseStamped, odom4_shift_callback)
        dista=[{},{},{},{}]
        distti=[{},{},{},{}]
        camtag={}
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")