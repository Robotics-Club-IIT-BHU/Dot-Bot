#!/usr/bin/env python3

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3, WrenchStamped
import pandas as pd
from time import time
import os
from datetime import datetime

world_link_frame="tag_12"

robot_link_frame="tag_11"

prev_location = None
curr_location = None
prev_timestamp = None
curr_timestamp = None
vel = [0,0,0]
prev_vel = [0,0,0]

def updateVel(msg):
    global world_link_frame, robot_link_frame, prev_location, curr_location, curr_timestamp, prev_timestamp, vel, prev_vel
    detections = msg.detections
   
    for detection in detections:
        if(detection.id[0]!=int(robot_link_frame.split("_")[-1])):
            continue
        
        pose = detection.pose
        pose_stamped = PoseStamped()
        pose_stamped.header = detection.pose.header
        pose_stamped.header.frame_id += "_link"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = pose.pose.pose
        # print(pose)
        try:
            transformed_pose_stameped = listener.transformPose(world_link_frame, pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue
        #print(pose_stamped, transformed_pose_stameped)
        duration = rospy.Duration(1)
        if curr_timestamp is None:
            curr_timestamp = rospy.Time.now()
            prev_timestamp = rospy.Time.now()
        else:
            prev_timestamp = curr_timestamp
            curr_timestamp = rospy.Time.now()
            duration = curr_timestamp - prev_timestamp
        #print(prev_location, curr_location)
        if curr_location is not None:
            prev_location[0],prev_location[1],prev_location[2] = curr_location[0],curr_location[1],curr_location[2]
        else:
            curr_location = [0,0,0]
        curr_location[0] = transformed_pose_stameped.pose.position.x
        curr_location[1] = transformed_pose_stameped.pose.position.y
        curr_location[2] = transformed_pose_stameped.pose.position.z
        #print(prev_location, curr_location)
        if prev_location is None:
            prev_location = [curr_location[0],curr_location[1],curr_location[2]]
        
        prev_vel = vel
        vel[0] = (curr_location[0] - prev_location[0])/duration.to_sec()
        vel[1] = (curr_location[1] - prev_location[1])/duration.to_sec()
        vel[2] = (curr_location[2] - prev_location[2])/duration.to_sec()
        #print(vel,prev_vel)
def loop():
    global prev_location, curr_location, robot_link_frame, base_link_frame, vel, prev_vel
    pub = rospy.Publisher("robot_vel", WrenchStamped, queue_size=10)
    r = rospy.Rate(30)
    vel_X, vel_Y, vel_Z, time_stamp = [], [], [], []
    while not rospy.is_shutdown():
        p = WrenchStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "usb_cam0_link"
        p.wrench.force.x = 0.95*vel[0] + 0.05*prev_vel[0]
        p.wrench.force.y = 0.95*vel[1] + 0.05*prev_vel[1]
        p.wrench.force.z = 0.95*vel[2] + 0.05*prev_vel[2]
        vel_X.append(vel[0])
        vel_Y.append(vel[1])
        vel_Z.append(vel[2])
        time_stamp.append(time())
        pub.publish(p)
        r.sleep()
    df = pd.DataFrame({"vel_x": vel_X, "vel_y": vel_Y, "vel_z": vel_Z, "time_stamp": time_stamp})
    now = datetime.now().strftime("%y-%m-%d_%H-%M-%S")
    df.to_csv("/home/hexplex0xff/stoch_log/velocities-"+now+".csv", index=False)
    rospy.spin()
if __name__=="__main__":
    try:
        rospy.init_node("tf_velestimate")
        tfBuffer = tf2_ros.Buffer()
        listener = tf.TransformListener(tfBuffer)

        rospy.Subscriber("tag_detections", AprilTagDetectionArray, updateVel)
        loop()
    except:
        print("done for the day")
