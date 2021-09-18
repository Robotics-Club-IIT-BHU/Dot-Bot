#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
import tf
import tf2_ros

global marker_arry, visited_ids
marker_arry = MarkerArray()
visited_ids = {}

N_TAGS = 6

def tags_callback(msg):
    detections = msg.detections  

    print("Found ", len(detections), " tags!!")

    for detection in detections:
        #if detection.id in visited_ids:
        #    continue
        
        visited_ids[detection.id] = True
        pose = detection.pose

        pose_stamped = PoseStamped()
        pose_stamped.header = detection.pose.header
        pose_stamped.pose = pose.pose.pose
 
        #try:
            #f = tfBuffer.lookup_transform(detection.pose.header.frame_id, 'world', rospy.Time())
            #print(f)
            #raise Exception("OK")
        tranformed_pose_stamped = listener.transformPose('world', pose_stamped)
        print('-'*5, tranformed_pose_stamped, pose_stamped, '-'*5)
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #    print(e)
        #    continue

        marker_msg = Marker()
        marker_msg.header = tranformed_pose_stamped.header
        marker_msg.id = detection.id[0]
        marker_msg.pose = tranformed_pose_stamped.pose
        marker_msg.scale = Vector3()
        marker_msg.scale.x = 1 
        marker_msg.scale.y = 1 
        marker_msg.scale.z = 1
        marker_msg.color = ColorRGBA()
        if(detection.pose.header.frame_id[-1]=="2"):
            marker_msg.color.r = 1
        if(detection.pose.header.frame_id[-1]=="3"):
            marker_msg.color.b = 1
        if(detection.pose.header.frame_id[-1]=="4"):
            marker_msg.color.g = 1
        
        marker_msg.color.a = 1
        marker_arry.markers.append(marker_msg)

        print("Found tag ", detection.id, ' at (', 
        tranformed_pose_stamped.pose.position.x, ', ', 
        tranformed_pose_stamped.pose.position.y)
    
    #if len(visited_ids) == N_TAGS:
    print('-'*10, "SENT MESSAGE", '-'*10)
    marker_arry_pub.publish(marker_arry)

if __name__ == '__main__':
    try:
        rospy.init_node('tf_apriltag')
        tfBuffer = tf2_ros.Buffer()
        listener = tf.TransformListener(tfBuffer)

        marker_arry_pub = rospy.Publisher('/tags_marker_array', MarkerArray, queue_size=10)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tags_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")