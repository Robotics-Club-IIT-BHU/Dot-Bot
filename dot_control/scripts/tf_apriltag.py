#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf

global marker_arry, visited_ids
marker_arry = MarkerArray()
visited_ids = {}

N_TAGS = 8

def tags_callback(msg):
    detections = msg.detections  

    print("Found ", len(detections), " tags!!")

    for detection in detections:
        if detection.id in visited_ids:
            continue
        
        visited_ids[detection.id] = True
        pose = detection.pose

        pose_stamped = PoseStamped()
        pose_stamped.header = detection.pose.header
        pose_stamped.pose = pose.pose.pose

        try:
            tranformed_pose_stamped = listener.transformPose('/world', pose_stamped)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue

        marker_msg = Marker()
        marker_msg.header = tranformed_pose_stamped.header
        marker_msg.id = detection.id
        marker_msg.pose = tranformed_pose_stamped.pose

        marker_arry.markers.append(marker_msg)

        print("Found tag ", detection.id, ' at (', 
        tranformed_pose_stamped.pose.position.x, ', ', 
        tranformed_pose_stamped.pose.position.y)
    
    if len(visited_ids) == N_TAGS:
        marker_arry_pub.publish(marker_arry)

if __name__ == '__main__':
    try:
        rospy.init_node('tf_apriltag')
        listener = tf.TransformListener()

        marker_arry_pub = rospy.Publisher('/tags_marker_array', MarkerArray, queue_size=10)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tags_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")