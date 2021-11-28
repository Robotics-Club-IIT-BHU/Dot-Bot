#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Vector3,TransformStamped
from std_msgs.msg import ColorRGBA
import tf
import tf2_ros

global marker_arry_1,marker_arry_2,marker_arry_3, marker_arry_4, visited_ids
marker_arry_1 = MarkerArray()
marker_arry_2 = MarkerArray()
marker_arry_3 = MarkerArray()
marker_arry_4 = MarkerArray()
visited_ids = {}
i_6 = 0.1524
i_3 = 0.0762


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
        
    try:
            trans = tfBuffer.lookup_transform('map', 'tag_14', rospy.Time())
            print(trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Hello")
 
        #try:
            #f = tfBuffer.lookup_transform(detection.pose.header.frame_id, 'world', rospy.Time())
            #print(f)
            #raise Exception("OK")
    #     tranformed_pose_stamped = listener.transformPose('world', pose_stamped)
    #     print('-'*5, tranformed_pose_stamped, pose_stamped, '-'*5)
    #     #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #     #    print(e)
    #     #    continue

    #     marker_msg = Marker()
    #     marker_msg.header = tranformed_pose_stamped.header
    #     marker_msg.id = detection.id[0]
    #     marker_msg.pose = tranformed_pose_stamped.pose
    #     marker_msg.scale = Vector3()
    #     marker_msg.scale.x = 1 
    #     marker_msg.scale.y = 1 
    #     marker_msg.scale.z = 1
    #     marker_msg.color = ColorRGBA()
    #     if(detection.pose.header.frame_id[-1]=="2"):
    #         marker_msg.color.r = 1
    #     if(detection.pose.header.frame_id[-1]=="3"):
    #         marker_msg.color.b = 1
    #     if(detection.pose.header.frame_id[-1]=="4"):
    #         marker_msg.color.g = 1
        
    #     marker_msg.color.a = 1
    #     marker_arry.markers.append(marker_msg)

    #     print("Found tag ", detection.id, ' at (', 
    #     tranformed_pose_stamped.pose.position.x, ', ', 
    #     tranformed_pose_stamped.pose.position.y)
    
    # #if len(visited_ids) == N_TAGS:
    # print('-'*10, "SENT MESSAGE", '-'*10)
    # marker_arry_pub.publish(marker_arry)
        # tranformed_pose_stamped = listener.transformPose('world', pose_stamped)
#        print('-'*5,  pose_stamped, '-'*5)
#        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#        #    print(e)
#        #    continue
#        if detection.id[0] == 14:
#            if pose_stamped.header.frame_id == "usb_cam":
#                br = tf.TransformBroadcaster()
#                br.sendTransform((pose_stamped.pose.position.x,pose_stamped.pose.position.y, 0),
#                        tf.transformations.quaternion_from_euler(0, 0, 0),
#                        rospy.Time.now(),
#                        pose_stamped.header.frame_id,
#                        "world")
#            # if pose_stamped.header.frame_id == "usb_cam_2":
#            #     br = tf.TransformBroadcaster()
#            #     br.sendTransform((pose_stamped.pose.position.x,pose_stamped.pose.position.y, 0),
#            #             tf.transformations.quaternion_from_euler(0, 0, 0),
#            #             rospy.Time.now(),
#            #             pose_stamped.header.frame_id,
#            #             "world")               
#            for k in range(16):
#                marker_msg = Marker()
#                marker_msg.header.frame_id = "/world"
#                marker_msg.id = k
#                marker_msg.pose.position.x = path1[k][0]
#                marker_msg.pose.position.y = path1[k][1]
#                marker_msg.pose.position.z = 0
#                marker_msg.scale.x = 0.1 
#                marker_msg.scale.y = 0.1 
#                marker_msg.scale.z = 0.1
#                marker_msg.color = ColorRGBA()
#                if(detection.pose.header.frame_id[-1]=="2"):
#                    marker_msg.color.r = 1
#                if(detection.pose.header.frame_id[-1]=="3"):
#                    marker_msg.color.b = 1
#                if(detection.pose.header.frame_id[-1]=="4"):
#                    marker_msg.color.g = 1
#                
#                marker_msg.color.a = 1
#                marker_arry_1.markers.append(marker_msg)
#            for k in range(18):
#                marker_msg = Marker()
#                marker_msg.header.frame_id = "/world"
#                marker_msg.id = k
#                marker_msg.pose.position.x = path2[k][0]
#                marker_msg.pose.position.y = path2[k][1]
#                marker_msg.pose.position.z = 0
#                marker_msg.scale.x = 0.1 
#                marker_msg.scale.y = 0.1 
#                marker_msg.scale.z = 0.1
#                marker_msg.color = ColorRGBA()
#                if(detection.pose.header.frame_id[-1]=="2"):
#                    marker_msg.color.r = 1
#                if(detection.pose.header.frame_id[-1]=="3"):
#                    marker_msg.color.b = 1
#                if(detection.pose.header.frame_id[-1]=="4"):
#                    marker_msg.color.g = 1
#                
#                marker_msg.color.a = 1
#                marker_arry_2.markers.append(marker_msg)
#            for k in range(18):
#                marker_msg = Marker()
#                marker_msg.header.frame_id = "/world"
#                marker_msg.id = k
#                marker_msg.pose.position.x = path3[k][0]
#                marker_msg.pose.position.y = path3[k][1]
#                marker_msg.pose.position.z = 0
#                marker_msg.scale.x = 0.1 
#                marker_msg.scale.y = 0.1 
#                marker_msg.scale.z = 0.1
#                marker_msg.color = ColorRGBA()
#                if(detection.pose.header.frame_id[-1]=="2"):
#                    marker_msg.color.r = 1
#                if(detection.pose.header.frame_id[-1]=="3"):
#                    marker_msg.color.b = 1
#                if(detection.pose.header.frame_id[-1]=="4"):
#                    marker_msg.color.g = 1
#                
#                marker_msg.color.a = 1
#                marker_arry_3.markers.append(marker_msg)

#        # print("Found tag ", detection.id, ' at (', 
#        # tranformed_pose_stamped.pose.position.x, ', ', 
#        # tranformed_pose_stamped.pose.position.y)
#    
#    #if len(visited_ids) == N_TAGS:
#    # print('-'*10, "SENT MESSAGE", '-'*10)
#    marker_arry_1_pub.publish(marker_arry_1)
#    marker_arry_2_pub.publish(marker_arry_2)
#    marker_arry_3_pub.publish(marker_arry_3)
#    marker_arry_4_pub.publish(marker_arry_4)




if __name__ == '__main__':
    try:
        rospy.init_node('tf_apriltag')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        marker_arry_1_pub = rospy.Publisher('/tags_marker_array_1', MarkerArray, queue_size=10)
        marker_arry_2_pub = rospy.Publisher('/tags_marker_array_2', MarkerArray, queue_size=10)
        marker_arry_3_pub = rospy.Publisher('/tags_marker_array_3', MarkerArray, queue_size=10)
        marker_arry_4_pub = rospy.Publisher('/tags_marker_array_4', MarkerArray, queue_size=10)

        rospy.Subscriber("/camera1/tag_detections", AprilTagDetectionArray, tags_callback)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "camera1"

        static_transformStamped.transform.translation.x = -0.25
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 1

        quat = tf.transformations.quaternion_from_euler(0,3.142,1.571)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
            
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")
