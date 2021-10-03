#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, Image

pub = None

def image_callback(data):
    global pub
    modded = CameraInfo()
    modded.header = data.header
    modded.width = 1920
    modded.height = 1080
    modded.distortion_model = "plumb_bob"
    modded.K = [1128.1868143323688, 0.0, 1026.9758240822644, 0.0, 1333.034820965338, 474.04433381194474, 0.0, 0.0, 1.0]
    modded.D = [-0.035381758548511826, -0.03526853745015409, 0.008314723970596172, -0.00401659478650466, 0.0]
    modded.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    modded.P = [1071.0687255859375, 0.0, 1026.4520766977148, 0.0, 0.0, 1326.9517822265625, 478.5601418099068, 0.0, 0.0, 0.0, 1.0, 0.0]
    pub.publish(modded)

def listener():
    global pub
    rospy.init_node("camera_info_mod", anonymous=True)


    pub = rospy.Publisher("modified/camera_info",CameraInfo,queue_size=10)

    rospy.Subscriber("image_rect", Image, image_callback)

    rospy.spin()


if __name__ == "__main__":
    listener()
