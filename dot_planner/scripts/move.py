#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import tf2_ros
import numpy as np
import math
import time

PI = math.pi
TOLERANCE = 0.1

class Vector:
    def __init__ (self, x, y):
        self.x = x
        self.y = y
    def dot(self, o):
        return o.x * self.x + o.y * self.y
    def normalize(self):
        norm = math.sqrt(self.x ** 2 + self.y ** 2)
        self.x /= norm
        self.y /= norm

class Dot:
    def __init__(self, name='dot'):
        self.cur_x = None
        self.cur_y = None
        self.theta = None
        self.vel_pub = rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=1000)
        
        rospy.Subscriber(f"/{name}/odom", Odometry, self.odom_callback)
        rospy.Subscriber(f"/{name}/imu", Imu, self.imu_callback)       

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.cur_x = pose.position.x
        self.cur_y = pose.position.y
    
    def imu_callback(self, msg):
        quaternion = [msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def move_to(self, goal):
        
        while not self.theta:
            time.sleep(0.1)

        rate = rospy.Rate(1000)
        while self.dist(goal) > TOLERANCE:

            v1 = Vector(math.cos(self.theta), math.sin(self.theta))
            v2 = Vector(goal[0]-self.cur_x, goal[1]-self.cur_y)
            v1.normalize()
            v2.normalize()

            error = math.acos(v1.dot(v2))

            if np.cross([v1.x, v1.y, 0], [v2.x, v2.y, 0])[-1] < 0:
                error *= -1

            twt = Twist()
            twt.linear.x = 0.002 / abs(error) # Linear velocity inversely propotinal to error
            twt.linear.y = 0
            twt.linear.z = 0

            twt.angular.x = 0
            twt.angular.y = 0
            twt.angular.z = error * 0.5 # angular velocity directly propotinal to error

            self.vel_pub.publish(twt)
            print(error, self.dist(goal))

            rate.sleep()

        twt = Twist()
        twt.linear.x = 0
        twt.linear.y = 0
        twt.linear.z = 0

        twt.angular.x = 0
        twt.angular.y = 0
        twt.angular.z = 0

        self.vel_pub.publish(twt)
    
    def dist(self, goal):
        return math.sqrt((goal[0] - self.cur_x) ** 2 + (goal[1] - self.cur_y) ** 2)
        

GOALS = [[0.25, -0.5], [-0.9, -0.5]]

if __name__ == '__main__':
    try:
        rospy.init_node('move_node')

        dot = Dot()
        for goal in GOALS:
            dot.move_to(goal)
            print('REACHED')
            time.sleep(10)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")