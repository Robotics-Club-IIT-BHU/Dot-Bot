#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point,Twist
from std_msgs.msg import Int16,String,Bool

city={
    "m": "3",
    "d": "6",
    "k": "9",
    "c": "2",
    "b": "5",
    "h": "8",
    "p": "1",
    "a": "4",
    "j": "7"
}
# city={
#     "m": "1",
#     "d": "2",
#     "k": "3",
#     "c": "4",
#     "b": "5",
#     "h": "6",
#     "p": "7",
#     "a": "8",
#     "j": "9"
# }

def rotCheck(bot,rot):
    zangle=abs(tf.transformations.euler_from_quaternion(rot)[2])
    zangle-=1.57*int(zangle/1.57)
    pub=rospy.Publisher("/dot"+bot+"/ang_vel",Twist,queue_size=0,latch=True)
    msg=Twist()
    print(zangle)
    if(abs(zangle-0.78)<0.5):
        msg.angular.z=50.0
        pub.publish(msg)
        rospy.sleep(1.5)
        msg.angular.z=0.0
        pub.publish(msg)
        rospy.sleep(1.5)
    
pseudo_name = {
    "dot1/center_link": "tag_4",
    "dot2/center_link": "tag_0",
    "dot3/center_link": "tag_5",
    "dot4/center_link": "tag_1"
}
def sendmsg(bot,val):
    # if(bot=="1"):
    #     done_pub1.publish(val)
    # elif(bot=="2"):
    #     done_pub2.publish(val)
    if(bot=="3"):
        done_pub3.publish(val)
    elif(bot=="4"):
        done_pub4.publish(val)
    # elif(bot=="5"):
    #     done_pub5.publish(val)
    # elif(bot=="6"):
    #     done_pub6.publish(val)

def callback(msg):
    # Updating variable
    bot=msg.data[1]
    index=city[msg.data[0]]

    print(">>> Bot "+bot+" is called")
    # Publishing droping of bot is false
    sendmsg(bot,0)

    (trans,rot)=tfl.lookupTransform(pseudo_name["dot"+bot+"/center_link"],"drop"+index,rospy.Time(0))
    rotCheck(bot,rot)
    (trans,rot)=tfl.lookupTransform(pseudo_name["dot"+bot+"/center_link"],"drop"+index,rospy.Time(0))
    point=Point()
    point.x=trans[0]
    point.y=trans[1]
    point.z=0
    if abs(point.x) >= abs(point.y):
        point.y=(1.519/abs(point.x))*point.y
        point.x=1.519*((-1,1)[point.x>0])
    else:
        point.x=(1.519/abs(point.y))*point.x
        point.y=1.519*((-1,1)[point.y>0])
    print("Value=\n",point)
    pub=rospy.Publisher("/dot"+bot+"/servo_cmd",Point,queue_size=0,latch=True)
    pub.publish(point)
    rospy.sleep(1.5)
    pub.publish(point)

    # Publishing droping of bot is done
    rospy.sleep(1.5)
    sendmsg(bot,1)
    traj_pub=rospy.Publisher("/dot"+bot+"/trajectory_finished", Bool, queue_size=0,latch=True)
    traj_pub.publish(True)


if __name__ == '__main__':
    rospy.init_node("dropbox",anonymous=True)
    tfl=tf.TransformListener()
    # rospy.Subscriber("/dot1/drop", String, callback)
    # rospy.Subscriber("/dot2/drop", String, callback)
    rospy.Subscriber("/dot3/drop", String, callback)
    rospy.Subscriber("/dot4/drop", String, callback)
    # rospy.Subscriber("/dot5/drop", String, callback)
    # rospy.Subscriber("/dot6/drop", String, callback)
    # done_pub1=rospy.Publisher("/dot1/dropdone", Int16, queue_size=0,latch=True)
    # done_pub2=rospy.Publisher("/dot2/dropdone", Int16, queue_size=0,latch=True)
    done_pub3=rospy.Publisher("/dot3/dropdone", Int16, queue_size=0,latch=True)
    done_pub4=rospy.Publisher("/dot4/dropdone", Int16, queue_size=0,latch=True)
    # done_pub5=rospy.Publisher("/dot5/dropdone", Int16, queue_size=0,latch=True)
    # done_pub6=rospy.Publisher("/dot6/dropdone", Int16, queue_size=0,latch=True)
    
    print("******* Drop Node Started *******")
    
    print("******* Drop Node Started *******")
    rospy.spin()
