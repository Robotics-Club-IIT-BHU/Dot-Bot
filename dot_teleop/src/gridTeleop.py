#!/usr/bin/env python3


import roslib
roslib.load_manifest('dot_teleop')
import rospy

from geometry_msgs.msg import Twist
import tf
import math
import sys, select, termios, tty

msg = """
Control Your dot!
---------------------------
Moving around:
   _    i    _
   j    k    l
   _    ,    _
q/z : increase/decrease max speeds by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

bot=""

def getVelocity(speed,x,y):
    tfl=tf.TransformListener()
    tfl.waitForTransform(bot+"/origin_link","/map",rospy.Time(0),rospy.Duration(4.0))
    (trans,rot)=tfl.lookupTransform(bot+"/origin_link","/map",rospy.Time(0))
    angle=tf.transformations.euler_from_quaternion(rot)[2]
    if(x!=0):
        speedx=speed*math.cos(angle)*x
        speedy=speed*math.sin(angle)*x
    elif(y!=0):
        speedy=speed*math.cos(angle)*y
        speedx=speed*math.sin(angle)*y
    return speedx,speedy

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('dot_teleop')
    pub = rospy.Publisher(bot+"/cmd_vel", Twist)

    x = 0
    y = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                y = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                if (key == '\x03'):
                    break

            target_speed = speed

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x , twist.linear.y = getVelocity(speed,x,y); 
            twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

