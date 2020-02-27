#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('car_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    <    >

anything else : stop

q/a : increase/decrease only linear speed by 10%
w/s : increase/decrease only angular speed by 10%

controlling column:
   r   t
   f   g
   v   b

CTRL-C to quit
"""

moveBindings = {
        'u':( 1, 0, 0, 1), # left front
        'i':( 1, 0, 0, 0), # front
        'o':( 1, 0, 0,-1), # right front
        'j':( 0, 0, 0, 1), # left
        'k':( 0, 0, 0, 0), # stop
        'l':( 0, 0, 0,-1), # right
        'm':(-1, 0, 0,-1), # left back
        ',':(-1, 0, 0, 0), # back
        '.':(-1, 0, 0, 1), # right back
    }

speedBindings={
        'q':( 1.1, 1.0), # increase linear speed
        'a':( 0.9, 1.0), # decrease linear speed
        'w':( 1.0, 1.1), # increase angular speed
        's':( 1.0, 0.9), # decrease angular speed
    }

controlBindings={
        'r':( 1, 0, 0), # link1 clockwise
        't':(-1, 0, 0), # link1 anti-clockwise
        'f':( 0, 1, 0), # link2 clockwise
        'g':( 0,-1, 0), # link2 anti-clockwise
        'v':( 0, 0, 1), # link3 clockwise
        'b':( 0, 0,-1), # link3 anti-clockwise
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def column(link1, link2, link3):
    return "link1: %s\tlink2: %s\tlink3 %s " % (link1, link2, link3)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    pub_link1 = rospy.Publisher('/car_model/joint_column_link1_controller/command', Float64, queue_size = 1)
    pub_link2 = rospy.Publisher('/car_model/joint_column_link2_controller/command', Float64, queue_size = 1)
    pub_link3 = rospy.Publisher('/car_model/joint_column_link3_controller/command', Float64, queue_size = 1)
    rospy.init_node('car_keyboard')
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 5.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    link1 = 0
    link2 = 0
    link3 = 0

    try:
        print(msg)
        print(vels(speed, turn))
        print(column(link1, link2, link3))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in controlBindings.keys():
                link1 = link1 + 0.1 * controlBindings[key][0]
                link2 = link2 + 0.1 * controlBindings[key][1]
                link3 = link3 + 0.1 * controlBindings[key][2]
                if link1<0.0001 and link1>-0.0001:
                    link1 = 0.0
                if link2<0.0001 and link2>-0.0001:
                    link2 = 0.0
                if link3<0.0001 and link3>-0.0001:
                    link3 = 0.0
                print(column(link1, link2, link3))
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub_move.publish(twist)

            float64 = Float64()
            float64.data = link1
            pub_link1.publish(float64)
            float64.data = link2
            pub_link2.publish(float64)
            float64.data = link3
            pub_link3.publish(float64)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub_move.publish(twist)
        float64 = Float64()
        float64.data = 0
        pub_link1.publish(float64)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
