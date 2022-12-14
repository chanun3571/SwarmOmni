#!/usr/bin/env python

import getch
import rospy
#rom std_msgs.msg import String#String message 
#from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3

pub = rospy.Publisher('keyboard', Vector3, queue_size=1) # "key" is the publisher name

class Keyboard_Input():
    def __init__(self):
        rospy.init_node('keypress',anonymous=True)
        self.x = 0
        self.y = 0
        self.omega = 0 
        self.button = ""
    def keys(self):
        self.k=ord(getch.getch())
        if self.k==119:
            button = "up"
            self.y = 1
        if self.k==97:
            button = "left"
            self.x = -1
        if self.k==100:
            button = "right"
            self.x = 1
        if self.k==115:
            button = "back"
            self.y= -1
        if self.k==113:
            button = "ccw"
            self.omega = -1
        if self.k==101:
            button = "cw"
            self.omega = 1
        rospy.loginfo(button)
        #w=119, a=97, d=100, s=115, q=113, e=101

    def motion(self,w,vx,vy):
        r = (45/2)/100 #m
        d = 60/100 #m
        #print(w,vx,vy)
        u1 = 1/r*(-d*w + vx)
        u2 = 1/r*(-d*w -1/2*vx -0.866*vy)
        u3 = 1/r*(-d*w -1/2*vx +0.866*vy)
        ros_translation = Vector3()
        ros_translation.x = u1
        ros_translation.y = u2
        ros_translation.z = u3
        pub.publish(ros_translation)
        rospy.loginfo(ros_translation)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            self.keys()
            self.motion(self.omega,self.x,self.y)

if __name__=='__main__':
    try:
        agent=Keyboard_Input()
        agent.spin()
    except rospy.ROSInterruptException:
        pass