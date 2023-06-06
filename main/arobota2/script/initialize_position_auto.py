#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from xbox_button import XBoxButton
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from math import sin, cos, pi, sqrt
from std_msgs.msg import String
#from std_msgs.msg import Int8
#from geometry_msgs.msg import Vector3


class Initialize_Pos():
    def __init__(self):
        rospy.init_node('joystickinput1',anonymous=True)
        self._uh = Twist()
        self.pubinitdone = rospy.Publisher('initialize_state',String, queue_size=10)
        self.pubvel =  rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)
        # self.t_delta = rospy.Duration(1.0/100)
        # self.t_next = rospy.Time.now() + self.t_delta
        # self.then = rospy.Time.now()
        rospy.Rate(20).sleep()
        self.now = rospy.Time.now()
        self.then = rospy.Time.now()
        # rospy.loginfo("Current time %i %i", self.now.secs, self.now.nsecs)
        # rospy.loginfo("Current time %i %i", self.then.secs, self.then.nsecs)

    def rotate(self):
        self.then = rospy.Time.now()
        # rospy.loginfo("Current time %i", self.then.secs)
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = 3
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        self.pubvel1.publish(self._uh)
        self.pubvel2.publish(self._uh)
        self.pubvel3.publish(self._uh)

    def stoprotate(self):
        joy_ux = 0
        joy_uy = 0
        joy_omega = 0
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        self.pubvel1.publish(self._uh)
        self.pubvel2.publish(self._uh)
        self.pubvel3.publish(self._uh)

    def spin(self):
        # initialize message
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            self.time=self.then.secs-self.now.secs
            # rospy.loginfo(self.time)
            self.rotate()
            if self.time>15: #time to rotate
                self.stoprotate()
                self.pubinitdone.publish("DONE")
                print("ROTATION END")
                break            
            
if __name__=='__main__':
    try:
        agent=Initialize_Pos()
        agent.spin()
    except rospy.ROSInterruptException:
        pass