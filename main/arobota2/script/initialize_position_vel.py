#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from xbox_button import XBoxButton
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from math import sin, cos, pi, sqrt
#from std_msgs.msg import String#String message 
#from std_msgs.msg import Int8
#from geometry_msgs.msg import Vector3


class Initialize_Pos():
    def __init__(self):
        rospy.init_node('joystickinput1',anonymous=True)
        self._uh = Twist()
        self.pubvel =  rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)
        # self.t_delta = rospy.Duration(1.0/100)
        # self.t_next = rospy.Time.now() + self.t_delta
        # self.then = rospy.Time.now()
        self.now = rospy.Time.now()
        self.then = rospy.Time.now()
        # rospy.loginfo("Current time %i %i", self.now.secs, self.now.nsecs)
        # rospy.loginfo("Current time %i %i", self.then.secs, self.then.nsecs)

    def move1(self):
        self.then = rospy.Time.now()
        # rospy.loginfo("Current time %i", self.then.secs)
        self.joy_ux = -0.3
        self.joy_uy = 0
        self.joy_omega = 0 
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        self.pubvel1.publish(self._uh)
        self.pubvel2.publish(self._uh)
        self.pubvel3.publish(self._uh)



    def move2(self):
        self.then = rospy.Time.now()
        # rospy.loginfo("Current time %i", self.then.secs)
        self.joy_ux = 0
        self.joy_uy = 0.3
        self.joy_omega = 0
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        self.pubvel1.publish(self._uh)
        self.pubvel2.publish(self._uh)
        self.pubvel3.publish(self._uh)

    def stop(self):
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
        while not rospy.is_shutdown():
            self.time=self.then.secs-self.now.secs
            self.move1()
            # rospy.loginfo(self.time)
            if self.time<3:
                self.stop()
            # if self.time>3:
            #     self.move2()
            #     if self.time>5:
            #         rospy.loginfo("DONE2")
            #         self.stop()
            #         break

if __name__=='__main__':
    try:
        agent=Initialize_Pos()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
