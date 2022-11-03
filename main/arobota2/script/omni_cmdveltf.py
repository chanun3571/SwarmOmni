#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist 

class TwistToVel():
    def __init__(self):
        rospy.init_node("twist_to_velocity")
        # self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        # self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        self.pub_motor = rospy.Publisher('wheel_vtarget', String,queue_size=10)
        rospy.Subscriber('/cmd_velocity', Twist, self.twistCallback)
        self.center = 0
        self.left = 0
        self.right = 0 
        self.wheelrad = 24 * (10**-3) #wheelradius=24 mm
        self.base = rospy.get_param("~base_width", 200*(10^-3)) #radius=100 mm
   
    def twistCallback(self,msg):
        self.dx = msg.linear.x
        self.dy = msg.linear.y
        self.dr = msg.angular.z
        self.right = (2*self.dx-(self.dr*self.base))/(2*self.wheelrad)
        self.left = (2*self.dx+(self.dr*self.base))/(2*self.wheelrad)
        self.center = 0
        # dx = (l + r) / 2
        # dr = (r - l) / w
        # self.right = 1.0 * self.dx + self.dr * self.w / 2 
        # self.left = 1.0 * self.dx - self.dr * self.w / 2
        x = str(self.left)+","+str(self.right)+","+str(self.center)
        self.pub_motor.publish(x)
        # self.pub_lmotor.publish(self.left)
        # self.pub_rmotor.publish(self.right)
        # rospy.spin()

if __name__ == '__main__':
    """ main """
    TwistToVel()
    rospy.spin()