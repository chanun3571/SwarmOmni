#!/usr/bin/env python3
import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi
import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64

#############################################################################
class OmniTf:
    def __init__(self):
        rospy.init_node("omni_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = (2**15)/(0.048*pi) # The number of wheel encoder ticks per meter of travel # 1 round = 2^15
        self.base_width = 0.250 # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id', "robot1_tf/base_footprint") # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', "robot1_tf/odom") # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.enc_center = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.center = 0
        self.lmult = 0
        self.rmult = 0
        self.cmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.prev_cencoder = 0
        self.x = 0                  # position in x
        self.y = 0                  # position in y 
        self.th = 0                 # position in theta
        self.dx = 0                 # speed in x
        self.dy = 0                 # speed in y
        self.dr = 0                 # speed in rot
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("lwheel", Int64, self.lwheelCallback)
        rospy.Subscriber("rwheel", Int64, self.rwheelCallback)
        rospy.Subscriber("cwheel", Int64, self.cwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry,queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
                d_center = 0
            else: #distance traveled for each wheel
                d_left = (self.left - self.enc_left) / self.ticks_meter 
                d_right = (self.right - self.enc_right) / self.ticks_meter
                d_center = (self.center - self.enc_center) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
            self.enc_center = self.center
            # distance traveled 
            dx = 0.57736*d_right -0.57736*d_left
            dy =  0.3333*d_right + 0.3333*d_left - 0.6666*d_center
            th = (0.3333*d_right + 0.3333*d_left + 0.3333*d_center )/ (self.base_width/2)
            # dx = (cos(pi/6)*d_right - cos(pi/6)*d_left)
            # dy =  sin(pi/6)*d_right + sin(pi/6)*d_left - d_center
            # th = (d_right + d_left + d_center )/ ((self.base_width/2))
            # calculate velocities (twist)
            self.vx = dx / elapsed
            self.vy = dy / elapsed
            self.vr = th / elapsed
            # calculate current position
            self.x = self.x + ( cos( self.th ) * dx - sin( self.th ) * dy )
            self.y = self.y + ( sin( self.th ) * dx + cos( self.th ) * dy )
            #self.x = self.x + dx
            #self.y = self.y + dy
            self.th = self.th + th            
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id

            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vr
            
            self.odomPub.publish(odom)

    #############################################################################        
    def cwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.cmult = self.cmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.cmult = self.cmult - 1
            
        self.center = 1.0 * (enc + self.cmult * (self.encoder_max - self.encoder_min)) 

        self.prev_cencoder = enc

    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 

        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))

        self.prev_rencoder = enc

#############################################################################
if __name__ == '__main__':
    """ main """
    omniTf = OmniTf()
    omniTf.spin()