#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import String
from math import sin, cos, pi
import tf
from numpy_ros import to_numpy, to_message

class create_visual_sphere():
    def __init__(self):
        rospy.init_node('publish_ball_pose')
        rospy.Subscriber('robot3/depth', String, self.depth_callback, queue_size=1)
        rospy.Subscriber('robot3/camera_status', String, self.camera_status, queue_size=1)
        self.camstat = "WAIT"
        rospy.Subscriber('robot3/amcl_pose', PoseWithCovarianceStamped, self.allpose_callback, queue_size=1)
        
        self.ball_cen_pub = rospy.Publisher('ball_pose_cen', Point, queue_size=2) 
        self.ball_status = rospy.Publisher('ball_status', String, queue_size=2) 

        self.ballstat = "WAIT"
        self.initdone = "WAIT"
        self.ball_pose = Pose()
        self.robotpos=Point()
        self.robotquat=Quaternion()
        self.depth= 0

    def depth_callback(self,msg):
        self.depth = msg.data 
    def camera_status(self,msg):
        self.camstat = msg.data      

    def allpose_callback(self, msg):       
        self.robotpos = msg.pose.pose.position
        self.robotquat = msg.pose.pose.orientation
        self.robotangle = tf.transformations.euler_from_quaternion([self.robotquat.x,self.robotquat.y,self.robotquat.z,self.robotquat.w])
        self.robotangle = self.robotangle[2] 
        # print(self.robotangle)   

    def send_ball_pose(self):
        self.ball_pose.position.x = (float(self.depth)*cos(self.robotangle))/100
        self.ball_pose.position.y = (float(self.depth)*sin(self.robotangle))/100
        print(self.ball_pose.position)

    def publish_sphere_pose(self):
        self.sphere_pose = Point()
        self.sphere_pose.x =  0.6143803111228972
        self.sphere_pose.y = -1.0637885177647733
        self.ball_cen_pub.publish(self.sphere_pose)
        self.ball_stat = "DETECTED"
        self.ball_status.publish(self.ball_stat)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.camstat == "tracking":
                self.send_ball_pose()
                self.publish_sphere_pose()
                break

if __name__=='__main__':
    try:
        agent=create_visual_sphere()
        agent.spin()
    except rospy.ROSInterruptException:
        pass


