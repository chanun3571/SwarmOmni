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
        
        self.marker_pub = rospy.Publisher('ball_pose', Marker, queue_size=2) 
        self.ball_cen_pub = rospy.Publisher('ball_pose_cen', Point, queue_size=2) 

        self.initdone = "WAIT"
        self.ballBroadcaster = ()
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

    def sphere_marker(self):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = 2
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.5
        marker.pose.position.x = self.robotpos.x+self.ball_pose.position.x 
        marker.pose.position.y = self.robotpos.y+self.ball_pose.position.y 
        marker.pose.position.z = 0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Publish the Marker
        self.marker_pub.publish(marker)

    def publish_sphere_pose(self):
        self.sphere_pose = Point()
        self.sphere_pose.x = self.robotpos.x+self.ball_pose.position.x
        self.sphere_pose.y = self.robotpos.y+self.ball_pose.position.y
        

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.camstat == "tracking":
                self.send_ball_pose()
                self.sphere_marker()
                self.publish_sphere_pose()

if __name__=='__main__':
    try:
        agent=create_visual_sphere()
        agent.spin()
    except rospy.ROSInterruptException:
        pass


