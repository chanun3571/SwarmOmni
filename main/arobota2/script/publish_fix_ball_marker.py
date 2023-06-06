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
        rospy.init_node('ball_marker')      
        self.marker_pub = rospy.Publisher('ball_pose', Marker, queue_size=2) 
        rospy.Subscriber('ball_status', String, self.ball_status, queue_size=1)
        rospy.Subscriber('ball_pose_cen', Point, self.ball_pose, queue_size=1) 
        self.ballpose = Point()
        self.ballstat = "WAIT"

    def ball_status(self,msg):
        self.ballstat = msg.data 
    def ball_pose(self,msg):
        self.ballpose = msg

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
        marker.pose.position.x = self.ballpose.x
        marker.pose.position.y = self.ballpose.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Publish the Marker
        self.marker_pub.publish(marker)


    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.ballstat == "DETECTED":
                self.sphere_marker()
                

if __name__=='__main__':
    try:
        agent=create_visual_sphere()
        agent.spin()
    except rospy.ROSInterruptException:
        pass


