#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray,Pose, Point, PoseStamped, Pose, Quaternion
from agent_util_multi import allPosi_allOrien
from std_msgs.msg import String
import math
from numpy_ros import to_numpy, to_message

class assign_centroid():
    def __init__(self):
        rospy.init_node('centroid') 
        rospy.loginfo("start centroid")
        # self._agent_util = allPosi_allOrien()
        # self.allPosition = self._agent_util.allPosition()
        self.pubpoint1 = rospy.Publisher('/robot1_formation_pos', Pose, queue_size=1)
        self.pubpoint2 = rospy.Publisher('/robot2_formation_pos', Pose, queue_size=1)
        self.pubpoint3 = rospy.Publisher('/robot3_formation_pos', Pose, queue_size=1)
        self.pubcentroid = rospy.Publisher('/centroid', Pose, queue_size=1)
        rospy.Subscriber('/swarm1/move_base_simple/goal', PoseStamped, self.submit_centroid)
        rospy.Subscriber('robot1/camera_status', String, self.cam1callback)
        rospy.Subscriber('robot2/camera_status', String, self.cam2callback)
        rospy.Subscriber('robot3/camera_status', String, self.cam3callback)
        self.ready = False
        self.centroid_pose = Pose()
        self.interrupt = "WAIT"


    def submit_centroid(self, msg):
        self.centroid_pose.position.x = msg.pose.position.x
        self.centroid_pose.position.y = msg.pose.position.y
        self.centroid_pose.orientation.z = msg.pose.orientation.z
        self.centroid_pose.orientation.w = msg.pose.orientation.w
        self.pubcentroid.publish(self.centroid_pose)
        self.ready = True

        # print(self.centroid_pose.position.x)
        # print(self.centroid_pose.position.y)


    def findpos(self):
        if self.ready:
            # rospy.loginfo(self.centroid_pose)
            self.centroid = to_numpy(self.centroid_pose.position)
            d1 = np.array([(-0.25)*math.cos(math.pi/6),(-0.25)*math.sin(math.pi/6),0])  
            d2 = np.array([0,0.25,0])
            d3 = np.array([0.25*math.cos(math.pi/6),-0.25*math.sin(math.pi/6),0])
            p_robot1 = to_message(Point,self.centroid + d1)
            p_robot2 = to_message(Point,self.centroid + d2)
            p_robot3 = to_message(Point,self.centroid + d3)
            q_robot1 = Quaternion(0,0,-0.9659258,0.258819)
            q_robot2 = Quaternion(0,0,-0.7071068,-0.7071068)
            q_robot3 = Quaternion(0,0,-0.258819,0.9659258)
            self.pubpoint1.publish(Pose(p_robot1, q_robot1))
            self.pubpoint2.publish(Pose(p_robot2, q_robot2))
            self.pubpoint3.publish(Pose(p_robot3, q_robot3))
            self.ready = False
    
    def cam1callback(self, msg):
        self.robot1_camstat = msg.data 
        if self.robot1_camstat == "tracking":
            self.interrupt = "STOP"
    def cam2callback(self, msg):
        self.robot2_camstat = msg.data 
        if self.robot2_camstat == "tracking":
            self.interrupt = "STOP"
    def cam3callback(self, msg):
        self.robot3_camstat = msg.data 
        if self.robot3_camstat == "tracking":
            self.interrupt = "STOP"

    def spin(self):
        while not rospy.is_shutdown():
            self.findpos()
            if self.interrupt == "STOP":
                break
            rospy.Rate(5).sleep()
        print("STOP centroid")


if __name__=='__main__':
    try:
        agent = assign_centroid()
        agent.spin()
    except rospy.ROSInterruptException:
        pass







