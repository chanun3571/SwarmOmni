#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray,Pose, Point, PoseStamped, Pose, Quaternion
from agent_util_multi import allPosi_allOrien
from std_msgs.msg import String, Bool
import math
from numpy_ros import to_numpy, to_message

class assign_centroid():
    def __init__(self):
        rospy.init_node('centroid') 
        rospy.loginfo("start centroid")
        # self._agent_util = allPosi_allOrien()
        # self.allPosition = self._agent_util.allPosition()
        self.pubpoint1 = rospy.Publisher('/robot1_formation_pos_ball', Pose, queue_size=1)
        self.pubpoint2 = rospy.Publisher('/robot2_formation_pos_ball', Pose, queue_size=1)
        self.pubpoint3 = rospy.Publisher('/robot3_formation_pos_ball', Pose, queue_size=1)
        self.pubcentroid = rospy.Publisher('/centroid', Pose, queue_size=1)
        # self.pubready = rospy.Publisher('/ready', Bool, queue_size=1)

        rospy.Subscriber('/ball_pose_cen', Point, self.ball_pose)
        self.ready = False
        self.centroid_pose = Pose()
        self.interrupt = "WAIT"

    def ball_pose(self, msg):
        self.centroid_pose.position = msg
        self.pubcentroid.publish(self.centroid_pose)
        self.ready = True
        # self.pubready.publish(self.ready)

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
            q_robot1 = Quaternion(0,0,0.9659258,0.258819)
            q_robot2 = Quaternion(0,0,0.7071068,-0.7071068)
            q_robot3 = Quaternion(0,0,0.258819,0.9659258)
            self.pubpoint1.publish(Pose(p_robot1, q_robot1))
            self.pubpoint2.publish(Pose(p_robot2, q_robot2))
            self.pubpoint3.publish(Pose(p_robot3, q_robot3))
            # self.pubpoint1.publish(to_message(Pose, p_robot1))
            # self.pubpoint2.publish(to_message(Pose, p_robot2))
            # self.pubpoint3.publish(to_message(Pose, p_robot3))
    

    def spin(self):
        while not rospy.is_shutdown():
            if self.ready:
                self.findpos()
            rospy.Rate(5).sleep()
        print("STOP centroid")


if __name__=='__main__':
    try:
        agent = assign_centroid()
        agent.spin()
    except rospy.ROSInterruptException:
        pass







