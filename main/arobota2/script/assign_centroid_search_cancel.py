#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped, PointStamped
from std_msgs.msg import String, Int32
import time 


class publish_goal_pose_to_robot():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('robot1/flag',String,self.robot1flagcallback, queue_size=1)
        rospy.Subscriber('robot2/flag',String,self.robot2flagcallback, queue_size=1)
        rospy.Subscriber('robot3/flag',String,self.robot3flagcallback, queue_size=1)
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)
        self.pubgoal = rospy.Publisher('/swarm1/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pubgoalpoint = rospy.Publisher('/swarm1/move_base_simple/point', PointStamped, queue_size=1)
        self.pubsend = rospy.Publisher('/swarm1/done', String, queue_size=10)
        rospy.Subscriber('robot1/camera_status', String, self.cam1callback)
        rospy.Subscriber('robot2/camera_status', String, self.cam2callback)
        rospy.Subscriber('robot3/camera_status', String, self.cam3callback)
        self.totalflag = 0 
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        self.flag = False
        self.initdone = "WAIT"
        self.goalpoint = PointStamped()
        self.goalpoint.header.frame_id = "map"
        self.interrupt = "WAIT"

    def robotinitdone(self, msg):
        self.initdone = msg.data
        # print(msg.data)

    def CustomWayPoints(self):
        # Create the dictionary 
        self.locations = dict()
        self.locations['1'] = Point(-0.7, 0.7, 0.000)
        self.locations['2'] = Point(-0.7, 0.3, 0.000)
        self.locations['3'] = Point(-0.6, 0.0, 0.000)
        self.locations['4'] = Point(-0.3, 0.0, 0.000)
        self.locations['5'] = Point(-0.3, -0.4, 0.000)  
        self.locations['6'] = Point(0, -0.2, 0.000)
        self.locations['7'] = Point(0, -0.5, 0.000)
        self.locations['8'] = Point(0.4, -0.8, 0.000)
        self.locations['9'] = Point(0.6, -0.6, 0.000)
        self.locations['10'] = Point(0.8, -0.8, 0.000)
        self.locations['11'] = Point(1, -0.8, 0.000)
        self.locations['11'] = Point(1.3, -1, 0.000)
        self.locations['12'] = Point(1.3, -1.3, 0.000)
        self.locations['13'] = Point(1.3, -1.5, 0.000)
        self.locations['14'] = Point(1.3, -1.8, 0.000)
        self.locations['15'] = Point(1.3, -2.1, 0.000)

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

    def sendGoals(self, waypoints):
        for key, value in waypoints.items():
            self.flag1 = 0
            self.flag2 = 0
            self.flag3 = 0
            self.flag = False
            self.goal = PoseStamped()
            self.goal.header.frame_id = "map"
            self.goal.pose.position.x = waypoints[key].x
            self.goal.pose.position.y = waypoints[key].y
            self.goal.pose.position.z = waypoints[key].z
            self.pubgoal.publish(self.goal)
            self.goalpoint.point=waypoints[key]
            self.pubgoalpoint.publish(self.goalpoint)
            print(self.goal)
            while not self.flag:
                # print(self.totalflag)
                rospy.Rate(5).sleep()
                self.pubgoal.publish(self.goal)
                self.checktotalflag(self.flag1, self.flag2,self.flag3) 
                print(self.flag1,self.flag2,self.flag3)
                if self.flag:
                    break
                if self.interrupt == "STOP":
                    break
            if self.interrupt == "STOP":
                    break
            

    def robot1flagcallback(self,msg):
        self.flag1 = int(msg.data)
    def robot2flagcallback(self,msg):
        self.flag2 = int(msg.data)
    def robot3flagcallback(self,msg):
        self.flag3 = int(msg.data)

    def checktotalflag(self,flag1,flag2,flag3):
        self.totalflag = flag1+flag2+flag3
        if self.totalflag != 3:
            self.flag = False
            # print(self.totalflag)
        if self.totalflag == 3:
            self.pubsend.publish("DONE")
            self.flag= True
            self.totalflag = 0
            print("next goal")

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.initdone == "DONE":
                self.CustomWayPoints()
                self.sendGoals(self.locations)
                break
        print("STOP CENTROID")

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
