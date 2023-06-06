#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped
from std_msgs.msg import String, Int32

class publish_goal_pose_to_robot1():
    def __init__(self):
        rospy.init_node('search')
        # rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.CustomWayPoints1, queue_size=1)
        rospy.Subscriber('move_base/result',MoveBaseActionResult,self.failcallback1, queue_size=1)
        rospy.Subscriber('interruptsignal', String, self.robotinterrupt,queue_size=10)
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)
        # self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.msg1 = "WAIT"
        self.initdone = "WAIT"
        self.interrupt = "FALSE"
        self.locations = dict()
        self.flag1 = 0

    def robotinterrupt(self, msg):
        self.interrupt = msg.data

    def robotinitdone(self, msg):
        self.initdone = msg.data
        print(msg.data)

    def CustomWayPoints1(self):
        # Create the dictionary 
        # self.locations['waypoint1'] = Pose(msg.goal.target_pose.pose.position, msg.goal.target_pose.pose.orientation)
        self.locations = dict()
        self.locations['1'] = Pose(Point(-0.7, 0.7, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['2'] = Pose(Point(-0.7, 0.3, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['3'] = Pose(Point(-0.6, 0.0, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['4'] = Pose(Point(-0.45, -0.0, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))      
        self.locations['5'] = Pose(Point(-0.3, 0.0, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['6'] = Pose(Point(-0.3, -0.4, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))  
        self.locations['7'] = Pose(Point(0, -0.5, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['8'] = Pose(Point(0, -0.2, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['9'] = Pose(Point(0.4, -0.8, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['10'] = Pose(Point(0.6, -0.6, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['11'] = Pose(Point(0.8, -0.8, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['12'] = Pose(Point(1, -0.8, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['13'] = Pose(Point(1.2, -1, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['14'] = Pose(Point(1.2, -1.3, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['15'] = Pose(Point(1.2, -1.5, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['16'] = Pose(Point(1.2, -1.8, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['17'] = Pose(Point(1.3, -2, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
    def sendGoals(self, waypoints):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        
        # Iterate over all the waypoits, follow the path 
        for key, value in waypoints.items():
            self.msg1 = "WAIT"
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = waypoints[key].position.x
            goal.target_pose.pose.position.y = waypoints[key].position.y
            goal.target_pose.pose.position.z = waypoints[key].position.z
            # Goal Orientation
            goal.target_pose.pose.orientation.x = waypoints[key].orientation.x
            goal.target_pose.pose.orientation.y = waypoints[key].orientation.y
            goal.target_pose.pose.orientation.z = waypoints[key].orientation.z
            goal.target_pose.pose.orientation.w = waypoints[key].orientation.w
            print(goal)
            client.send_goal(goal)
            while self.msg1 != "Goal reached.":
                if self.interrupt == "STOP":
                    client.cancel_goal()
                    print("cancel_goal")
                    break
            if self.interrupt == "STOP":
                client.cancel_all_goals()
                print("cancel_all_goals")
                break

                
    def failcallback1(self, msg):
        rospy.loginfo(msg.status.text)
        self.msg1 = msg.status.text
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag1 = 1
            # rospy.loginfo("robot1")
            print(self.flag1)
        # if msg.status.text== "Goal reached." :
        #     self.flag1 = 0
            
    def resubmit1(self):
        if self.flag1 == 1:
            self.flag1 = 0
            self.sendGoals(self.locations)
            rospy.loginfo(self.locations)
            rospy.loginfo("resubmit robot #1")
            print(self.flag1)
            # rospy.Rate(0.5).sleep()
            self.flag1 = 0
            print(self.flag1)

    def spin(self):
        while not rospy.is_shutdown():
            if self.initdone == "DONE":
                self.CustomWayPoints1()
                self.sendGoals(self.locations)
                break


if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot1()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
