#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import Int32,String

class publish_goal_pose_to_robot3():
    def __init__(self):
        rospy.init_node('custom_waypoints3')
        rospy.loginfo('start robot3')
        rospy.Subscriber('/robot3_formation_pos_ball', Pose, self.waypoint)
        rospy.Subscriber('/robot3/move_base/result',MoveBaseActionResult,self.failcallback3)
        self.flag = rospy.Publisher('/robot3/flag', String, queue_size=1)
        self.interrupt = "WAIT"        
        self.done = "WAIT"
        self.locations = dict()
        self.flag3 = 0
        self.ready = False

    def waypoint(self, msg):
        self.pose = msg
        self.ready = True
        
    def sendGoals(self, waypoint):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot3/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        if self.ready:
            self.reached = False
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose
            client.send_goal(goal)
            wait = client.wait_for_result()



    def failcallback3(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        rospy.loginfo(msg.status.text)
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag3 = 1
            self.sendGoals(self.locations)
            rospy.loginfo("robot3")
            print(self.flag3)
        if msg.status.text=="Goal reached.":
            self.reached = True
            
    def resubmit3(self):
        if self.flag3 == 1:
            self.flag3 = 0
            self.sendGoals(self.locations)
            rospy.loginfo("resubmit robot #3")
            print(self.flag3)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.ready:
                self.sendGoals(self.locations)
                break
            # self.resubmit1()
            rospy.Rate(5).sleep()


if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot3()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
