#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import Int32,String

class publish_goal_pose_to_robot3():
    def __init__(self):
        rospy.init_node('custom_waypoints3')
        rospy.loginfo('start robot3')
        rospy.Subscriber('/robot3_formation_pos', Pose, self.waypoint)
        rospy.Subscriber('/robot3/move_base/result',MoveBaseActionResult,self.failcallback3)
        self.flag = rospy.Publisher('/robot3/flag', String, queue_size=1)
        rospy.Subscriber('/swarm1/done', String, self.donecallback)
        rospy.Subscriber('robot1/camera_status', String, self.cam1callback)
        rospy.Subscriber('robot2/camera_status', String, self.cam2callback)
        rospy.Subscriber('robot3/camera_status', String, self.cam3callback)

        self.interrupt = "WAIT"        
        self.done = "WAIT"
        self.locations = dict()
        self.flag3 = 0
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


    def donecallback(self,msg):
        self.done = msg.data
        # print(self.done)

    def waypoint(self, msg):
        self.pose = msg
        self.ready = True
        
    def sendGoals(self, waypoint):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot3/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        if self.ready:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose
            client.send_goal(goal)
            wait = client.wait_for_result()
            while self.done != "DONE":
                if wait and self.reached:
                    self.flag_done = "1"
                    self.flag.publish(self.flag_done)
                    # rospy.loginfo("robot3 done")
                    self.done = self.done
                if self.done == "DONE":
                    self.flag_done = "0"
                    self.flag.publish(self.flag_done)
                    self.done= "WAIT"
                    break
                if self.interrupt == "STOP":
                    client.cancel_all_goals()
                    client.cancel_goal()
                    print("cancel_goal")
                    break


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
            self.sendGoals(self.locations)
            if self.interrupt == "STOP":
                break
            # self.resubmit3()
            rospy.Rate(5).sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot3()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
