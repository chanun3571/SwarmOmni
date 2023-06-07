#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
import math 
import datetime
import pandas as pd


class Log:
    def __init__(self):
        rospy.init_node("log")

        self._log_path = rospy.get_param("~log_path","~/Desktop/")
        self._log = []
        self._log_hz = 100
        self._start_t = None
        self.initdone = "WAIT"
        rospy.Subscriber('survey_state', String, self.surveystate_callback)

        self._average_pose_stamped = PoseStamped()
        self._random_pose_stamped = PoseStamped()
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.robot1pose = Pose()
        self.surveystate = "WAIT"
        self.posmag_robot1 = 0
      ##########################################################################
    #### subscriber
    ##########################################################################
    def pose_callback(self, msg):
        print(msg.pose.pose.position.y)
        print(self.robot1pose.position.x)       
        self.robot1pose.position.x = 1.3-float(msg.pose.pose.position.y)
        self.robot1pose.position.y = (float(msg.pose.pose.position.x) +1.5)
        self.robot1pose.orientation = msg.pose.pose.orientation
        self.posmag_robot1 = math.sqrt((self.robot1pose.position.x)**2 + (self.robot1pose.position.y)**2)

    def surveystate_callback(self, msg):       
        self.surveystate = msg.data

    def robotinitdone(self, msg):
        self.initdone = msg.data
        print(msg.data)

    def pose_stamped2yaw(self, robotname):
        euler = tf.transformations.euler_from_quaternion(
            [
                robotname.orientation.x,
                robotname.orientation.y,
                robotname.orientation.z,
                robotname.orientation.w,
            ]
        )
        return euler[2]

    def log(self):
        now = rospy.Time.now()
        if self._start_t is None:
            self._start_t = now
        t = now - self._start_t
        euler_yaw_robot1 = self.pose_stamped2yaw(self.robot1pose)
        data = [
            t.to_sec(),
            self.robot1pose.position.x,
            self.robot1pose.position.y,
            euler_yaw_robot1,
            self.posmag_robot1
        ]
        self._log.append(data)

    def savelog(self, other_str=""):
        now = datetime.datetime.now()
        filename = self._log_path + now.strftime("%Y%m%d_%H%M%S") + other_str
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "time [s]",
                "position_x",
                "position_y",
                "orientation_z",
                "position_magni"
            ],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)

    def spin(self):
        rate = rospy.Rate(self._log_hz)
        while not rospy.is_shutdown():
            if self.initdone == "DONE":
                self.log()
            rate.sleep()
            if self.surveystate == "DONE":
                self.savelog()
                break

if __name__=='__main__':
    try:
        agent = Log()
        agent.spin()
    except rospy.ROSInterruptException:
        pass