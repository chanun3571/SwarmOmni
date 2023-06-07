#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped

import datetime
import pandas as pd
import math


class Log:
    def __init__(self):
        rospy.init_node("log")

        self._log_path = rospy.get_param("~log_path")
        self._log = []
        self._log_hz = 100
        self._start_t = None
        self._initdone = "WAIT"
        rospy.Subscriber('survey_state', String, self.surveystate_callback)

        self._uh = Twist()
        self._average_pose_stamped = PoseStamped()
        self._random_pose_stamped = PoseStamped()
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)
        rospy.Subscriber("uh", Twist, self.uh_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.robot1pose = Pose()
        self.surveystate = "WAIT"
      ##########################################################################
    #### subscriber
    ##########################################################################
    def pose_callback(self, msg):       
        self.robot1pose = msg.pose.pose

    def surveystate_callback(self, msg):       
        self.surveystate = msg.data

    def pose_callback(self, msg):       
        self.robot1pose = msg.pose.pose

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
            euler_yaw_robot1
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
            ],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)

    def spin(self):
        rate = rospy.Rate(self._log_hz)
        while not rospy.is_shutdown():
            if self.initdone:
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