#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, PoseStamped

import datetime
import pandas as pd
import math


class Log:
    def __init__(self):

        self._log_path = rospy.get_param("~log_path")
        self._log = []
        self._log_hz = 100
        self._start_t = None
        self._main_start = False

        self._uh = Twist()
        self._average_pose_stamped = PoseStamped()
        self._random_pose_stamped = PoseStamped()
        rospy.Subscriber("startmain", Bool, self.maincontrol_callback)
        rospy.Subscriber("uh", Twist, self.uh_callback)
        rospy.Subscriber(
            "centralpose", PoseStamped, self.average_pose_callback, queue_size=1
        )
        rospy.Subscriber(
            "randompose", PoseStamped, self.random_pose_callback, queue_size=1
        )
        rospy.Subscriber("landingstatus", Bool, self.landing_callback, queue_size=1)

    ##########################################################################
    #### subscriber
    ##########################################################################
    def maincontrol_callback(self, msg):
        if msg.data == True:
            rospy.loginfo("main start")
            self._main_start = True

            # self._take_off = False
            # self._land = False
            
    def landing_callback(self, msg):
        if msg.data == True:
            rospy.loginfo("land")
            self._main_start = False
            self._take_off = False
            self._land = True
            self.savelog()

    def landing_callback(self, msg):
        if msg.data == True:
            rospy.loginfo("land")
            self._main_start = False
            self._take_off = False
            self._land = True
            self.savelog()

    def uh_callback(self, msg):
        self._uh = msg

    def average_pose_callback(self, msg):
        self._average_pose_stamped = msg

    def random_pose_callback(self, msg):
        self._random_pose_stamped = msg

    def spin(self):
        rate = rospy.Rate(self._log_hz)
        while not rospy.is_shutdown():
            if self._main_start:
                self.log()
            rate.sleep()
        # self.savelog()

    def pose_stamped2yaw(self, pose_stamped):
        average_orientation = pose_stamped.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [
                average_orientation.x,
                average_orientation.y,
                average_orientation.z,
                average_orientation.w,
            ]
        )
        return euler[2]

    def log(self):
        now = rospy.Time.now()
        if self._start_t is None:
            self._start_t = now
        t = now - self._start_t
        average_position = self._average_pose_stamped.pose.position
        # rospy.loginfo(average_position)
        average_yaw = self.pose_stamped2yaw(self._average_pose_stamped)

        random_position = self._random_pose_stamped.pose.position
        random_yaw = self.pose_stamped2yaw(self._random_pose_stamped)

        error_x = random_position.x - average_position.x
        error_y = random_position.y - average_position.y
        error_z = random_position.z - average_position.z
        error_w = random_yaw - average_yaw
        if error_w > math.pi:
            error_w -= 2 * math.pi
        elif error_w < -math.pi:
            error_w += 2 * math.pi

        # rospy.loginfo(error_x)
        data = [
            t.to_sec(),
            self._uh.linear.x,
            self._uh.linear.y,
            self._uh.linear.z,
            self._uh.angular.z,
            error_x,
            error_y,
            error_z,
            error_w,
        ]
        self._log.append(data)

    def savelog(self, other_str=""):
        now = datetime.datetime.now()
        filename = self._log_path + now.strftime("%Y%m%d_%H%M%S") + other_str
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "time [s]",
                "uh_x",
                "uh_y",
                "uh_z",
                "uh_w",
                "error_x",
                "error_y",
                "error_z",
                "error_w",
            ],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)


if __name__ == "__main__":
    rospy.init_node("log")
    node = Log()
    node.spin()