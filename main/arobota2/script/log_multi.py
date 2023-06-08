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

        self._log_path = rospy.get_param("~log_path","~/Desktop/")
        self._log = []
        self._log_hz = 100
        self._start_t = None
        self.initdone = "WAIT"
        rospy.Subscriber('survey_state', String, self.surveystate_callback)
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)
        self.robot1pose = Pose()
        self.robot2pose = Pose()
        self.robot3pose = Pose()
        self.surveystate = "WAIT"
    ##########################################################################
    #### subscriber
    ##########################################################################
    def surveystate_callback(self, msg):       
        self.surveystate = msg.data

    def allpose1_callback(self, msg):       
        self.robot1pose.position.x = 1.3-float(msg.pose.pose.position.y)
        self.robot1pose.position.y = (float(msg.pose.pose.position.x) +1.5)
        self.robot1pose.orientation = msg.pose.pose.orientation
        self.posmag_robot1 = math.sqrt((self.robot1pose.position.x)**2 + (self.robot1pose.position.y)**2)
    def allpose2_callback(self, msg):
        self.robot2pose.position.x = 1.3-float(msg.pose.pose.position.y)
        self.robot2pose.position.y = (float(msg.pose.pose.position.x) +1.5)
        self.robot2pose.orientation = msg.pose.pose.orientation
        self.posmag_robot2 = math.sqrt((self.robot2pose.position.x)**2 + (self.robot2pose.position.y)**2)
    def allpose3_callback(self, msg):
        self.robot3pose.position.x = 1.3-float(msg.pose.pose.position.y)
        self.robot3pose.position.y = (float(msg.pose.pose.position.x) +1.5)
        self.robot3pose.orientation = msg.pose.pose.orientation
        self.posmag_robot3 = math.sqrt((self.robot3pose.position.x)**2 + (self.robot3pose.position.y)**2)

    def robotinitdone(self, msg):
        self.initdone = msg.data
        print(msg.data)
        


    ##########################################################################
    #### function
    ##########################################################################

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
        euler_yaw_robot2 = self.pose_stamped2yaw(self.robot2pose)
        euler_yaw_robot3 = self.pose_stamped2yaw(self.robot3pose)
        self.average_euler = (euler_yaw_robot1+euler_yaw_robot2+euler_yaw_robot3)/3
        self.average_pose_x = (self.robot1pose.position.x + self.robot2pose.position.x +self.robot3pose.position.x)/3
        self.average_pose_y = (self.robot1pose.position.y + self.robot2pose.position.y +self.robot3pose.position.y)/3
        self.average_mag= (self.posmag_robot1 + self.posmag_robot2 +self.posmag_robot3 )/3
        data = [
            t.to_sec(),
            self.average_pose_x,
            self.average_pose_y,
            self.average_euler,
            self.average_mag,
            self.robot1pose.position.x,
            self.robot1pose.position.y,
            euler_yaw_robot1,
            self.posmag_robot1,
            self.robot2pose.position.x,
            self.robot2pose.position.y,
            euler_yaw_robot2,
            self.posmag_robot2,
            self.robot3pose.position.x,
            self.robot3pose.position.y,
            euler_yaw_robot3,
            self.posmag_robot3
            ]
        self._log.append(data)

    def savelog(self, other_str=""):
        now = datetime.datetime.now()
        filename = self._log_path + now.strftime("%Y%m%d_%H%M%S") + other_str
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "time [s]",
                "centroid_x",
                "centriod_y",
                "centriod_orientation_z",
                "centriod_aver_mag"
                "position_x_1",
                "position_y_1",
                "orientation_z_1",
                "position_magni_1"
                "position_x_2",
                "position_y_2",
                "orientation_z_2",
                "position_magni_2"               
                "position_x_3",
                "position_y_3",
                "orientation_z_3",
                "position_magni_3"
            ],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)

    def spin(self):
        rate = rospy.Rate(self._log_hz)
        while not rospy.is_shutdown():
            if self.initdone=="DONE":
                self.log()
            rate.sleep()
            if self.surveystate == "DONE":
                self.savelog()

if __name__=='__main__':
    try:
        agent = Log()
        agent.spin()
    except rospy.ROSInterruptException:
        pass