#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import String
import transformations

class cancel_goal():
    def __init__(self):
        rospy.init_node('publish_ball_pose')
        rospy.Subscriber('/depth', String, self.depth_callback, queue_size=1)
        rospy.Subscriber('/x',String, self.x_callback, queue_size=1)
        rospy.Subscriber('/camera_status', String, self.camera_status, queue_size=1)
        self.camstat = "WAITING"
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.allpose_callback)
        # rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        # rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        # rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)

        # self.robot1 = Point()
        # self.robot2 = Point()
        # self.robot3 = Point()
        self.initdone = "WAIT"
        self.ballBroadcaster = ()

    def send_ball_pose(self):
        self.ballBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
        eulertoquaternion
                quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
    def allpose_callback(self, msg):       
        self.robot = msg.pose.pose.position
    # def allpose1_callback(self, msg):       
    #     self.robot1 = msg.pose.pose.position
    # def allpose2_callback(self, msg):
    #     self.robot2 = msg.pose.pose.position
    # def allpose3_callback(self, msg):
    #     self.robot3 = msg.pose.pose.position

    def point_callback(self, msg):
        self.point = msg

    def formation_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.02
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.02

        # marker line points
        marker.points = []
        # first point
        first_line_point = self.robot1
        second_line_point = self.robot2
        third_line_point = self.robot3
        forth_line_point = self.robot1
        marker.points.append(first_line_point)
        marker.points.append(second_line_point)
        marker.points.append(third_line_point)
        marker.points.append(forth_line_point)

        # Publish the Marker
        self.marker_pub.publish(marker)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.initdone == "DONE":
                self.formation_marker()

if __name__=='__main__':
    try:
        agent=createvisuallinepoint()
        agent.spin()
    except rospy.ROSInterruptException:
        pass


    def moveforward(self):
        self.joy_ux = 0.3
        self.joy_uy = 0
        self.joy_omega = 0
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)

    def stopmotion(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = 0
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)
    def counterrotate(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = 3
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)
    def rotate(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = -3
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)

    def allpose_callback(self, msg):       
        self.robot = msg.pose.pose.position

    def x_callback(self,msg):
        self.x = msg.data

    def depth_callback(self,msg):
        self.depth = msg.data 

    def camera_status(self,msg):
        self.camstat = msg.data

    def navtoball(self):
            # rate = rospy.Rate()
            print(self.camstat)
            self.stopmotion()
            # print(self.depth)
            while self.camstat == "tracking":
                # rate.sleep()
                # if self.camstat == "not_found":
                #     self.stopmotion()
                #     break
                # if self.camstat == "not_tracking":
                #     if self.x < 10:
                #         self.rotate()
                #     if self.x > 18:
                #         self.counterrotate()
                if float(self.depth)<45:
                    self.stopmotion()
                    break
                if float(self.depth)>45:
                    self.stopmotion()
                    break
                if self.camstat != "tracking":
                    self.stopmotion()

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.camstat == "tracking":
                self.pubinterrupt.publish("STOP")
                print("canceled goal")
                # self.cancel_pub.publish(self.cancel_msg)
                self.navtoball()
                rate.sleep()
                break
                
if __name__=='__main__':
    try:
        agent=cancel_goal()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
