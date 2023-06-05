#!/usr/bin/env python
import sys
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

# Functions
import HSV_filter as hsv
import shape_recognition as shape
import triangulation as tri
import calibration

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError

class color_shape_detection_1:
    #convert the ROS Image message to a CV2 Image
    def __init__(self):
        rospy.init_node('robot1_camera_shape_color_cv',anonymous=True)
        self.pubz = rospy.Publisher('robot1/depth', String, queue_size=10)
        self.pubx = rospy.Publisher('robot1/x', String, queue_size=10)
        rospy.Subscriber("robot1/image/compressed", CompressedImage, self.image_callback)
        self.bridge = CvBridge()
        self.pubimgstatus = rospy.Publisher('robot1/camera_status', String, queue_size=10 )
        # Stereo vision setup parameters
        self.frame_rate = 30    #Camera frame rate (maximum at 120 fps)
        self.B = 6               #Distance between the cameras [cm]
        # f = 150            #Camera lense's focal length [mm]
        self.alpha = 30        #Camera field of view in the horisontal plane [degrees]
        self.cv_image = np.array([])
        self.flag=0

    def image_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError:
            rospy.loginfo("img_Failed")

    def opencv(self, img):
        try:
            self.frame_left = img[:360,0:640]
            self.frame_right = img[:360,640:]
            # Calibration
            # self.frame_right, self.frame_left = calibration.undistortRectify(self.frame_right, self.frame_left)
            self.mask_right = hsv.add_HSV_filter(self.frame_right, 1)
            self.mask_left = hsv.add_HSV_filter(self.frame_left, 0)

            # Result-frames after applying HSV-filter mask
            res_right = cv2.bitwise_and(self.frame_right, self.frame_right, mask=self.mask_right)
            res_left = cv2.bitwise_and(self.frame_left, self.frame_left, mask=self.mask_left) 

            # APPLYING SHAPE RECOGNITION:
            circles_right, radius_right = shape.find_circles(self.frame_right, self.mask_right)
            circles_left, radius_left  = shape.find_circles(self.frame_left, self.mask_left)

            # Hough Transforms can be used aswell or some neural network to do object detection


            ################## CALCULATING BALL DEPTH #########################################################

            # If no ball can be caught in one camera show text "TRACKING LOST"
            if np.all(circles_right) == None or np.all(circles_left) == None:
                cv2.putText(self.frame_right, "NOT FOUND", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                cv2.putText(self.frame_left, "NOT FOUND", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                self.pubimgstatus.publish("not found")
            else:
                # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
                # All formulas used to find depth is in video presentaion
                self.x, self.depth = tri.find_depth(circles_right, circles_left, self.frame_right, self.frame_left, self.B, self.alpha)
                if 55<self.depth<90 and 77<radius_right<160 and 77<radius_left<160 and 10<self.x<30:
                    cv2.putText(self.frame_right, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    cv2.putText(self.frame_left, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    cv2.putText(self.frame_right, "Distance: " + str(round(self.depth,2)), (300,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    cv2.putText(self.frame_left, "Distance: " + str(round(self.depth,2)), (300,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    cv2.putText(self.frame_right, "X: " + str(round(self.x,2)), (500,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    cv2.putText(self.frame_left, "X: " + str(round(self.x,2)), (500,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.pubx.publish(str(self.x))
                    self.pubz.publish(str(self.depth))
                    self.pubimgstatus.publish("tracking")
                else:
                    cv2.putText(self.frame_right, "DETECTED: NOT TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                    cv2.putText(self.frame_left, "DETECTED: NOT TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                    self.pubimgstatus.publish("not_tracking")
                    self.pubx.publish(str(self.x))
                    self.pubz.publish(str(self.depth))
                    
            cv2.imshow("frame right", self.frame_right) 
            cv2.imshow("frame left", self.frame_left)
            cv2.imshow("mask right", self.mask_right) 
            cv2.imshow("mask left", self.mask_left)  
            # print(radius_right, radius_left) 
            cv2.waitKey(3)
        except:
            print("cv_Failed") 
                                   
    def spin(self):
        # initialize message
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.opencv(self.cv_image)
            rate.sleep()

if __name__=='__main__':
    try:
        agent=color_shape_detection_1()
        agent.spin()
    except rospy.ROSInterruptException:
        pass