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
import cv2
from cv_bridge import CvBridge, CvBridgeError

class color_shape_detection:
    #convert the ROS Image message to a CV2 Image
    def __init__(self):
        rospy.init_node('camera_shape_color_cv_2',anonymous=True)
        self.pubz = rospy.Publisher('/robot2/depth', String, queue_size=10)
        self.pubx = rospy.Publisher('/robot2/x', String, queue_size=10)
        rospy.Subscriber("/robot2/image/compressed", CompressedImage, self.image_callback)
        self.pubopencvimg = rospy.Publisher('/robot2/opencv/image/compressed', CompressedImage, queue_size=10 )
        self.pubopencvmask = rospy.Publisher('/robot2/opencv/mask/compressed', CompressedImage, queue_size=10 )
        self.bridge = CvBridge()
        self.pubimgstatus = rospy.Publisher('/robot2/camera_status', String, queue_size=10 )
        # Stereo vision setup parameters
        self.frame_rate = 30    #Camera frame rate (maximum at 120 fps)
        self.B = 6               #Distance between the cameras [cm]
        self.alpha = 33    #Camera field of view in the horisontal plane [degrees] for robot 2
        self.cv_image = np.array([])
        self.flag=0

    def image_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError:
            rospy.loginfo("img_Failed")

    def opencv(self, img):
        try:
            self.frame_left = img[:480,0:640]
            self.frame_right = img[:480,640:]
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
                self.frame_right=cv2.putText(self.frame_right, "NOT FOUND", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                self.frame_left=cv2.putText(self.frame_left, "NOT FOUND", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                self.pubimgstatus.publish("not_found")
            else:
                # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
                # All formulas used to find depth is in video presentaion
                self.x, self.depth = tri.find_depth(circles_right, circles_left, self.frame_right, self.frame_left, self.B, self.alpha)
                if 40<self.depth<70 and 110<radius_right<200 and 110<radius_left<200 and 10<self.x<30:
                    self.frame_right=cv2.putText(self.frame_right, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.frame_left=cv2.putText(self.frame_left, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.frame_right=cv2.putText(self.frame_right, "Distance: " + str(round(self.depth,2)), (300,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.frame_left=cv2.putText(self.frame_left, "Distance: " + str(round(self.depth,2)), (300,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.frame_right=cv2.putText(self.frame_right, "X: " + str(round(self.x,2)), (500,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.frame_left=cv2.putText(self.frame_left, "X: " + str(round(self.x,2)), (500,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                    self.pubx.publish(str(self.x))
                    self.pubz.publish(str(self.depth))
                    self.pubimgstatus.publish("tracking")
                else:
                    self.frame_right=cv2.putText(self.frame_right, "DETECTED: NOT TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                    self.frame_left=cv2.putText(self.frame_left, "DETECTED: NOT TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                    self.pubimgstatus.publish("not_tracking")
                    self.pubx.publish(str(self.x))
                    self.pubz.publish(str(self.depth))

            output = np.concatenate((self.frame_left, self.frame_right), axis=1)
            mask = np.concatenate((self.mask_left, self.mask_right), axis=1)
            # cv2.imshow("output", output)
            self.pubopencvimg.publish(self.bridge.cv2_to_compressed_imgmsg(output))
            self.pubopencvmask.publish(self.bridge.cv2_to_compressed_imgmsg(mask))
            # cv2.waitKey(3)

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
        agent=color_shape_detection()
        agent.spin()
    except rospy.ROSInterruptException:
        pass