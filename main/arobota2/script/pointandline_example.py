#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('line_pub_example')
pub_line_min_dist = rospy.Publisher('line_ezample', Marker, queue_size=1)
rospy.loginfo('Publishing example line')

while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.02
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = -1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = 0.0
    first_line_point.y = 0.0
    first_line_point.z = 0.0
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = 1.0
    second_line_point.y = 0.0
    second_line_point.z = 0.0
    marker.points.append(second_line_point)
    # third point
    third_line_point = Point()
    third_line_point.x = 0.5
    third_line_point.y = 0.5
    third_line_point.z = 0.0
    marker.points.append(third_line_point)
    # forth point
    forth_line_point = Point()
    forth_line_point.x = 0.0
    forth_line_point.y = 0.0
    forth_line_point.z = 0.0
    marker.points.append(forth_line_point)


    # Publish the Marker
    pub_line_min_dist.publish(marker)

    rospy.sleep(0.5)