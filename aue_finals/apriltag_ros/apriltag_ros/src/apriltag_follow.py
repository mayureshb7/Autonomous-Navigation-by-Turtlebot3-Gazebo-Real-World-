#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray

rospy.init_node('turtlebot3', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

def callback(msg):
    try:
        x = msg.detections[0].pose.pose.pose.position.x
        z = msg.detections[0].pose.pose.pose.position.z
        vel_msg.linear.x = z*20
        vel_msg.angular.z = x*-30
        pub.publish(vel_msg)

    except:
        rospy.loginfo('Tag not found')
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.1
        pub.publish(vel_msg)



sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
rospy.spin()
