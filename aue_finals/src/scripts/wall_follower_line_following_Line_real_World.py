#! /usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetectionArray
import math
import time
from signal import pthread_sigmask

class LineFollower(object):

    def __init__(self,pub):
        self.pub = pub
        self.bridge_object = CvBridge()   
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.twist_object = Twist()
        self.scan=rospy.Subscriber('/scan',LaserScan,self.laser_callback)
        self.image_sub = rospy.Subscriber("/camera/image_rae",Image,self.camera_callback)
        self.flag = 0
        self.kp = 0.3
        self.stop=rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.stop_sign_callback)
        self.count = 0
        self.i = 1
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.april_tag_callback)
        self.apriltagcondition = 0

    def stop_sign_callback(self,msg):
            for box in msg.bounding_boxes:
                class_stop = box.Class
                if class_stop == 'stop sign':
                    self.count = 1
                    self.i = 0
    
    def april_tag_callback(self,data):

        if data.detections:
            print('April Tag Found')
            self.direction = data.detections[0].pose.pose.pose.position.x
            self.distance = data.detections[0].pose.pose.pose.position.z
            self.apriltagcondition = 1

    def laser_callback(self,msg):

        FL = 0.17
        FA = -0.07
        FA2 = -0.45

        Array = list(msg.ranges)

        self.Array = msg.ranges

        for i in range(len(Array)):
            if Array[i]==0:
                Array[i]=3.3

        Left = ((min(i for i in Array[320:335] if i>0)))
        Forward = min((min(i for i in Array[0:25] if i>0)),(min(i for i in Array[335:360] if i>0)))
        Right = ((min(i for i in Array[25:90] if i>0)))   
        
        if self.flag == 0:
            
            if Forward == 0:
                Forward = 3.3
            if Right == 0:
                Right = 3.3
            if Left == 0:
                Left = 3.3

            if Forward > 1.3:
                self.twist_object.linear.x = Forward * FL
                self.twist_object.angular.z= FA*(Left - Right)/Forward
                self.pub.publish(self.twist_object)

            else:
                self.twist_object.linear.x = Forward * FL
                self.twist_object.angular.z= FA2 * (Left - Right)/Forward
                self.pub.publish(self.twist_object)
        
        self.pub.publish(self.twist_object)
    

    def camera_callback(self, data):

        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+200):int((height/2)+240)][1:int(width)]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, True)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################

        if (cx != 240 and cy != 320 and self.apriltagcondition == 1) or (cx != 240 and cy != 320 and self.apriltagcondition == 0):
            # print('YELLOW Line DETECTED ')
            self.flag = 1
            blob = -1*(int(cx)-282)/322 #calculating where is blob with respect to my center of the camera
            self.twist_object.angular.z = blob*self.kp*1.5 #making turn proportionate to the blob offset
            self.twist_object.angular.z = round(self.twist_object.angular.z,2)
            self.pub.publish(self.twist_object)
            if self.twist_object.angular.z == 0 and self.count == 0:
                self.twist_object.linear.x = 0.1
                # print('throttlecommand')
            elif self.twist_object.angular.z != 0 and self.count == 0:
                self.twist_object.linear.x = 0.03/abs(self.twist_object.angular.z)
                self.twist_object.linear.x = min(0.1, self.twist_object.linear.x)
                self.pub.publish(self.twist_object)
            elif self.count == 1:
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0
                self.pub.publish(self.twist_object)
                time.sleep(3)
                self.count = 0
                self.i = 0
                print('stop sign changed')
        elif cx == 240 and cy == 320 and self.apriltagcondition == 1:
            print(self.distance)
            if self.distance > 0.04:
                self.twist_object.angular.z = self.direction * -15
                self.twist_object.linear.x = self.distance*2
                self.pub.publish(self.twist_object)
            else:
                self.twist_object.linear.x = 0
                self.twist_object.angular.z = 0

            self.pub.publish(self.twist_object)
        elif cx == 240 and cy == 320 and self.apriltagcondition == 0:
            self.flag = 0
        self.pub.publish(self.twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('line_following_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5)
    line_follower_object = LineFollower(pub)    
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()
        
if __name__ == '__main__':
    main()