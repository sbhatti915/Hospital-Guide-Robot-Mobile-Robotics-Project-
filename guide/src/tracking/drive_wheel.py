#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time

# Callback function called whenever
# x-y coordinate received
def drive_callback(data):
    global pub_vel
    global vel
    ball_x 	= data.x
    distMetric 	= data.y
    width  	= data.z

    # Create Twist() instance
    vel = Twist()

    print("dist metric", distMetric)
    if distMetric == 1234.0:
        vel.linear.x = 0
    elif distMetric < 325:
        vel.linear.x = .001*(325-distMetric)
    else:
        vel.linear.x = 0
    #vel.linear.x = .01*(325-distMetric)

    # 
    if ball_x < 0:
        vel.angular.z = 0
    else:
        # Determine center-x, normalized deviation from center
        mid_x  	= int(width/2)
        delta_x	= ball_x - mid_x
        norm_x 	= delta_x/width

        if norm_x > 0.15:
            print ("delX: {:.3f}. Turn right".format(norm_x))
            vel.angular.z = -0.5
            vel.linear.x = 1
        elif norm_x <= -0.15:
            print ("delX: {:.3f}. Turn left".format(norm_x))
            vel.angular.z = 0.5
            vel.linear.x = 1
        if abs(norm_x) < 0.15:
            print ("delX: {:.3f}. Stay in center".format(norm_x))
            vel.angular.z = 0
        # publish vel on the publisher
        pub_vel.publish(vel)

if __name__ == '__main__':
    global vel, pub_vel

    # intialize the node
    rospy.init_node('drive_wheel', anonymous=True)

    # subscribe to /ball_location topic to receive coordinates
    img_sub = rospy.Subscriber("/ball_location",Point, drive_callback)

    # publish to /cmd_vel topic the angular-z velocity change

    pub_vel = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=5)
    rospy.spin()