#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import random
from apriltags_ros.msg import AprilTagDetectionArray
from math import atan2, sqrt, cos, sin, pi, asin, degrees
import numpy as np
import time
from nav_msgs.msg import Odometry


class GoToPose():

    def __init__(self):
        # Start node
        rospy.init_node('slave_comm', anonymous=True)

        # Subscibe to target topic
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)

        # Subscribe to odometry topic
        rospy.Subscriber("tb3_2/odom", Odometry, self.odomCallback)

        # Publish to the comm topic
        self.pub=rospy.Publisher('/tb3_2/comm', Int8, queue_size=10)

        # Publish to cmd_vel topic
        self.vel_pub=rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)

        # Define target tag and publish it to comm
        #self.dest_tag = random.randint(0, 6)
        self.dest_tag = 0
        print("Target tag is", 0)
        self.pub.publish(self.dest_tag)

        # Stores current odometry data
        self.odom = None

        # Master tag location
        self.master_x = None
        self.master_y = None

        # See if the target tag is found, and if so where
        self.target_found = False
        self.target_x = None
        self.target_y = None

        self.local_pos = [0,0,0]
        self.master_id = 7

 
    def tagCallback(self, data):
        vel_update=Twist()        
    
        try:
            tag_id = data.detections[0].id
            x_pos = data.detections[0].pose.pose.position.x
            y_pos = data.detections[0].pose.pose.position.y
            print("Saw tag", tag_id)
            

            # If it is the target tag
            if tag_id == self.dest_tag:
                self.target_found = True
                self.target_x = x_pos
                self.target_y = y_pos
                print("Destination Relative Distance is ", (x_pos, y_pos))
                #Send cmd_vel to Slave
                euclidean_d = sqrt(self.target_x**2+self.target_x**2)
                if euclidean_d > 0.2:
                    vel_update.linear.x=euclidean_d*2
                    vel_update.angular.z= self.angular_vel(self.target_x, self.target_y)
                    self.vel_pub.publish(vel_update)
                    print("Sending X vel and yaw: ", vel_update.linear.x, vel_update.angular.z)
                else:
                    vel_update.linear.x = 0
                    vel_update.angular.z = 0
                    self.vel_pub.publish(vel_update)
            

                
            # Else, if it is the master tag
            elif tag_id == self.master_id:
                self.master_x = x_pos
                self.master_y = y_pos
                print("Master Relative Distance is ", (x_pos, y_pos))
                #Send cmd_vel to Slave
                euclidean_d = sqrt(self.master_x**2+self.master_y**2)-.1
                vel_update.linear.x=euclidean_d*5
                vel_update.angular.z= self.angular_vel(self.master_x, self.master_y)
                print("Sending X vel and yaw: ", vel_update.linear.x, vel_update.angular.z)
                self.vel_pub.publish(vel_update)
                

        except Exception as e:
            print(str(e))
            vel_update.linear.x = 0
            vel_update.angular.z = 0
            self.vel_pub.publish(vel_update)

    def steering_angle(self, goal_x, goal_y):
        #steering angle takes the atan(dy/dx) and atan2 specifically takes the signs of the values into account. this gives the angle of the ray from the turtle to the desired waypoint 
        steering_rad = np.arctan2(goal_y, goal_x)
        print("steering angle is", steering_rad)
        #this keeps theta within a positive 0-pi axis
        # if steering_rad < 0:
        #      return steering_rad + 3.14
        return steering_rad

    def angular_vel(self, goal_x, goal_y, constant=.5):
        #sets angular velocity by finding the delta between the ray of the steering angle and the ray of the current turtle heading/theta
        ccw_steering = self.steering_angle(goal_x, goal_y)
        print("Steering angle", ccw_steering)
        ccw_pose_theta = self.local_pos[2]
        print("Current angle", ccw_pose_theta)
        #below ensures atan2 does not flip from positive to negative values trying to equal the same point in the grid. (ie: if atan2 eq from steering angle func returns -270,
        #we would want 90 instead) this stops a massive delta from occuring which causes the turtle to suddenly have massive angular velocity for 1 timestep and overshoot its correction and do a loop-d-loop


        vel = (ccw_steering)
        print("Difference", vel)
        # if vel > 3.14:
        #     vel = vel - 6.28
        # elif vel < -3.14:
        #     vel = vel + 6.28
        return vel * constant
        #return vel


    # Updates knowledge of robot odometry
    def odomCallback(self, data):
        try:
            x_robo = data.pose.pose.position.x
            y_robo = data.pose.pose.position.y
            q_x = data.pose.pose.orientation.x
            q_y = data.pose.pose.orientation.y
            q_z = data.pose.pose.orientation.z
            q_w = data.pose.pose.orientation.w
            #convert quaternion to euler angle abt robot Z axis
            x_poopoo, y_poopoo, yawangle = self.quaternion_to_euler(q_x,q_y,q_z,q_w)

            self.local_pos = [x_robo, y_robo, yawangle]
            #print("updated local position: ", self.local_pos)
        except Exception as e:
            print(str(e))
            pass

    # Convert a quaternion to euler angles
    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = atan2(t3, t4)

        return X, Y, Z

if __name__ == '__main__':
    node = GoToPose()
    rospy.spin()
    