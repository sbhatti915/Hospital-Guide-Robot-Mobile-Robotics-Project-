#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8
import random
from nav_msgs.msg import Odometry
import yaml
from math import sqrt
import time

class GoToPose():

    def __init__(self):
        # Start node
        rospy.init_node('go_to_goal', anonymous=True)

        # Fetch tag data from yaml
        with open(r'/home/ubuntu/catkin_ws/src/guide/config/hospital_tags.yaml') as stream:
            self.tag_data = yaml.safe_load(stream)

        # Subscibe to target topic
        rospy.Subscriber('/tb3_2/comm', Int8, self.targetCallback)

        # Configure publishing to move action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Queue for storing upcoming targets
        self.tag_queue = []

        # Stores current target ID
        self.target = None

        # Home base xy coordinates
        self.base_x = 0
        self.base_y = 0

    # Adds tag to the tag queue
    def targetCallback(self, data):
        self.tag_queue.append(data)
        print(self.tag_queue)

    # Navigates the robot to the location of the target tag
    def navToTarget(self, target):
        print("Heading to target number", target)

        # Establish goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(self.tag_data[target]['pos'][0], self.tag_data[target]['pos'][1], 0.0), Quaternion(0.0, 0.0, 1.0, 1.0))

        # Publish the move_base goal
        self.move_base.send_goal(goal)

        result = False
        while not result:
            success = self.move_base.wait_for_result(rospy.Duration(120)) 
            state = self.move_base.get_state()
            if success and state == GoalStatus.SUCCEEDED:
            # We made it!
                result = True
            time.sleep(1)
        

    # Navigates the robot back to the base location
    def navToBase(self):
        print("Heading back to base")

        # Establish goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(self.base_x, self.base_y, 0.0), Quaternion(0.0, 0.0, 1.0, 1.0))

        # Publish the move_base goal
        self.move_base.send_goal(goal)

        result = False
        while not result:
            success = self.move_base.wait_for_result(rospy.Duration(120)) 
            state = self.move_base.get_state()
            if success and state == GoalStatus.SUCCEEDED:
            # We made it!
                result = True
            time.sleep(1)

    # Core proceedure, idles until a target is in the queue, then hops between targets until
    # the queue is empty--then it will return to the base station
    def goto(self):
        while not rospy.is_shutdown():

            # Idle until a target is added to the queue
            if len(self.tag_queue):
                # Navigate to the target goal
                self.navToTarget(self.tag_queue.pop(0).data)

                print("Reached Target")

                if not len(self.tag_queue):
                    self.navToBase()
                    print("Reached home base")

if __name__ == '__main__':
    node = GoToPose()
    node.goto()
    