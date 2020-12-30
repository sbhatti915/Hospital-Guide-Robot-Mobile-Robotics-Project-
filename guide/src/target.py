#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int8

rospy.init_node('target', anonymous=True)
pub=rospy.Publisher('/target', Int8, queue_size=10)

 # Publish new target
def target():
    while True: 
        current_target = input("Enter Desired April Tag: ")
        if current_target == 'quit':
            return
        pub.publish(current_target)           

if __name__ == "__main__":
    target()
