#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import time


class threetas:

    def __init__(self):

        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)


    def drive_callback(self, drive_msg):
        drive_msg.drive.speed = 0.3
        self.drive_pub.publish(drive_msg)
        time.sleep(1)
        drive_msg.drive.speed = -0.3
        self.drive_pub.publish(drive_msg)
        time.sleep(1)
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)




if __name__ == "__main__":
    rospy.init_node("Three_Ta's")
    node = threetas()
    rospy.spin()
