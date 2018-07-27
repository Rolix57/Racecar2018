#!/usr/bin/env python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Follow_Wall():

    def __init__(self):
        # subscribe to Ackermann
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
 
        # publish to Ackermann
        self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)


        self.kp = 1.2
        self.kd = 0.4
        self.ki = 0.02

        self.velCoeff = 1
        self.futCon = 0

        self.averageL = 0
        self.idealDis = 0.45
        self.error = 0

        self.prop = 0
        self.deriv = 0
        self.integ = 0
        self.prev_error = 0
        self.output = 0

        self.prev_time = 0
        self.current_time = time.time()

    def laser_callback(self,msg):
        
        ranges = msg.ranges
        #Left average
        self.futureL = np.mean(ranges[705: 740])
        self.averageL = np.mean(ranges[740 : 900])

        print("Ideal Distance = {0:.5f}".format(self.idealDis))
        print("Future distance = {0:.5f}".format(self.futureL))
        self.error = self.averageL - self.idealDis
        self.PID(self.error)

    def PID(self, error):
        
        self.current_time = time.time()

        #P
        self.prop = self.kp * error

        #I
        self.integ = self.ki * ((error + self.prev_error) * (self.current_time - self.prev_time))

        #D
        self.deriv = self.kd * ((error - self.prev_error) / (self.current_time - self.prev_time))

        self.prev_error = error
        self.prev_time = self.current_time

        if (self.futureL - self.averageL) >= 0.235:
            self.futCon = - 0.1
        elif (self.futureL - self.averageL) <= 0.185:
            self.futCon = 0.1

        self.output = self.prop + self.integ + self.deriv + self.futCon

        if abs(self.output) >= 0.34:
            self.velCoeff = 0.7
        elif abs(self.output) >= 0.25:
            self.velCoeff = 0.9
        else:
            self.velCoeff = 1

        print("P = {} I = {} D = {}, PID = {}".format(round(self.prop, 4), round(self.integ, 4), round(self.deriv, 4), round(self.output, 4)))
        print("Angle = {}".format(self.output))

        self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = 0.7 * self.velCoeff
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1
        self.cmd_pub.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("Follow_Wall")
    node = Follow_Wall()
rospy.spin()
