#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class stop():
	def __init__(self):
		rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.callback)
		self.cmd_pub = rospy.Publisher('ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)	
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
	def laser_callback(self,msg):
	       #Read from a specific set fo laser shots
	       ranges = msg.ranges
	       #get average
	       self.averagel = 0
	       counter = 0
	       for i in range(520, 560): #ranges set to work for left wall
		   counter += 1
		   self.averagel += ranges[i]
	       self.averagel /= counter
	
	def callback(self,msg):
		print "STOP"
                if self.averagel<0.5:
			msg = AckermannDriveStamped()
			msg.drive.speed = 0.0
			msg.drive.steering_angle = 0.0
			msg.header.stamp = rospy.Time.now()
		self.cmd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("stop", anonymous = True)
    node =stop()
    rospy.spin()
