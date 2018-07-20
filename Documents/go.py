#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class stop():
	def __init__(self):
		rospy.Subscriber('/scan', LaserScan, callback)
		rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)		

	def callback(msg):
		print "GO"
		msg_a = AckermannDriveStamped()
		msg_a.drive.speed = 0.7
		msg_a.drive.steering_angle = 0.0
		msg_a.header.stamp = rospy.Time.now()
		pub.publish(msg_a)


if __name__ == "__main__":
    rospy.init_node("stop", anonymous = True)
    node =stop()
    rospy.spin()
