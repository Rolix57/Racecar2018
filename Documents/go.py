#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class go():
	def __init__(self):
		rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.callback)
		self.cmd_pub = rospy.Publisher('ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 10)	
		
	def callback(self,msg):
		print "GO"
		msg = AckermannDriveStamped()
		msg.drive.speed = 0.7
		msg.drive.steering_angle = 0.0
		msg.header.stamp = rospy.Time.now()
		self.cmd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("go", anonymous = True)
    node =go()
    rospy.spin()
