#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(msg):
	if msg.markers.id = 1:
		#follow left wall
	else if msg.markers.id = 2:
		#follow right wall
	else if msg.markers.id = 3:
		#follow closest wall
	else if msg.markers.id = 4:
		#follow line
	else:
		pass

rospy.init_node('read_ar_tag')
sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rospy.spin()