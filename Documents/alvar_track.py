#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback,  queue_size = 1)

    def callback(self,msg):
        if self.msg.markers.id != 1:
            print ("AR tag" + self.msg.marker.id)   
                              
if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
    rospy.spin()
