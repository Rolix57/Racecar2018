#! /usr/bin/env python

import rospy
#from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
import os
import time

class ar_switch():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        #rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback,  queue_size = 1)

    def callback(self,marker):
        if len(marker.markers) > 0:
            if marker.markers[0].id == 3:
                time.sleep(3)
                os.system('go.py')
            if mafker.markers[0].id == 4:
                time.sleep(3)
                os.system('stop.py')
                   

if __name__ == "__main__":
    rospy.init_node("ar_switch", anonymous = True)
    node =ar_switch()
    rospy.spin()
