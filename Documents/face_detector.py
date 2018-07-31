#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import time
import matplotlib.pyplot as plt

class FaceNode:
    def __init__( self ):
        rospy.Subscriber("/zed/rgb/image_rect_color",Image, self.callback, queue_size = 1)
        self.bridge=CvBridge()
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        
        # publish to Ackermann
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped, queue_size = 20)

    
        self.kp = -0.35
        self.kd = -.25
        self.ki = .0

        '''
        self.kp = 1.5
        self.kd = 0.8
        self.ki = 0.02
        chidas:
        1.2
        0.4
        0.02
        '''

        self.average = 0
        self.av = 0
        self.idealDis = 0.5
        self.error = 0

        self.prop = 0
        self.deriv = 0
        self.integ = 0
        self.prev_error = 0
        self.output = 0

        self.prev_time = 0
        self.current_time = 0

        
    def callback(self,msg):
        self.msg=msg
        frame = self.bridge.imgmsg_to_cv2(msg)
        img_copy = np.copy(frame)
        self.gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier("data/haarcascade_frontalface_alt.xml")
        faces = face_cascade.detectMultiScale(gray_img, scaleFactor=1.3, minNeighbors=5)
        
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
             if cx<640:
                 self.cx = 1280-(x +w/2)
        print('Faces found: ', len(faces))
        
        cv2.imshow("mask",self.gray)
        cv2.waitKey(1)
   
 
        self.error=(float(self.cx)-640)/(640)
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

        self.output = self.prop + self.integ + self.deriv
	#if abs(self.output) > 0.3:
		#self.output = 0.3
        #if self.cx>640:
            #self.output*=-1
        #print self.output

        print("P = {} I = {} D = {}, PID = {}".format(round(self.prop, 4), round(self.integ, 4), round(self.deriv, 4), round(self.output, 4)))

        self.ackermann_cmd_input_callback(AckermannDriveStamped())
	print("cx = {}		cy = {}".format(self.cx, self.cy))
	print("cols = {}		rows = {}".format(self.cols, self.rows))

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = 0.7
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1.42
        self.cmd_pub.publish(msg)

              
if __name__ == "__main__":
    rospy.init_node("Yellow_Node", anonymous = True)
    node =YellowNode()
    rospy.spin()
