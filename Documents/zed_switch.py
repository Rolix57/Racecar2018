#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import time

class YellowNode:
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
        
        global self.maskcontour  
        
    def callback(self,msg):
        self.msg=msg
        frame = self.bridge.imgmsg_to_cv2(msg)
        rows=frame.shape[0]
        cols=frame.shape[1]
        cropped = frame[frame.shape[1]*1/2:frame.shape[0], :, :]
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
        
        lower_orange = np.array([10,130,220])
        upper_orange = np.array([25,255,255])
        lower_green = np.array([10,130,220])
        upper_green = np.array([25,255,255])
        lower_blue = np.array([10,130,220])
        upper_blue = np.array([25,255,255])
        lower_red = np.array([10,130,220])
        upper_red = np.array([25,255,255])
        
        self.maskorange=cv2.inRange(self.hsv,lower_yellow,upper_yellow)
        self.maskgreen=cv2.inRange(self.hsv,lower_green ,upper_green )
        self.maskblue = cv2.inRange(self.hsv, lower_blue, upper_blue)
        self.maskred = cv2.inRange(self.hsv, lower_red, upper_red)
        
        
        #area1=np.array([[[cols,0], [cols/2,0], [cols,rows],[cols,rows-1]]],dtype=np.int32)
        #area2=np.array([[[cols/2,0], [0,0], [0,rows],[1,rows]]],dtype=np.int32)
        #cv2.fillPoly(self.mask,area1,0)
        #cv2.fillPoly(self.mask,area2,0)
        
        threshold = []
        counterorange = 0
        countegreen = 0
        counterblue = 0
        counterred = 0
        
        for x in range(rows):
            for y range(cols): 
                if maskorange[x][y] != 0:
                    threshold.append(255)
                    counterorange+= 1
                else:
                    threshold.append(0)

        for x in range(rows):
            for y range(cols): 
                if maskgreen[x][y] != 0:
                    threshold.append(255)
                    countergreen+= 1
                else:
                    threshold.append(0)
        for x in range(rows):
            for y range(cols): 
                if maskblue[x][y] != 0:
                    threshold.append(255)
                    counterblue+= 1
                else:
                    threshold.append(0)
        for x in range(rows):
            for y range(cols): 
                if maskred[x][y] != 0:
                    threshold.append(255)
                    counterred+= 1
                else:
                    threshold.append(0)  
                    
          
        if counterorange >= 15:
            self.mask_contour=self.maskorange
            print("sigue la naranja")
        elif countergreen >= 15:
            self.mask_contour=self.maskgreen
            print("sigue la verde")
        elif counterblue >= 15:
            self.mask_contour=self.maskblue
            print("sigue la azul")
        elif counterred >= 15:
            self.mask_contour=self.maskred
            print("sigue la roja")
        else:
            print("alto")
        
        contours,hierachy=cv2.findContours(self.mask_contour.copy(), 1, cv2.CHAIN_APPROX_NONE)    
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            self.cx = int(M['m10']/M['m00'])
            self.cy = int(M['m01']/M['m00'])
            cv2.line(self.mask_contour,(self.cx,0),(self.cx,720),(255,0,0),0)
            cv2.line(self.mask_contour,(0,self.cy),(1280,self.cy),(255,0,0),0)
            
        elif ZeroDivisionError:
            self.cx=cols/2
            self.cy=rows/2
     
        cv2.imshow("mask",self.mask)
        cv2.waitKey(1)
        #cv2.bitwise_and(mask,x, mask=mask)
      #threshold=[]
    #width=len(self.mask[0])
    #height=len(self.mask)
    
    #self.region=cv2.bitwise_and(mask,y)
      
    #print self.msg.header
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

        #print("P = {} I = {} D = {}, PID = {}".format(round(self.prop, 4), round(self.integ, 4), round(self.deriv, 4), round(self.output, 4)))

        self.ackermann_cmd_input_callback(AckermannDriveStamped())

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = 0.7
        msg.drive.steering_angle = self.output
        msg.drive.steering_angle_velocity = 1.42
        self.cmd_pub.publish(msg)

              
if __name__ == "__main__":
    rospy.init_node("Yellow_Node", anonymous = True)
    node =YellowNode()
    rospy.spin()
