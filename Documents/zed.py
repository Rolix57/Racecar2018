#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge


class YellowNode:
  def __init__( self ):
    rospy.Subscriber("/camera/zed/rgb/image_rect_color",Image, self.callback, queue_size = 1)
    self.bridge=CvBridge()
    rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
   
    # publish to Ackermann
    self.cmd_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

    self.Kp = 2.5
    self.Kd = 1.3
    self.Ki = .01
    self.prev_error = 0
    self.sum_error = 0
    self.prev_error_deriv = 0
    self.curr_error_deriv = 0
    self.control = 0
    self.dt = 0.2
    self.last_integral = 0
    self.desiredd =240
  def callback(self,msg):
    self.msg=msg
    frame = self.bridge.imgmsg_to_cv2(msg)
    
    
    self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   
    lower_yellow = np.array([20,200,200])
    upper_yellow = np.array([40,255,255])
    self.mask=cv2.inRange(self.hsv,lower_yellow,upper_yellow)
    rows=self.mask.shape[0]
    cols=self.mask.shape[1]
    area1=np.array([[[cols,0], [cols/2,0], [cols,rows],[cols,rows-1]]],dtype=np.int32)
    area2=np.array([[[cols/2,0], [0,0], [0,rows],[1,rows]]],dtype=np.int32)
    cv2.fillPoly(self.mask,area1,0)
    cv2.fillPoly(self.mask,area2,0)
    cv2.imshow('mask',self.mask)
     
    contours,hierachy=cv2.findContours(self.mask.copy(), 1, cv2.CHAIN_APPROX_NONE)    

    if len(contours) > 0:
      c = max(contours, key=cv2.contourArea)
      M = cv2.moments(c)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      #cv2.line(self.mask,(cx,0),(cx,720),(255,0,0),1)
      #cv2.line(self.mask,(0,cy),(1280,cy),(255,0,0),1)
      cv2.drawContours(self.mask, contours, -1, (0,255,0), 1)
      print cx-self.desiredd
      self.PID(cx-self.desiredd)
    #if cx >= 120:
        #print "Turn Left!"
      #if cx < 120 and cx > 50:
        #print "On Track!"
      #if cx <= 50:
        #print "Turn Right"
    #else:
      #print "I don't see the line"
    #print self.desiredd-cx

    cv2.waitKey(0)
    #cv2.bitwise_and(mask,x, mask=mask)
    #threshold=[]
    #width=len(self.mask[0])
    #height=len(self.mask)
    
    #self.region=cv2.bitwise_and(mask,y)
    
    #print self.msg.header
    

  def PID(self, current_error, reset_prev=False):
    integral_gain = self.last_integral
    proportional_gain = self.Kp*current_error
    integral_gain += self.Ki * current_error *self.dt
    derivative_gain = self.Kd * (current_error - self.prev_error) / self.dt

    #update value
    self.control = proportional_gain + integral_gain + derivative_gain
    self.prev_error = current_error
    self.last_integral = integral_gain

    self.ackermann_cmd_input_callback(AckermannDriveStamped())
    return self.control

  def ackermann_cmd_input_callback(self, msg):
    msg.drive.speed = 1.0
    msg.drive.steering_angle = self.control
    msg.drive.steering_angle_velocity = 1
    self.cmd_pub.publish(msg)
                    

              
if __name__ == "__main__":
    rospy.init_node("Yellow_Node", anonymous = True)
    node =YellowNode()
rospy.spin()
