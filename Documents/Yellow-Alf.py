import cv2
import numpy as np

video=cv2.VideoCapture(0)

while True:
    check,frame=video.read()
    #yellow filter
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20,100,120])
    upper_yellow = np.array([50,255,255])
    mask=cv2.inRange(hsv,lower_yellow,upper_yellow)
    cv2.imshow("mask",mask)    
    #Canny
    canny=cv2.Canny(mask, 100, 100)
    cv2.imshow("Canny",canny)
    #ROI
    rows=mask.shape[0]
    cols=mask.shape[1]
    area1=np.array([[[cols,0], [cols/2,0], [cols,rows],[cols,rows-1]]],dtype=np.int32)
    area2=np.array([[[cols/2,0], [0,0], [0,rows],[1,rows]]],dtype=np.int32)
    x=cv2.fillPoly(mask,area1,0)
    y=cv2.fillPoly(mask,area2,0)
    cv2.bitwise_and(mask,x)
    region=cv2.bitwise_and(x,y)
    #show region and video
    cv2.imshow("region",region)
    cv2.imshow("video",frame)
    cv2.waitKey(1)              

video.release()
cv2.destroyAllWindows()
