import numpy as np
import cv2

image = cv2.imread("IMG_6941.jpg") 
def RGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: 
        #B = image[y,x,0]
        #G = image[y,x,1]
        #R = image[y,x,2]
        #print("Red: ",R)
        #print("Green: ",G)
        
        #print("Blue: ",B)
        color=np.uint8([[image[y,x]]])
        hsv = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
        rgb = cv2.cvtColor(color,cv2.COLOR_BGR2RGB)
        print("HSV: ",hsv)
        print("RGB: ",rgb)
        print("Cordenadas: X: ",x,"Y: ",y)
        cv2.imshow("RGB",image)

cv2.namedWindow("RGB", cv2.WINDOW_NORMAL) 
cv2.setMouseCallback('RGB',RGB)      
cv2.resizeWindow("RGB", 400, 300)
cv2.imshow("RGB", image) 

cv2.waitKey(0)

cv2.destroyAllWindows()
