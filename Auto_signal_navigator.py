
import cv2
from cvzone.SerialModule import SerialObject
from time import sleep
import numpy as np
import serial #for Serial communication
import time 

arduino = serial.Serial('com10',9600)
time.sleep(2)
webcam = cv2.VideoCapture(0)
while(1):
    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 50, 70], np.uint8)
    red_upper = np.array([9, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red2_lower = np.array([159, 50, 70], np.uint8)
    red2_upper = np.array([180, 255, 255], np.uint8)
    red2_mask = cv2.inRange(hsvFrame, red2_lower, red2_upper)
##    blue_lower = np.array([100, 150, 0], np.uint8)
##    blue_upper = np.array([140, 255, 255], np.uint8)
##    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    yellow_lower = np.array([25, 50, 70], np.uint8)
    yellow_upper = np.array([35, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)
    green_lower = np.array([36, 50, 70], np.uint8)
    green_upper = np.array([89, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
    red2_mask = cv2.dilate(red2_mask, kernal)
    res_red2 = cv2.bitwise_and(imageFrame, imageFrame, mask = red2_mask)
##    blue_mask = cv2.dilate(blue_mask, kernal)
##    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask)
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame, mask = yellow_mask)
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask)
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
         arduino.write(b'2')
        
    
        else:
           arduino.write(b'1')
    contours, hierarchy = cv2.findContours(red2_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
         arduino.write(b'2')
        else:
           arduino.write(b'1')
                
##    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
##	
##    for pic, contour in enumerate(contours):
##        area = cv2.contourArea(contour)
##        if(area > 300):
##         l1.write(1)
##         
##        
##        else:
##           l1.write(0)
           
           
           
    contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
         arduino.write(b'3')
       
        
        else:
           arduino.write(b'1')

    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
         arduino.write(b'1')
        else:
           arduino.write(b'1')
          
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
	    cap.release()
	    cv2.destroyAllWindows()
	    break
