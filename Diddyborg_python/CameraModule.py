import cv2
import numpy as np
import time
import calendar
import csv
from time import sleep
import datetime

#current date for file name
dt = str(datetime.datetime.now())

#video capture object of openCV
cap = cv2.VideoCapture(0)

#font
font = cv2.FONT_HERSHEY_COMPLEX

#each row of the recorded
# timestamped_camera = np.zeros(6) #time_stamp, id_object, distance, angle

canny_threshold_1 = 30
canny_threshold_2 = 150
area_threshold = 200

# focal_length = 3.60 #mm <-- this is camera module v1
focal_length = 3.04 #mm <-- this is camera module v2

epsilon_multiplier = 0.1#0.02


low_H = 0   #black
low_S = 0
low_V = 70
high_H = 255
high_S = 255
high_V = 255
lower_red = np.array([low_H, low_S, low_V])
upper_red = np.array([high_H, high_S, high_V])
kernel = np.ones((5, 5), np.uint8)
#output-format : time-stamp , no-of-object-edges, c-x, c-y, w, h <-- for every shape detected

while True:
    _, frame = cap.read()
    # #put on gray scale
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # #add Gaussian blurr 
    # blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    # #put canny edge detection with max gradient canny_threshold_2
    # edged = cv2.Canny(blurred, canny_threshold_1, canny_threshold_2)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    edged = cv2.erode(mask, kernel)

    # Contours detection
    contours, _ = cv2.findContours(edged, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    timestamp= time.time()

    for cnt in contours:
        c_area = cv2.contourArea(cnt)
        
        # we consider contour if it has area above 200 pixel*pixel to eliminate small detections
        if c_area>area_threshold:
            #get the center of the contour
            M = cv2.moments(cnt)
            c_x = int(M['m10']/M['m00'])
            c_y = int(M['m01']/M['m00'])

            #approximate contour with polyon : Douglas-Peucker Algorithm
            epsilon = epsilon_multiplier*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            edges_count = len(approx)

            #bound contour with rectangle to measure width and height
            x,y,w,h = cv2.boundingRect(cnt)

            #comment these after checking
            #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            print('an object with {0} edges is found, with center at ({1},{2}) and boxed width,height is ({3},{4})'.format(edges_count,c_x,c_y,w,h))


    cv2.imshow("edged", edged)

    if cv2.waitKey(0) == 0:
        break
    







