"""
Color Detection Module

2021.6.11
"""

import cv2
import numpy as np

def findColor(img):
    imgColor = img.copy()
    imgHSV = cv2.cvtColor(imgColor,cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
    #print(h_min,h_max,s_min,s_max,v_min,v_max)
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHSV,lower,upper)
    imgResult = cv2.bitwise_and(imgColor,imgColor,mask=mask)
    return imgResult

def empty(a):
    pass

def main():
    img = cv2.imread("../Resources/lambo.PNG")
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars",640,240)
    cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
    cv2.createTrackbar("Hue Max","TrackBars",19,179,empty)
    cv2.createTrackbar("Sat Min","TrackBars",110,255,empty)
    cv2.createTrackbar("Sat Max","TrackBars",240,255,empty)
    cv2.createTrackbar("Val Min","TrackBars",153,255,empty)
    cv2.createTrackbar("Val Max","TrackBars",255,255,empty)
    while True:
        imgColor = findColor(img)
        cv2.imshow("Output", imgColor)
        cv2.waitKey(1)

if __name__ == "__main__": #check whether we are running this script or we are calling this script from the other script
    main()