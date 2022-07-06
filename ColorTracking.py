import cv2
import ColorDetectionModule as cdm 
import ShapeDetectionModule as sdm
import SerialModule as sm 
import numpy as np 
import time

# frameWidth = 640
# frameHeight = 480

camSet = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

cap = cv2.VideoCapture(camSet)
ser = sm.initconnection('/dev/ttyACM0', 115200)
cap.set(10,120)

perrorLR, perrorUD = 0,0

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

def getContours(img,imgContour):
    _, contours, _ = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cx, cy = -1, -1 
    objectsOut = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area","Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            x,y,w,h = cv2.boundingRect(approx)
            #cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),5)
        
            objectsOut.append([[x,y,w,h],w*h])    
            
    objectsOut = sorted(objectsOut, key = lambda x:x[1], reverse = True)
    return objectsOut

def findCenter(imgContour,objects):
    cx, cy = -1, -1 # there is no objects
    w,h = 192,144 #screen size
    if len(objects)!=0:
        x,y,w,h = objects[0][0]
        cx = x+ w//2
        cy = y+ h//2
        cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),5)
        cv2.circle(imgContour,(cx,cy),2,(0,255,0),cv2.FILLED)
        ih, iw, ic = imgContour.shape
        cv2.line(imgContour,(iw//2,cy),(cx,cy),(0,255,0),1)
        cv2.line(imgContour,(cx,ih//2),(cx,cy),(0,255,0),1)
    return cx, cy, w,h,imgContour
            
    

def trackObject(cx,cy,w,h,bh):
    global perrorLR, perrorUD
    print(w,h)
    kLR = [0.6, 0.1]
    kUD= [0.5, 0.1]
    if cx!=-1: # if there is an object
        #Left and Right
        errorLR = w//2 - cx
        posTurn = kLR[0]*errorLR + kLR[1]*(errorLR-perrorLR)
        posTurn = int(np.interp(posTurn,[-w//2,w//2],[100,-100]))
        perrorLR = errorLR
        ##Up and Down
        errorUD = bh
        posSpeed = kUD[0]*errorUD + kUD[1]*(errorUD-perrorUD)
        posSpeed = int(np.interp(posSpeed,[0,h//2],[45,40]))
        perrorUD = errorUD
        print(posSpeed,posTurn)
        sm.sendData(ser,[abs(posSpeed),posTurn],4)
    else:
        sm.sendData(ser,[0,0],4)


def empty(a):
    pass

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",75,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",205,255,empty)
cv2.createTrackbar("Sat Min","TrackBars",64,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",140,255,empty)
cv2.createTrackbar("Val Min","TrackBars",80,232,empty)
cv2.createTrackbar("Val Max","TrackBars",240,255,empty)
cv2.createTrackbar("Area","TrackBars",20000,30000,empty)
cv2.createTrackbar("Threshold1","TrackBars",67,255,empty)
cv2.createTrackbar("Threshold2","TrackBars",118,255,empty)



while True:
    success, img = cap.read()
    img = cv2.resize(img, (0,0), None, 0.3, 0.3) # decrease image size to increase process speed
    imgContour = img.copy()
    imgResult = findColor(img)
    imgBlur = cv2.GaussianBlur(imgResult,(7,7),1)
    imgGray = cv2.cvtColor(imgBlur,cv2.COLOR_BGR2GRAY)
    
    threshold1 = cv2.getTrackbarPos("Threshold1","TrackBars")
    threshold2 = cv2.getTrackbarPos("Threshold2", "TrackBars")
    imgCanny = cv2.Canny(imgBlur,threshold1,threshold2)
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny,kernel,iterations = 1)
    objects = getContours(imgDil,imgContour)
    cx, cy, bw,bh,imgContour = findCenter(imgContour, objects)
    
    h, w, c = imgContour.shape
    #cv2.line(imgContour,(w//2,0),(w//2,h),(255,0,255),1)
    #cv2.line(imgContour,(0,h//2),(w,h//2),(255,0,255),1)

    trackObject(cx,cy,w,h,bh)

    #imgContour = cv2.resize(imgContour, (0,0), None, 3, 3) #resize the image to show the regular image
    cv2.imshow("image", imgContour)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        sm.sendData(ser,[0,0],4)
        break