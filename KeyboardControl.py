import cv2
import time
import SerialModule as sm 

camSet = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
cap = cv2.VideoCapture(camSet)

ser = sm.initconnection('/dev/ttyACM0',115200)

while True:
    success, img = cap.read()
    cv2.imshow("result", img)
    key = cv2.waitKey(1)

    if key == ord('w'): sm.sendData(ser,[45,15],4)
    elif key == ord('s'): sm.sendData(ser,[-45,15],4)
    elif key == ord('a'): sm.sendData(ser,[57,-90],4)
    elif key == ord('d'): sm.sendData(ser,[50,40],4)
    elif key == ord('q'): break
    else: sm.sendData(ser,[0,0],4)
        
