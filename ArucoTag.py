import cv2
import time
import numpy
import RPi.GPIO as GPIO
from cv2 import aruco
import keyboard

GPIO.cleanup() #ensure GPIO cleaned from last use
camPort = 0     # defining the camera to use
camHandle = cv2.VideoCapture(camPort)   # defining a camera handle

markerSize = 6  # number of boxes in rows and cols of the tag
totalMarkers = 250  # used for defining the dictionary

pinFriend = 14 # assign pin numbers
pinEnemy = 15 # assign pin numbers

GPIO.setmode(GPIO.BCM) # set gpio pinmode to BCM/Broadcom
GPIO.setup(pinFriend, GPIO.OUT) # set pins to output
GPIO.setup(pinEnemy, GPIO.OUT) # set pins to output

# defining the aruco dictionary
key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
arucoDict = aruco.Dictionary_get(key)

#while(1):
 #   GPIO.output(pinEnemy, GPIO.HIGH)


try:
    while(1):     # main loop
        ret, img = camHandle.read()     # reading frames from video feed
       # cv2.imshow("test",img)   
       # time.sleep(.25)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # converting to grayscale
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict)     # finding tags in image
       # print(type(ids))
       # print(bboxs)
       # print(bboxs[0][0][0])
       # print(bboxs[0][0][1])
        idx = 0
       # print(ids[0])
        if type(ids) == numpy.ndarray:
            size0 = numpy.linalg.norm((bboxs[0][0][0]-bboxs[0][0][1]))
            print(size0)
            if len(ids) > 1:
                size0 = numpy.linalg.norm((bboxs[0][0][0]-bboxs[0][0][1]))
                size1 = numpy.linalg.norm((bboxs[1][0][0]-bboxs[1][0][1]))
               # print(size0)
               # print(ids[0])
               # print(size1)
               # print(ids[1])
               # if size0>size1:
                   # idx = 0
               # else:
                   # idx = 1
            if ids[idx] <= 9 and ids[idx] >= 0 and size0 >= 70:
                print("friend")
                print(ids[idx])
                GPIO.output(pinEnemy, GPIO.LOW)
                GPIO.output(pinFriend, GPIO.HIGH)
                time.sleep(0.25)
            elif ids[idx] >= 10 and size0 >= 70:
                print("enemy")
                print(ids[idx])
                GPIO.output(pinFriend, GPIO.LOW)
                GPIO.output(pinEnemy, GPIO.HIGH)
                time.sleep(0.25)
        else:
            GPIO.output(pinFriend, GPIO.LOW)
            GPIO.output(pinEnemy, GPIO.LOW)
            time.sleep(0.1)
       #print(ids)
       #print(bboxs)
        aruco.drawDetectedMarkers(img, bboxs)   # the detected marker on the image
        cv2.imshow("detected", img)     # showing the image
        cv2.waitKey(1)

except KeyboardInterrupt:
    GPIO.output(pinFriend, GPIO.LOW)
    GPIO.output(pinEnemy, GPIO.LOW)
    camHandle.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()


