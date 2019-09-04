import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import numpy as np
from matplotlib import pyplot as plt
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
frequencyHertz = 50
pwm = GPIO.PWM(11,frequencyHertz)

leftPosition = 0.75
rightPosition = 2.5
middlePosition = (rightPosition-leftPosition)/2 + leftPosition

positionList = [leftPosition, middlePosition, rightPosition,middlePosition]

msPercycle = 1000/frequencyHertz

cap = cv2.VideoCapture(0)

while (cap.isOpened()):
    _,frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    

    for i in range(3):
        for position in positionList:
            dutyCyclePercentage = position*100/msPercycle
            print"Position: "+str(position)
            print"Duty Cycle: "+str(dutyCyclePercentage)+"%"
            print""
            pwm.start(dutyCyclePercentage)
            time.sleep(.5)
    pwm.stop()
    GPIO.cleanup()

    cv2.imshow("hsv",hsv)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()
