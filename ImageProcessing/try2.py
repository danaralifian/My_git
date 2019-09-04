import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    #konvert hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #cvCvtColor(imgRGB,imgHSV, CV_BGR2HSV);
    
    #threshold
    retval, threshold = cv2.threshold(frame, 12, 255, cv2.THRESH_BINARY)
    retval12, threshold2 = cv2.threshold(hsv, 12, 255, cv2.THRESH_BINARY)
    #gaus = cv2.adaptiveThreshold(hsv, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
    #retval12,otsu = cv2.threshold(hsv,125,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    

    #hsv h
    lower_red = np.array([24,50,50])
    upper_red = np.array([36,116,255])

    mask = cv2.inRange(frame, lower_red, upper_red)
    res = cv2.bitwise_and(hsv, frame, mask = mask)
    res2 = cv2.bitwise_and(res, frame, mask = mask)
    edges = cv2.Canny(res,100,200)
    #smooth
    kernel = np.ones((15,15),np.float32)/255
    smoothed = cv2.filter2D(edges,-1,kernel)

    #gradient
    laplacian = cv2.Laplacian(frame,cv2.CV_64F)

    median = cv2.medianBlur(threshold,15)
    
    #show
    cv2.imshow('frame', frame)
    cv2.imshow('hsv', hsv)
    cv2.imshow('res', res)
    #cv2.imshow('mask', mask)
    #cv2.imshow('threshold', threshold)
    #cv2.imshow('smoothed', smoothed)
    #cv2.imshow('gaus', gaus)
    #cv2.imshow('otsu', otsu)
    #cv2.imshow('edges', edges)
    #cv2.imshow('median', median)

    if cv2.waitKey(1) & 0xFF == ord('q') :
        break

cv2.destroyAllWindows()
cap.release()
