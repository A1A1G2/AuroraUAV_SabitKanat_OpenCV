#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  5 21:24:11 2022

@author: aag
"""

import cv2
import numpy as np
 
cap = cv2.VideoCapture(0)
 
while(1):
    _, frame = cap.read()
    # It converts the BGR color space of image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     
    # Threshold of blue in HSV space
    
    lower_red = np.array([0,150,150])
    upper_red = np.array([5,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    
    lower_red = np.array([170,180,150])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
 
    # preparing the mask to overlay
    mask = mask0 + mask1;
     
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)
 
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('result', result)
     
    if cv2.waitKey(30) == ord('q'):
        break 
 
cv2.destroyAllWindows()
cap.release()

