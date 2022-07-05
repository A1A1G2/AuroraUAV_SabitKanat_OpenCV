#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  5 22:00:12 2022

@author: aag
"""

import cv2
import numpy as np
 
cap = cv2.VideoCapture(0)
_, frame = cap.read()
(h, w) = frame.shape[:2]
while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #hsv renk uzayında kırmızı renkleri ikiye ayrılır
    #birinci kısım
    lower_red = np.array([0,150,100])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    #ikinci kısım
    lower_red = np.array([170,150,100])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    
    #ikisini mask ta birleştiriyoruz
    mask = mask0 + mask1;
    
    #ortalamasını alıyoruz
    M = cv2.moments(mask)
    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
    else:
    #eğer filtrede hiç renk yoksa x ve y 0
        x, y = 0, 0

    
    cv2.circle(frame, (x, y), 5, (255, 125, 125), -1)
    frame = cv2.line(frame, (x, y), (w//2,h//2), (0, 255, 0), 3)
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