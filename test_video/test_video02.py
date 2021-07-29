#-*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
import ipywidgets as widgets
from PyQt5.QtWidgets import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

cap = cv.VideoCapture('test.mp4')
 
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.01,
                       minDistance = 30,
                       blockSize = 14)
 
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 0,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
 
# Create some random colors
color = np.random.randint(0,255,(100,3))
 
# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
 
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
 
wImg1 = widgets.Image(layout = widgets.Layout(border="solid"), width="40%") 
wImg2 = widgets.Image(layout = widgets.Layout(border="solid"), width="40%") 

items = [wImg1, wImg2]
box = Box(children=items)
display.display(box)

while(True):
    try:
        ret,frame = cap.read()
 
        if not ret:
            break
 
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
 
        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]
 
        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)
         
        wImg1.value = cv.imencode(".jpeg", frame)[1].tostring()
        wImg2.value = cv.imencode(".jpeg", img)[1].tostring()
        
 
        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)
 
    except KeyboardInterrupt:
        break
        
if cap.isOpened():
    cap.release()
