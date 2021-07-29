#-*- coding: utf-8 -*-
import cv2 as cv
import numpy as np


def main():
    
    #cap = cv.VideoCapture('test.mp4')  #비디오로 영상 뽑을 떄    
    cap = cv.VideoCapture(0)            # 웹켐으로 영상 뽑을 때
    bgs = cv.createBackgroundSubtractorKNN(dist2Threshold =500,detectShadows=False)
    
    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            break
        
        frame = cv.resize(frame,None,fx=1.0,fy=1.0,interpolation = cv.INTER_CUBIC)
        fgmask = bgs.apply(frame)
    
        cv.imshow('video', frame)
        cv.imshow('moving', fgmask)
        
        if cv.waitKey(1) == ord('q'):
            break
        
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
