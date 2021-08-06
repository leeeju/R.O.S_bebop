# -*- coding: utf-8 -*-
import cv2

cascade_src = 'C:/fire/Bike Detection/two_wheeler.xml'
video_src = 'C:/fire/Bike Detection/two_wheeler2.mp4'
#파일 경로[절대경로]를 입력해 주세요.

cap = cv2.VideoCapture(video_src)
car_cascade = cv2.CascadeClassifier(cascade_src)

while True:
    ret, img = cap.read()
    
    if (type(img) == type(None)):
        break
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    cars = car_cascade.detectMultiScale(gray,1.19, 1)


    for (x,y,w,h) in cars:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,215),2)
    
    cv2.imshow('video', img)
    
    if cv2.waitKey(33) == 27:
        break

cv2.destroyAllWindows()
