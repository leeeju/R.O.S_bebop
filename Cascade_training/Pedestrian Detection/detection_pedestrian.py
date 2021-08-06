# -*- coding: utf-8 -*-
import cv2
print('사람을 학습시킨 .xml 파일')

video_src = '/fire/Pedestrian Detection/pedestrians.avi'
cap = cv2.VideoCapture(video_src)
bike_cascade = cv2.CascadeClassifier('/fire/Pedestrian Detection/pedestrian.xml')

while True:
    ret, img = cap.read()
	   
    if (type(img) == type(None)):
        break
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bike = bike_cascade.detectMultiScale(gray,1.3,2)

    for(a,b,c,d) in bike:
        cv2.rectangle(img,(a,b),(a+c,b+d),(0,255,210),4)
    
    cv2.imshow('video', img)
    
    if cv2.waitKey(33) == 27:
        break

cv2.destroyAllWindows()
