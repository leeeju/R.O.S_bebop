#-*- coding: utf-8 -*-
import cv2
import numpy as np
def trackking():
    try:
        print('cam_on')
        cap = cv2.VideoCapture(0)
    except:
        print('cam_off')
        return
    while True:
        ret, frame = cap.read()
        #BGR을 HSV 로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #HSV에서 BGR 정의할 범위값   [RGB == R:레드, G: 그린,  B:블루]
        lower_blue = np.array([110, 100, 100])
        upper_blue = np.array([130, 255, 255])  #파랑색

        lower_green = np.array([50, 100, 100])  # 초록색
        upper_green = np.array([70, 255, 255])

        lower_red = np.array([-10, 100, 100])    #빨강색
        upper_red = np.array([10, 255, 255])
        #HSV 이미지에서 빨,녹,청 색만 추출
        maks_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        maks_green = cv2.inRange(hsv, lower_green, upper_green)
        maks_red = cv2.inRange(hsv, lower_red, upper_red)
        #이미지 비트 연산
        rec1 = cv2.bitwise_and(frame, frame, mask=maks_blue)
        rec2 = cv2.bitwise_and(frame, frame, mask=maks_green)
        rec3 = cv2.bitwise_and(frame, frame, mask=maks_red)
        #화면 출력
        cv2.imshow('original', frame)
        cv2.imshow('BLUE', rec1)
        cv2.imshow('GREEN', rec2)
        cv2.imshow('RED', rec3)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()

trackking()
