#-*- coding: utf-8 -*-
import cv2 #OpenCV 영상처리

#classifier
#faceCascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/fire_detection.xml')
faceCascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/haarcascade_frontalface_default.xml')
#video caputure setting
capture = cv2.VideoCapture(0) # initialize, # is camera number
capture.set(cv2.CAP_PROP_FRAME_WIDTH,1280) #CAP_PROP_FRAME_WIDTH == 3
capture.set(cv2.CAP_PROP_FRAME_HEIGHT,720) #CAP_PROP_FRAME_HEIGHT == 4

#console message
face_id = input('\n enter user id end press <return> ==> ')
print("\n [INFO] Initializing face capture. Look the camera and wait ...")

count = 0 # # of caputre face images
#영상 처리 및 출력
while True: 
    ret, frame = capture.read() #카메라 상태 및 프레임
    #cf. frame = cv2.flip(frame, -1) #상하반전
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #흑백으로
    faces = faceCascade.detectMultiScale(
        gray,#검출하고자 하는 원본이미지
        scaleFactor = 1.2, #검색 윈도우 확대 비율, 1보다 커야 한다
        minNeighbors = 6, #얼굴 사이 최소 간격(픽셀)
        minSize=(20,20) #얼굴 최소 크기. 이것보다 작으면 무시
    )

    #얼굴에 대해 rectangle 출력
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        #inputOutputArray, point1 , 2, colorBGR, thickness)
        count += 1
        cv2.imwrite("dataset/User."+str(face_id)+'.'+str(count)+".jpg",gray[y:y+h, x:x+w])
    cv2.imshow('image',frame)

    #종료 조건
    if cv2.waitKey(1) > 0 : break #키 입력이 있을 때 반복문 종료
    elif count >= 100 : break #100 face sample

print("\n [INFO] Exiting Program and cleanup stuff")

capture.release() #메모리 해제
cv2.destroyAllWindows()#모든 윈도우 창 닫기
