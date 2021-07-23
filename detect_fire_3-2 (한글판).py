#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy, sys, cv2, os, datetime, time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectFire:

  def __init__(self):
    self.sub = rospy.Subscriber("/image_raw", Image, self.callback)
    self.bridge = CvBridge()
    self.cv_msg = cv2.imread("cimg.png")    

  def callback(self,data):
    try:
      self.cv_msg = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    except CvBridgeError as e:
      print(e)

  def save_picture(self, picture):    # 프레임 자동 저장 경로 및 저장 이름 설정
    try:
      img = picture
      now = datetime.datetime.now()
      date = now.strftime('%Y%m%d')   #년,월,일
      hour = now.strftime('%H%M%S')   #시,분,초
      user_id = '00001'               #저장 번호
      filename = '/home/fire_pictures/cvui_{}_{}_{}.png'.format(date, hour, user_id) # 저장 위치 및 저장 형식 지정
      cv2.imwrite(filename, img)       
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':

    rospy.init_node('fire_detector', anonymous=True)
    df = DetectFire() # 클라스에 있는 내용이 df로 들어감
    rospy.sleep(1.0)   # 1초 대기
    
    try:
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/fire_detector/scripts/fire_detection.xml')  # 인식할 표적의 데이처 값
        
        while not rospy.is_shutdown():
        
            frame = df.cv_msg
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)            
                             
            if len(fire) == 0:
                rospy.set_param("/fire_detector/fire_detection_state", False)
                #print(rospy.get_param("/fire_detector/fire_detection_state"))
            
            for (x,y,w,h) in fire:    # 표적에 사각 프레임 생성 ,색상(R,G,B) /선두깨
                cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                rospy.set_param("/fire_detector/fire_detection_state", True)
                
                fx = 0.0 #불의 x좌표의 평균을 나타내는 변수명이다. fire_x의 약어이다.
                fy = 0.0 #불의 y좌표의 평균을 나타내는 변수명이다. fire_y의 약어이다.

                fxs = 0.0 #불의 x좌표의 합계를 나타내는 변수명이다. 자세한 설명은 아래에서 한다.
                fys = 0.0 #불의 y좌표의 합계를 나타내는 변수명이다. 자세한 설명은 아래에서 한다.

                for i in range(50):  # 50번 반복

                    fcx = x + (w * 0.5)   # 가운데를 나누는 값
                    fcy = y + (h * 0.5)

                    fxs = fxs + fcx
                    fys = fys + fcy
                    
                    fx = fxs / (i+1)     # 나누기 2
                    fy = fys / (i+1)

 
                if ((fx >= 190)and(fx <= 450)) and ((fy >= 150)and(fy <= 330)):   # 화면의 프레임값 640x480 의 중간 지점, 가운데 찾기
                    print("fire is center({}, {})".format(fx, fy))
                    print("목표를 포착했습니다.")
                    df.save_picture(frame)

                else:
                    if ((fx >= 0) and (fx <= 640)) and ((fy >= 331) and(fy <= 480)):   # x축의 위에 있을 때
                        print("fire is not center({}, {})".format(fx, fy))
                        print("목표지점 보다 위에 있습니다.")
                    
                if ((fx >= 0) and (fx <= 640)) and ((fy >= 0 ) and(fy <= 149)):        # x축의 아래 있을 때
                      print("fire is not center({}, {})".format(fx, fy))
                      print("목표지점 보다 아래 있습니다.")                    
                                                  
                elif ((fx >= 0) and (fx <= 189)) and ((fy >= 0 ) and(fy <= 480)):       # y축의 좌측에 있을 때
                      print("fire is not center({}, {})".format(fx, fy))
                      print("목표지점 보다 왼쪽에 있습니")

                else:
                    if ((fx >= 451) and (fx <= 640)) and ((fy >= 0 ) and(fy <= 480)):    # y축의 우측에 있을 때
                      print("fire is not center({}, {})".format(fx, fy))
                      print("목표지점 보다 오른쪽에 있습니다")
            
            cv2.imshow('frame', frame)   
            if cv2.waitKey(1) & 0xFF == ord('q'):    
                break
                        
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
