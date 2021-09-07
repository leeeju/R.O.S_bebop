#!/usr/bin/env python
#-*- coding: utf-8 -*-

#불을 발견하고 이에 따라 비행을 제어하는 코드를 아래와 같다.

import rospy, cv2, datetime, time, subprocess, rosnode
#import serial
from std_msgs.msg import String
from bb2_pkg.MoveBB2_2 import MoveBB2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np


class DetectFire:

  def __init__(self):
    self.sub = rospy.Subscriber("image_raw", Image, self.callback)
    #노트북의 캠을 쓰는 경우, 즉, rosrun uvc_camera uvc_camera_node를 실행한 경우
    #self.sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)      #드론 비밥의 이미지를 구독할 경우
    
    self.bridge = CvBridge()
    self.cv_msg = cv2.imread("cimg.png")

  def callback(self,data):
    try:
      self.cv_msg = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    except CvBridgeError as e:
      print(e)

  def save_picture(self, picture):
    try:
      img = picture
      now = datetime.datetime.now() #datetime.strftime(format)은 명시적인 포맷 문자열에 의해 제어되는 날짜와 시간을 나타내는 문자열을 반환한다.
      date = now.strftime('%Y%m%d')
      hour = now.utcnow().strftime('%H%M%S%f')
      filename = '/home/kicker/fire_img/fire_{}_{}.png'.format(date, hour)
      cv2.imwrite(filename, img)
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':

    rospy.init_node('fire_detector', anonymous=False)
    fly = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
    #sp = serial.Serial('/dev/ttyUSB0', 9600)
    df = DetectFire()
    tw = Twist()
    rospy.sleep(1.0)

    #사진을 찍는 코드의 실행 횟수를 저장하는 인덱스 변수를 아래와 같이 설정한다.
    j = 0

    try:
        #fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/bb2_pkg/scripts/fire_detection.xml') 
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/bb2_pkg/scripts/cascade014.xml')
        
        while not rospy.is_shutdown():
           cod1 = rospy.get_param("/fire_detectorl/param_of_detector")
           if cod1 == True:
            #print("화재 탐지를 시작합니다.") 여기서 이 출력문을 넣으면, 반복문이 돌 때 계~속 출력함.
            frame = df.cv_msg
            #gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)

            #'''
            # hsv 스케일 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
            lower_red = np.array([-10, 100, 100])    #빨강색
            upper_red = np.array([10, 255, 255])
            maks_red = cv2.inRange(hsv, lower_red, upper_red)
            rec3 = cv2.bitwise_and(frame, hsv, mask = maks_red)
            
            # 윤곽선 추출
            img_gray = cv2.cvtColor(rec3, cv2.COLOR_BGR2GRAY)
            ret, img_binary = cv2.threshold(img_gray, 127, 255, 0)
            contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            #'''
            
            #이는 불을 잠깐 인지했으나, 화면의 중앙에 맞추는 가운데 그 개체가 사라진 경우를 처리할 때 사용한다. 만일, 최초로 불을 발견한 시점과 불을 인지하지 못한 시점이 어느 한계를 넘어간다면 다시 Flytarget을 실행한다.         
            if len(fire) == 0:
              #불이 발견되지 않은 경우에 "/play_alarm/fire_detection_state"를 0으로 설정한다. /play_alarm/fire_detection_state는 화재가 발생했음을 소리나 SMS 등으로 알리는 코드를 실행하기 위한 파라미터(parameter)이다.
              rospy.set_param("/play_alarml/fire_detection_state", False)
              #이전에 불을 발견했으나(즉, per_fire가 1이나) 발견되지 않은 시간을 저장한다
            else:
              for (x,y,w,h) in fire:
                 cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                 #화재 사진을 찍는다.
                 #'''
              for cnt in contours:  #중심점 잡기
                 cv2.drawContours(img_gray, [cnt], 0, (255, 0, 0), 5)

              for cnt in contours:  #윤곽선 잡기
                 hull = cv2.convexHull(cnt)
                 cv2.drawContours(img_gray, [hull], 0, (0, 0, 255), 5)
                 #'''
                 
                 df.save_picture(frame)
                 rospy.set_param("/play_alarml/fire_detection_state", True)
                 #화재 촬영하는 시간을 저장한다.
                 print("화재 장면을 {}회 찍었습니다.".format(j+1))
                 #사진 촬영의 횟수를 j에 반영한다.
                 j = j + 1

            cv2.imshow('frame', frame)
            cv2.imshow('HSV', rec3)
            cv2.imshow('contours', img_binary)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
