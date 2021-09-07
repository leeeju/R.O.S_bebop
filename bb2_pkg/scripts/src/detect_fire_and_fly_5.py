#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, sys, cv2, os, datetime, time
import numpy as np
#import SMS_04                                                  #<--- 출발과 동시에 문자를 보낼떄 import 앞의 # 을 제거 하세요.
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DetectFire:

  def __init__(self):
    self.sub = rospy.Subscriber("/image_raw", Image, self.callback)    # Subscriber   ---> image_raw
    self.bridge = CvBridge()
    self.cv_msg = cv2.imread("fire2.jpeg")                             # 이지미 파일

  def callback(self,data):
    try:
      self.cv_msg = self.bridge.imgmsg_to_cv2(data, "bgr8")            # cv 브릿지를 사용한 영상변환
        
    except CvBridgeError as e:
      print(e)

  def save_picture(self, picture):     #실시간 켑쳐 된 영상이 들어오는 경로 및 저장 형식
     try:
      img = picture
      now = datetime.datetime.now()
      date = now.strftime('%Y%m%d')    #년,월,일
      hour = now.strftime('%H%M%S')    #시,분,초
      user_id = '00001'
      filename = '/home/kicker/cvui_{}_{}_{}.png'.format(date, hour, user_id)   #저장 형태
      cv2.imwrite(filename, img)                                                # 이미지를저장 한다      
     except CvBridgeError as e:
      print(e)


if __name__ == '__main__':   # 여기서 부터 메인문

    rospy.init_node('fire_detector', anonymous=True)
    df = DetectFire()
    rospy.sleep(1.0)
    
    try:
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/bb2_pkg/scripts/fire_detection.xml')   # 불을 찾기 위한 데이터 [.xml]
        
        while not rospy.is_shutdown():
        
            frame = df.cv_msg
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)
                             
            if len(fire) == 0:
                rospy.set_param("/fire_detector/fire_detection_state", False)
                #print(rospy.get_param("/fire_detector/fire_detection_state"))
            
            for (x,y,w,h) in fire:                                                                       #표적에 사각형을 박스를 생성한다
                cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                rospy.set_param("/fire_detector/fire_detection_state", True)
                
                fx = 0.0 
                fy = 0.0 

                fxs = 0.0 
                fys = 0.0 

                for i in range(50):                                                                     #표적이 중앙 오면 표적의 위치를 계산[50]번 해서 좌표를 산출

                    fcx = x + (w * 0.5)                                                                 # 계산 부분
                    fcy = y + (h * 0.5)

                    fxs = fxs + fcx
                    fys = fys + fcy
                    
                    fx = fxs / (i+1)
                    fy = fys / (i+1)

 
                if ((fx >= 190)and(fx <= 450)) and ((fy >= 150)and(fy <= 330)):                        #표적의 위치에 따라서 프린터 되는 츨력 및 좌표 값.
                    print("fire is center({}, {})".format(fx, fy))
                    print("Target found")
                    df.save_picture(frame)
                
                elif ((fx >= 190) and (fx <= 450)) and ((fy >= 331) and(fy <= 480)):                   # 1차 방정식 사용 [화면의 중심값]
                        print("fire is not center({}, {})".format(fx, fy))
                        print("Go Down")
                        rospy.set_param("fly_by_param_07/param_to_fly", 2)                             # param으로 넘겨주는 숫자 값 (2) [후면]
                    
                elif ((fx >= 190) and (fx <= 450)) and ((fy >= 0 ) and(fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Up.")
                      rospy.set_param("fly_by_param_07/param_to_fly", 8)                               # 8번 전면

                elif ((fx >= 0) and (fx <= 189)) and ((fy >= 151 ) and(fy <= 329)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Left")
                      rospy.set_param("fly_by_param_07/param_to_fly", 4)                               # 4번 좌측

                elif ((fx >= 0) and (fx <=189)) and ((fy >= 331) and (fy <= 480)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire left down")
                      rospy.set_param("fly_by_param_07/param_to_fly", 2)                               # 8번 후면

                elif ((fx >= 0) and (fx <=189)) and ((fy >= 0) and (fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire left up")
                      rospy.set_param("fly_by_param_07/param_to_fly", 8)                               # 8번 전면

                elif ((fx >= 450) and (fx <=640)) and ((fy >= 331) and (fy <= 640)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire right down")
                      rospy.set_param("fly_by_param_07/param_to_fly", 2)                               # 2번 후면                            

                elif ((fx >= 450) and (fx <=640)) and ((fy >= 0) and (fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire right up")
                      rospy.set_param("fly_by_param_07/param_to_fly", 8)                               # 8번 전면

                else:
                    if ((fx >= 451) and (fx <= 640)) and ((fy >= 150 ) and(fy <= 330)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Right")
                      rospy.set_param("fly_by_param_07/param_to_fly", 6)                               # 4번 우측

            cv2.imshow('frame', frame)   #종료 할때
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                        
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
