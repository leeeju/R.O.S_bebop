#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, sys, cv2
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
      self.cv_msg = self.bridge.imgmsg_to_cv2(data, "bgr8") # 이미지 변환
        
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':

    rospy.init_node('fire_detector', anonymous=True)
    df = DetectFire()
    rospy.sleep(1.0)
    
    try:
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/detect_fire/scripts/fire_detection.xml')
        
        while not rospy.is_shutdown():
        
            frame = df.cv_msg
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 이미지변환
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)        # 표적을 추적하는 박스 생성     
                             
            if len(fire) == 0:
                rospy.set_param("/fire_detector/fire_detection_state", False)
                #print(rospy.get_param("/fire_detector/fire_detection_state"))
            
            for (x,y,w,h) in fire:
                cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                rospy.set_param("/fire_detector/fire_detection_state", True)
                print(rospy.get_param("/fire_detector/fire_detection_state",))
                print("fire is detected(x:{}, y:{}, w:{}, h:{})".format(x, y, w, h)) # 아이디어 :이선환
                                                                                     # 화면상의 표적 위치값 프린트
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                        
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
