#!/usr/bin/env python

import rospy, sys, cv2
import numpy as np
from playsound import playsound
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectFire:

  def __init__(self):
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber("/image_raw", Image, self.callback)
    self.cv_img = Image()

  def callback(self,data):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imwrite("/home/gnd0/catkin_ws/src/detect_fire/scripts/img_msg.png", cv_img)
        
    except CvBridgeError as e:
      print(e)
    
    #img = cv2.imread('/home/gnd0/catkin_ws/src/detect_fire/scripts/img_msg.png')
    
    #cv2.imshow("Image window", img)
    #cv2.waitKey(3)
    
    
if __name__ == '__main__':

    rospy.init_node('detect_fire', anonymous=True)
    df = DetectFire()
    rospy.sleep(0.5)
    #rospy.spin()
    #'''
    try:
        fire_cascade = cv2.CascadeClassifier('/home/gnd0/catkin_ws/src/detect_fire/scripts/fire_detection.xml')
        #cap = np.asarray(ic.cv_img) # cap = cv2.VideoCapture(0)
        
        while not rospy.is_shutdown():
            #frame = df.cv_img       #ret, frame = cap.read()
            frame = cv2.imread('/home/gnd0/catkin_ws/src/detect_fire/scripts/img_msg.png')
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)
            
            for (x,y,w,h) in fire:
                cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                print("fire is detected")
                playsound('/home/gnd0/catkin_ws/src/detect_fire/scripts/audio.mp3')

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    #'''


