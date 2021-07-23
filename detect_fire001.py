#!/usr/bin/env python

import rospy, sys, cv2, os, datetime, time
import numpy as np
#import SMS_04
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from playsound import playsound

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

  def save_picture(self, picture):
    try:
      img = picture
      now = datetime.datetime.now()
      date = now.strftime('%Y%m%d')
      hour = now.strftime('%H%M%S')
      user_id = '00001'
      filename = '/home/cho/cvui_{}_{}_{}.png'.format(date, hour, user_id)
      cv2.imwrite(filename, img)       
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':

    rospy.init_node('fire_detector', anonymous=True)
    df = DetectFire()
    rospy.sleep(1.0)
    
    try:
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/fire_detector/scripts/fire_detection.xml')
        
        while not rospy.is_shutdown():
        
            frame = df.cv_msg
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)            
                             
            if len(fire) == 0:
                rospy.set_param("/fire_detector/fire_detection_state", False)
                #print(rospy.get_param("/fire_detector/fire_detection_state"))
            
            for (x,y,w,h) in fire:
                cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                #playsound('/home/kicker/catkin_ws/src/fire_detector/scripts/audio.mp3',block=False)
                playsound('C:/home/kicker/catkin_ws/src/fire_detector/scripts/audio.mp3')
                rospy.set_param("/fire_detector/fire_detection_state", True)
                
                fx = 0.0 
                fy = 0.0 

                fxs = 0.0 
                fys = 0.0 

                for i in range(50):

                    fcx = x + (w * 0.5)
                    fcy = y + (h * 0.5)

                    fxs = fxs + fcx
                    fys = fys + fcy
                    
                    fx = fxs / (i+1)
                    fy = fys / (i+1)

 
                if ((fx >= 190)and(fx <= 450)) and ((fy >= 150)and(fy <= 330)):
                    print("fire is center({}, {})".format(fx, fy))
                    print("Target found")
                    df.save_picture(frame)
                
                elif ((fx >= 190) and (fx <= 450)) and ((fy >= 331) and(fy <= 480)):
                        print("fire is not center({}, {})".format(fx, fy))
                        print("Go Down")
                    
                elif ((fx >= 190) and (fx <= 450)) and ((fy >= 0 ) and(fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Up.")                    
                                                  
                elif ((fx >= 0) and (fx <= 189)) and ((fy >= 151 ) and(fy <= 329)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Left")
     
                elif ((fx >= 0) and (fx <=189)) and ((fy >= 331) and (fy <= 480)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire left down")
                
                elif ((fx >= 0) and (fx <=189)) and ((fy >= 0) and (fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire left up")

                elif ((fx >= 450) and (fx <=640)) and ((fy >= 331) and (fy <= 640)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire right down")

                elif ((fx >= 450) and (fx <=640)) and ((fy >= 0) and (fy <= 149)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("fire right up")

                else:
                    if ((fx >= 451) and (fx <= 640)) and ((fy >= 150 ) and(fy <= 330)):
                      print("fire is not center({}, {})".format(fx, fy))
                      print("Go Right")
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                        
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
