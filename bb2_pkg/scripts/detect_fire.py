#!/usr/bin/env python
import rospy, sys, cv2, time, datetime
import numpy as np
from PIL import Image as pil
from playsound import playsound
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DetectFire:

  def __init__(self):
    self.sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)
    self.sub = rospy.Subscriber("/image_raw", Image, self.callback)
    self.bridge = CvBridge()
    self.cv_msg = cv2.imread("/home/kicker/catkin_ws/src/bb2_pkg/scripts/fire2.png")    

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
      filename = '/home/kicker/Pictures/fire_{}_{}_{}.png'.format(now, hour, user_id)
      cv2.imwrite(filename, img)

    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':

    rospy.init_node('detect_fire', anonymous=True)
    df = DetectFire()
    rospy.sleep(1.0)
    
    try:
        fire_cascade = cv2.CascadeClassifier('/home/kicker/catkin_ws/src/bb2_pkg/scripts/cascade025.xml')
        
        while not rospy.is_shutdown():
            frame = df.cv_msg
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5)
            i = 0

            for (x,y,w,h) in fire:
              cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
              roi_gray = gray[y:y+h, x:x+w]
              roi_color = frame[y:y+h, x:x+w]
              print("fire is detected")
              playsound('/home/kicker/catkin_ws/src/bb2_pkg/scripts/alarm.mp3')
              if i < 5:
                df.save_picture(frame)
                i = i + 1
            
            if len(fire) == 0:
              i = 0
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                        
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


