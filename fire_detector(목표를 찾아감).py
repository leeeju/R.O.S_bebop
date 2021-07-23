#!/usr/bin/env python
import rospy, cv2, sys
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from bb2_pkg.MoveBB2 import MoveBB2
from math import radians, degrees

class image_converter:

  def __init__(self):    
    rospy.Subscriber("/image_raw",Image,self.callback)    
    self.bridge = CvBridge()
    self.cv_img = cv2.imread("/home/kicker/catkin_ws/src/fire_detector/scripts/fire.jpg")
    
    self.x = self.y = self.w = self.h = 0
    
  def callback(self,data):
    try:
      self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    '''          x        x + w                     640
           +-----+---------+------------------------+
           |(x,y)|    w    |                        |
        y  +-----x----+----+------------------------+
           |     |////|////|                        |
           |   h +----o----+  o(x+w/2, y+h/2)       |
           |     |////|////|                        |
     y + h +-----+----+----x------------------------+
           |     |         |(x+w, y+h)              |
           |     |         |                        |
           |     |         |                        |
           |     |         |                        |
           |     |         |                        |
       480 +-----+---------+------------------------+
    
    '''
  def get_center(self):
    if self.x == self.y == self.w == self.h == 0:
      return (0, 0)
    else:
      return (self.x + 0.5 * self.w, self.y + 0.5 * self.h)
  
  def get_area(self):
      return self.w * self.h
  
  def detect(self, cascade):
    pub_img = rospy.Publisher("/with_box", Image, queue_size = 10)    
    pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
    
    tw = Twist()
    offset = 10    
    
    image = ic.cv_img
    gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    faces = cascade.detectMultiScale(
      gray,
      scaleFactor=1.1,
      minNeighbors=5,
      minSize=(30, 30),
      flags = cv2.CASCADE_SCALE_IMAGE
    )
    
    if len(faces) == 0:
      self.x = self.y = self.w = self.h = 0
    
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        self.x = x; self.y = y; self.w = w; self.h = h
        print("%s, %s, %s, %s" %(self.x, self.y, self.w, self.h))
    
    if self.w != 0:
        #'''
        if   self.w > 130 + offset:
            tw.linear.x = -0.125
            print "self.w: %s" %(ic.w)    
        elif self.w < 130 - offset:
            tw.linear.x =  0.125
            print "self.w: %s" %(ic.w)
        else:
            tw.linear.x =  0.0
        #'''
        center = self.get_center()
        
        if   center[0] > 320 + offset:
            tw.angular.z = -radians(10)
            print "center_x: %s" %(ic.w)    
        elif center[0] < 320 - offset:
            tw.angular.z = radians(10)
            print "center_x: %s" %(ic.w)
        else:
            tw.linear.x =  0.0
            
        pub_cmd.publish(tw)
        
    else:
        tw.linear.x  = tw.linear.y  = tw.linear.z  = 0.0
        tw.angular.x = tw.angular.y = tw.angular.z = 0.0
        pub_cmd.publish(tw)  
      
        
    #cv2.imshow("Faces found", image)
    pub_img.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    
      
    
    
    #if self.get_area() != 0:
        
    
    #cv2.waitKey(0)      
    #if cv2.waitKey(1) > 0:
    #   break
 
    
if __name__ == '__main__':
  
  rospy.init_node('image_converter', anonymous=True)
  
  ic = image_converter()
  tw = Twist() 
  rospy.sleep(1.0) 
  offset = 5
  
  cascade_filename = '/home/kicker/catkin_ws/src/fire_detector/scripts/fire_detection.xml'
  cascade = cv2.CascadeClassifier(cascade_filename)
  
  try: 
    while not rospy.is_shutdown():
    
      ic.detect(cascade)
        
        
      #if cv2.waitKey(1) & 0xFF == ord('q'):
      #  break
    
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()
