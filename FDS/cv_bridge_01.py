#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                     Image, self.image_callback)
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    cv2.imshow("window", image)
    cv2.waitKey(3)
follower = Follower()
rospy.init_node('follower')
rospy.spin()

# [출처] ROS kinetic에서 cv_bridge 사용하려면 어떻게 해야할까요 (오픈소스 소프트웨어 & 하드웨어: 로봇 기술 공유 카페 (오로카)) | 작성자 방화동명근이
