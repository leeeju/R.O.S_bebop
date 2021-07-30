#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from bb2_pkg.MoveBB2 import MoveBB2 # 이동하느 값
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class RotateByAtti:   # 클라스 만들기
  def __init__(self):
    rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged', Ardrone3PilotingStateAttitudeChanged, self.get_atti)
    self.atti_now = 0.0

  def get_atti(self, msg):
    self.atti_now = msg.yaw
  
  def roundgo(self):
    #rba = RotateByAtti() 
    tw  = Twist()
    bb2 = MoveBB2()
	       
    print('round start')
    rospy.sleep(3.0)
    jump = 0.5

    for i in range(5):
       
      bb2.move_x(jump, 0.01)   # 초회 직진 코드  (직진 거리, 오차 허용값)
      rospy.sleep(1.0)                
      bb2.rotate(-90, 0.3)     # 몇도 돌릴지, 오차허용 값 (회전각도[-면 우회전], 오차 허용값)
      rospy.sleep(1.0)
      bb2.move_x(jump, 0.01)   # 2회차 직진
      jump = jump + 0.5   

    #bb2.landing()
    rospy.sleep(2.0) # 2초 대기 후 착륙


   # except rospy.ROSInterruptException:
    #  pass
        

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
