#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import detect_fire_test
from bb2_pkg.MoveBB2 import MoveBB2
from geometry_msgs.msg import Twist
from math import degrees, radians, pi
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class RotateByAtti:   # 클라스로 방향 전환을 만들어줌
    
    def __init__(self):
        rospy.init_node('bb2_sub_atti','bb2_sub_odom', anonymous = True)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.get_atti)
                        
        self.atti_now = 0.0

    def get_atti(self, msg):
        self.atti_now = msg.yaw

       # !!!드론의 머리를 돌려줘야함.!!! 
if __name__ == '__main__':       # 여기부터 메인 코드
    
    pb  = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)  # 버플리셔
    rba = RotateByAtti()                                            # 치환
    tw  = Twist()     
    bb2 = MoveBB2()    
    
    try:       
        print('탐색시작을 합니다.') 
              
        bb2.takeoff();  rospy.sleep(2.0)  #2초 대기 후 이륙.
        
        bb2.move_x(2.0, 0.1)   # 초회 직진 코드  (직진 거리, 오차 허용값)

        
        bb2.rotate(-90, 0.3)   # 몇도 돌릴지, 오차허용 값 (회전각도[-면 우회전], 오차 허용값)
        
        bb2.move_x(2.0, 0.01)   # 2회차 직진   
        
        bb2.rotate(-90, 0.3)
       
        bb2.move_x(3.0, 0.1)   # 3회차 직진(탐색)
        
        #bb2.rotate(85, 0.01)
        #rospy.sleep(1.0)
        #bb2.move_x(3.0, 0.1)   # 4회차 직진(탐색)
        
        #bb2.rotate(85, 0.01)
        #rospy.sleep(1.0)
        #bb2.move_x(4.0, 0.1)   # 5회차 직진(탐색)
        
        #bb2.rotate(85, 0.01)
        #rospy.sleep(1.0)
        #bb2.move_x(4.0, 0.1)   # 6회차 직진(탐색)
        
        #bb2.rotate(85, 0.01)
        #rospy.sleep(1.0)
        #bb2.move_x(5.0, 0.1)   # 7회차 직진(탐색) 
                               # 해결과제 드론의 고도(높이)의 따른 탐색 영역 확인 및 중첩 탐색
                              
                               
        bb2.landing();  rospy.sleep(2.0) # 2초 대기 후 착륙
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
