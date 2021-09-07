#!/usr/bin/env python
#-*- coding: utf-8 -*-

'''
시작점(home)으로 되돌아 오는 코드입니다 
'''
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bb2_pkg.bebop_move_by_gps_module_5 import MoveByGPS

if __name__ == '__main__':
    rospy.init_node('FlyBack', anonymous = True)
    pb1 = rospy.Publisher('/bebop/land', Empty, queue_size = 0)
    tw = Twist()
    em = Empty()
    mbg = MoveByGPS()

    #일단, 구동하자마자 현 시작점의 위도와 경도값을 입력 받는다. 단, 구동을 하고 있으되, 파라미터가 1일 경우에만 되돌아오는 비행 코드가 동작하도록 하였다.
    target_lad1 = mbg.lati_now
    target_lod1 = mbg.long_now

    while True:
        #파라미터가 참일 경우에 되돌아 온다.
        if rospy.get_param("/FlyBackl/param_of_back") == "1":
            rospy.sleep(1)
            #시작점으로 되돌아 가는 비행을 한다.
            mbg.fly_to_target(target_lad1, target_lod1)
            #착륙한다.
            pb1.publish(em)
            #코드를 종료한다.
            exit()
    
