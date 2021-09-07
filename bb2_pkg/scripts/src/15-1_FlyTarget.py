#!/usr/bin/env python
#-*- coding: utf-8 -*-
#사용자가 지정한 목적지로 가는 코드를 작성하면 아래와 같다.

import rospy, os, threading
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bb2_pkg.bebop_move_by_gps_module_4 import MoveByGPS
from bb2_pkg.round_move import RotateByAtti

def startdetecting():
  os.system("rosrun bb2_pkg 15-3_detect_fire.py ")

if __name__ == '__main__':
    #노드를 초기화한다.
    rospy.init_node('fly_to_target', anonymous = True) 
    #이륙을 위한 퍼블리셔를 초기화한다.
    pb0 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 0)
    #메시지를 초기화한다.
    tw = Twist()
    em = Empty()
    #목적지로 비행하는 코드를 사용하기 위하여 객체를 초기화한다.
    mbg = MoveByGPS()
    #이륙한다.
    pb0.publish(em)

    #이륙을 알리는 SMS를 보낸다.(코드 추가 예정)
    #code none

    #정찰 비행을 하다가 불을 인식했다가 끊긴 경우 다시 목적지로 되돌아가 순회하도록 하기 위하여 아래와 같이 반복문의 장치를 둔다.
    while True:
        #파라미터 서버로부터 위경도의 정보를 입력받는다.
        target_la = rospy.get_param("/lati")
        target_lo = rospy.get_param("/long")

        #위도와 경도가 유효한 숫자일 경우에 아래의 비행 코드를 실행한다. 여기서 유효한 위경도의 값이란 대한민국의 범위에 해당하는 위경도 값이다.
        if ( (target_la >=34.0)and(target_la <=38.0 ) )and( (target_lo >= 126.0)and(target_lo <=130.0 ) ) :

            #목적지로 이동하는 코드를 실행한다.
            mbg.fly_to_target(target_la, target_lo)

            #도착했다면, 드론이 정북을 바라보도록 한다. 아직 코드는 없다.
            #code none

            #화재를 인식하는 코드를 활성화한다.
            rospy.set_param("/fire_detectorl/param_of_detector", "1")
            sf = threading.Thread(target=startdetecting())
            sf.start()

            #목적지에서 순회하는 코드를 실행하기 위하여 객체를 만들고 실행한다.
            rfy = RotateByAtti()
            rfy.roundgo()

            #불 인식 코드를 종료한다
            os.system("rosnode kill /fire_detectorl")

            #처음 시작점으로 되돌아 오는 비행 코드를 활성화한다.
            rospy.set_param("/FlyBackl/param_of_back", "1")

            #코드를 종료한다.
            exit()
