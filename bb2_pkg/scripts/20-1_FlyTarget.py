#!/usr/bin/env python
#-*- coding: utf-8 -*-

#사용자가 지정한 목적지로 가는 코드를 작성하면 아래와 같다.

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bb2_pkg.Module_Gps_Class_20 import MoveByGPS
from bb2_pkg.round_move_2 import RotateByAtti

if __name__ == '__main__':
    #노드를 초기화한다.
    rospy.init_node('fly_to_targetl', anonymous = False)
    #메시지를 초기화한다.
    tw = Twist()
    em = Empty()
    #사용자로부터 입력받은 위도, 경도를 저장할 변수를 아래와 같이 선언한다.
    target_la = 0.0
    target_lo = 0.0
    #시작점의 위도, 경도를 저장할 변수를 아래와 같이 선언한다.
    start_la = 0.0
    start_lo = 0.0
    #목적지로 비행하는 코드를 사용하기 위하여 객체를 초기화한다.
    mbg = MoveByGPS()

    #정찰 비행을 하다가 불을 인식했다가 끊긴 경우 다시 목적지로 되돌아가 순회하도록 하기 위하여 아래와 같이 반복문의 장치를 둔다.
    while not rospy.is_shutdown():

            #비행 모드의 파라미터 값을 확인한다.
            fmode = rospy.get_param("/fly_to_targetl/param_of_flying")
            #목표 지점으로 비행하는 모드라면, (즉, fmode가 1이라면)
            if fmode == 1:
                print("목표로 이동하는 모드를 시작합니다.")
                
                #파라미터 서버로부터 위경도의 정보를 입력받을 때까지 서버로부터 입력을 계속 입력 받는다.
                while not ( target_la >=34.0 and target_la <=38.0 and target_lo >= 126.0 and target_lo <=130.0 ):
                    if not rospy.is_shutdown():
                        target_la = rospy.get_param("/tar_lati")
                        target_lo = rospy.get_param("/tar_long")
                print('목표 지점의 좌표를 {}, {}로 입력 받았습니다.'.format(target_la, target_lo))
                #FlyTarget이 실행 중이라면, 메니저 노드에서 다시 실행하지 않아도 되기 때문에 해당 파라미터를 0으로 초기화한다.
                rospy.get_param("/Fire_drone_managerl/order", 0)
                #목적지로 이동하기 전에 불 인식 코드를 비활성화한다
                rospy.set_param("/fire_detectorl/param_of_detector", False)
                #화재 경보를 알리는 코드를 비활성화한다.
                rospy.set_param("/play_alarml/fire_detection_state", False)
                print("설정을 완료했습니다.")
                #목적지로 이동하는 코드를 실행한다.
                mbg.fly_to_target(target_la, target_lo)
                print("도착했습니다.")
                #순찰하는 비행모드로 파라미터 값을 바꾼다.
                rospy.set_param("/fly_to_targetl/param_of_flying", 2)
                #메니저 노드에서 다시 flyTartget 노드를 실행하도록 파라미터를 설정한다.
                rospy.set_param("/Fire_drone_managerl/order", 1)
                exit()

            if fmode == 2:
                print("순찰 비행을 시작합니다.")
                #도착한 후에 화재를 인식하는 코드를 활성화한다.
                rospy.sleep(5)
                rospy.set_param("/fire_detectorl/param_of_detector", True)
                #목적지에서 순찰하는 코드를 실행하기 위하여 객체를 만들고 실행한다.
                print("화재 탐지를 시작합니다.")
                la_unit = 0.000001
                lo_unit = 0.000001
                while not rospy.is_shutdown():
                    target_la = rospy.get_param("/tar_lati")
                    target_lo = rospy.get_param("/tar_long")
                    mbg.fly_without_rotate(target_la, target_lo)
                    i = rospy.get_param("/patrol_num")
                    if i <= 0:
                        break
                    elif i%2 == 0:
                        #북쪽의 위도 좌표로 이동한다.
                        target_la = target_la + la_unit
                        mbg.fly_without_rotate(target_la, target_lo)
                        #동쪽의 경도 좌표로 이동한다.
                        target_lo = target_lo + lo_unit
                        mbg.fly_without_rotate(target_la, target_lo)
                        #목표의 위도/경도를 파라미터에 저장한다.
                        rospy.set_param("/tar_lati", target_la)
                        rospy.set_param("/tar_long", target_lo)
                        #이동 단위를 증가시킨다.
                        la_unit = la_unit + la_unit
                    else:
                        #남쪽의 위도 좌표로 이동한다.
                        target_la = target_la - la_unit
                        mbg.fly_without_rotate(target_la, target_lo)
                        #서쪽의 좌표로 이동한다.
                        target_lo = target_lo - lo_unit
                        mbg.fly_without_rotate(target_la, target_lo)
                        #목표의 위도/셩도를 파라미터에 저장한다.
                        rospy.set_param("/tar_lati", target_la)
                        rospy.set_param("/tar_long", target_lo)
                        #이동 단위를 증가시킨다.
                        lo_unit = lo_unit + lo_unit
                    i = i - 1
                    rospy.set_param("/patrol_num", i)
                #순찰을 마친 후에 불 인식 코드를 비활성화한다.
                rospy.set_param("/fire_detectorl/param_of_detector", False)
                print("화재 탐지를 마쳤습니다.")
                #비행 모드를 되돌아오는 모드로 바꾸는 파라미터를 설정한다.
                rospy.set_param("/fly_to_targetl/param_of_flying", 3)
                #메니저 노드에서 다시 flyTartget 노드를 실행하도록 파라미터를 설정한다.
                rospy.set_param("/Fire_drone_managerl/order", 1)
                exit()

            #처음 시작점으로 되돌아오는 비행모드라면, (즉, fmode가 2라면) 아래의 코드를 실행한다.
            if fmode == 3:
                print("시작 지점으로 돌아가는 모드를 시작합니다.")
                #화재 인식 코드를 비활성화한다.
                rospy.set_param("/fire_detectorl/param_of_detector", False)
                #매니저 노드에서 다시 비행 노드를 실행하지 않도록 파라미터를 설정한다.
                rospy.set_param("/Fire_drone_managerl/order", 0)
                #화재 경보를 알리는 코드를 비활성화한다.
                rospy.set_param("/play_alarml/fire_detection_state", False)
                #파라미터 서버로부터 시작점의 위도와 경도를 가져온다.
                while not ( start_la >=34.0 and start_la <=38.0 and start_lo >= 126.0 and start_lo <=130.0 ):
                    if not rospy.is_shutdown():
                        start_la = rospy.get_param("/ori_lati")
                        start_lo = rospy.get_param("/ori_long")
                print('시작 지점의 좌표를 {}, {}로 입력 받았습니다.'.format(start_la, start_lo))
                print("시작 지점으로 되돌아갑니다.")
                #시작점으로 되돌아 가는 비행을 한다.
                mbg.fly_to_target(start_la, start_lo)
                #착륙한다.
                mbg.mb.landing()
                #종료한다.
                exit()
  