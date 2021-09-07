#!/usr/bin/env python
#-*- coding: utf-8 -*-

'''
화재 지점을 포착하면 화재지점의 좌표를 사용자에게 문자로 보내줌.
'''
import rospy
from twilio.rest import Client
from playsound import playsound
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged

def cb_get_gps(msg):
    global lati_now
    global long_now
    lati_now = msg.latitude
    long_now = msg.longitude 

if __name__ == '__main__':
    rospy.init_node('play_alarm') # (노드 초기화) 화재가 식별 되면 알람을 울림 
    rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged', Ardrone3PilotingStatePositionChanged, cb_get_gps)  # 목표 지점의 좌표(위도,경도)를 받아옴
    account_sid = 'AC3a674bf50d4d0511d8600550e6e50739'  # twilio 개인키
    auth_token = '715e9cbd3d7bbd699606e9915362381c'     # twilio 개인키
    client = Client(account_sid, auth_token)

    try:
        while True:
            if rospy.get_param("/play_alarml/fire_detection_state") is True:
                playsound('/home/kicker/catkin_ws/src/bb2_pkg/scripts/alarm.mp3') # 상대경로로서 같은 경로의 폴더에 있는 audio.mp3를 실행한다.
                alarm_s1 = str(lati_now) + ', '                                   # 위도
                alarm_s2 = str(long_now)                                          # 경도
                alarm_s3 = "에서 화재가 발생하였습니다"                                 # 프린트문
                alarm_message = alarm_s1+alarm_s2+alarm_s1+alarm_s3               # 문자열 연결
                message = client.api.account.messages.create(to="+821077572419", from_="+17378885431", body= alarm_message )
    except rospy.ROSInterruptException:                       #    개인휴대폰         #  twilio 개인번호        [s1+s2+s3]이 합처진 문자열
        pass
