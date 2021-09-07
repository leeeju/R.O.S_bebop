#!/usr/bin/env python
#-*- coding: utf-8 -*-

from twilio.rest import Client
import rospy
from playsound import playsound
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged

def cb_get_gps(msg):
    global lati_now
    global long_now
    lati_now = msg.latitude
    long_now = msg.longitude 

if __name__ == '__main__':
    rospy.init_node('play_alarm')
    rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged', Ardrone3PilotingStatePositionChanged, cb_get_gps)
    account_sid = 'AC3a674bf50d4d0511d8600550e6e50739'
    auth_token = '715e9cbd3d7bbd699606e9915362381c'
    client = Client(account_sid, auth_token)

    try:
        while not rospy.is_shutdown():
            if rospy.get_param("/play_alarml/fire_detection_state") is True:
                playsound('/home/kicker/catkin_ws/src/bb2_pkg/scripts/alarm.mp3') #상대경로로서 같은 경로의 폴더에 있는 audio.mp3를 실행한다.
                alarm_s1 = str(lati_now) + ', '
                alarm_s2 = str(long_now)
                alarm_s3 = "에서 화재가 발생하였습니다"
                alarm_message = alarm_s1+alarm_s2+alarm_s1+alarm_s3
                message = client.api.account.messages.create(to="+821077572419", from_="+17378885431", body= alarm_message )
    except rospy.ROSInterruptException:
        pass