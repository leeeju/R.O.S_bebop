#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from playsound import playsound    

if __name__ == '__main__':

    rospy.init_node('play_alarm', anonymous=False)

    try:
        while True:
            if rospy.get_param("/play_alarml/fire_detection_state") is True:
                playsound('/home/kicker/catkin_ws/src/bb2_pkg/scripts/alarm.mp3') #상대경로로서 같은 경로의 폴더에 있는 audio.mp3를 실행한다.
                #화재를 알리는 SMS를 보내는 코드(예정)

    except rospy.ROSInterruptException:
        pass
