#!/usr/bin/env python
import rospy
from playsound import playsound    
    
if __name__ == '__main__':

    rospy.init_node('play_alarm', anonymous=False)
    
    try:
    
        while not rospy.is_shutdown():
            while rospy.get_param("/fire_detector/fire_detection_state") is True:
                print "alarm!!!"
                playsound('/home/kicker/catkin_ws/src/detect_fire/Project/audio.mp3')
    
    except rospy.ROSInterruptException:
        pass
