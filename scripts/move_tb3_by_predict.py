#!/usr/bin/env python
'''
################################################################################
#  Be sure the topic("/tb3pose") has to be published before start this code!!! #
################################################################################
'''
import rospy
from tutorial.MoveTB3 import MoveTB3 # <---- import like this
'''  ----------- -------     -------
          ^         ^              ^
src/tutorialr > MoveTB3.py > class MoveTB3:
'''
from math import radians

if __name__ == '__main__':

    try:
        rospy.init_node('rotate_by_pose', anonymous = True)        
        tb3 = MoveTB3()
        angle = radians(input("input angle to rotate(deg): "))
        tb3.rotate(angle)
        dist = float(input("input distance to stright(m): "))
        tb3.straight(dist)
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
