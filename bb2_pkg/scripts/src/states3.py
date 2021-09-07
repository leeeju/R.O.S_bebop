#!/usr/bin/python
#-*- coding: utf-8 -*-


import rospy
from math import *
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  


class StateMachine():
    def __init__(self):
        rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged) 
          
    def update_batter(self):
        print(CommonCommonStateBatteryStateChanged)
        


if __name__ == '__main__':
    sm = StateMachine()
    #bt = Battery()
 
    
    
    
    
    
    
    
    
