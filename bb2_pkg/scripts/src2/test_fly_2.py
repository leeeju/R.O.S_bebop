#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from bb2_pkg.Module_Gps_Class_20 import MoveByGPS

if __name__ == '__main__':
  rospy.init_node('Test_FlyWithoutRotate')

  mg = MoveByGPS()
  
  mg.mb.takeoff()
  rospy.sleep(3)
  mg.mb.landing()
  
