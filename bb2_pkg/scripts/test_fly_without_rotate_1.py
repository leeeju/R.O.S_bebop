#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from bb2_pkg.Module_Gps_Class_20 import MoveByGPS

if __name__ == '__main__':
  rospy.init_node('Test_FlyWithoutRotate')

  target_lati = float(input("목표 지점의 위도를 입력하세요:"))
  target_long = float(input("목표 지점의 경도를 입력하세요:"))

  mg = MoveByGPS()
  mg.fly_without_rotate(target_lati, target_long)
  
