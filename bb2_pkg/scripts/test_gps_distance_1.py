#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from bb2_pkg.Module_Gps_Class_20 import MoveByGPS

if __name__ == '__main__':

  rospy.init_node('test_gps_distance')
  
  lati1 = float(input("현재 지점의 위도를 입력하세요:"))
  long1 = float(input("현재 지점의 경도를 입력하세요:"))
  
  lati2 = float(input("목표 지점의 위도를 입력하세요:"))
  long2 = float(input("목표 지점의 경도를 입력하세요:"))
  
  mb = MoveByGPS()
  mb.cal_distance(lati1, long1, lati2, long2)
  
