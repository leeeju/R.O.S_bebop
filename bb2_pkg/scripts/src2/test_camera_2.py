#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, sys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged, \
                           Ardrone3PilotingStatePositionChanged, \
                           Ardrone3PilotingStateAltitudeChanged, \
                           Ardrone3GPSStateNumberOfSatelliteChanged
from scipy import sqrt, cos, sin, arctan2, pi
from math import degrees, radians


if __name__ == '__main__':
  rospy.init_node('test_camera')
  pub4 = rospy.Publisher('/bebop/camera_control', Twist, queue_size = 1)
  tw = Twist()
  tw.angular.y = 0.0
  rospy.spin()

