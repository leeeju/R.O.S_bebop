#!/usr/bin/env python

import rospy
from bb2_pkg.MoveBB2_3 import MoveBB2


if __name__ == '__main__':
  rospy.init_node('test_move')
  mb = MoveBB2()
  mb.takeoff()
  rospy.sleep(3)
  mb.move_xy(-5, 0.1, -0.5, -0.5)
  mb.landing()


