#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

# 메세지 수신 이벤트 발생 시 호출될 콜백함수 callback() 정의
def get_hello(msg):
    rospy.loginfo(rospy.get_caller_id() + ' subscribed message: %s', msg.data)

if __name__ == '__main__':
    rospy.init_node('sample_sub')
    rospy.Subscriber('hello', String, get_hello)
    
    rospy.spin()
