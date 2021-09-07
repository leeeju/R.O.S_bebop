#!/usr/bin/env python

import rospy
#import geometry_msgs.msg
from geometry_msgs.msg import Twist

if __name__ == '__main__':

    rospy.init_node("move_turtlesim")
    pb = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    
    tw  = Twist()
    
    tw.linear.x  = 0.5
    tw.angular.z = 0.5

    while not rospy.is_shutdown():
        
        pb.publish(tw)
