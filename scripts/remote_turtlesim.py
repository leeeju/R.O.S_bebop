#!/usr/bin/env python

import rospy
from GetChar import GetChar
from geometry_msgs.msg import Twist
#import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node("move_turtlesim")
    pb = rospy.Publisher('/turtle1/cmd_vel', Twist)
    
    tw  = Twist()
    key = GetChar()
    
    ch = "~"

    while ch != 'Q':
        
        ch = key.getch()
        
        if   ch == 'w':
            tw.linear.x  =  2.0;    tw.angular.z =  0.0;    print "forward"
        
        elif ch == 's':
            tw.linear.x  = -2.0;    tw.angular.z =  0.0;    print "backward"
        
        elif ch == 'a':
            tw.linear.x  =  0.0;    tw.angular.z =  2.0;    print "rotate ccw"
        
        elif ch == 'd':
            tw.linear.x  =  0.0;    tw.angular.z = -2.0;    print "rotate cw"
        
        else:   pass
            
        pb.publish(tw)
    
