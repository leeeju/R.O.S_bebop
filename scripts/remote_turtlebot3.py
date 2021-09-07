#!/usr/bin/env python

import rospy
from GetChar import GetChar
from geometry_msgs.msg import Twist

MAX_LIN_SPD = 0.22
MAX_ANG_SPD = 2.84

LIN_SPD = MAX_LIN_SPD * 0.125
ANG_SPD = MAX_ANG_SPD * 0.125

if __name__ == '__main__':

    rospy.init_node("move_turtlebot3")
    pb = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    tw  = Twist()
    key = GetChar()
        
    ch = "~"
        
    while ch != 'Q':
        
        ch = key.getch()
        
        if   ch == 'w':
            tw.linear.x  =  LIN_SPD;    tw.angular.z =  0.0;    print "forward"
        
        elif ch == 's':
            tw.linear.x  = -LIN_SPD;    tw.angular.z =  0.0;    print "backward"
        
        elif ch == 'a':
            tw.linear.x  =  0.0;    tw.angular.z =  ANG_SPD;    print "rotate ccw"
        
        elif ch == 'd':
            tw.linear.x  =  0.0;    tw.angular.z = -ANG_SPD;    print "rotate cw"
        
        elif ch == ' ':
            tw.linear.x  =  0.0;    tw.angular.z = 0.0;    print "stop"
        
        else:   pass
            
        pb.publish(tw)
        rospy.sleep(1.0)
        
        tw.linear.x = tw.angular.z = 0.0
        pb.publish(tw)
        
    tw.linear.x = tw.angular.z = 0.0
    pb.publish(tw)
    
