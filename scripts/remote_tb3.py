#!/usr/bin/env python

import rospy
from GetChar import GetChar
from geometry_msgs.msg import Twist

MAX_LIN_SPD =  0.22
MAX_ANG_SPD =  2.84

MIN_LIN_SPD = -0.22
MIN_ANG_SPD = -2.84

LIN_STP = 0.022
ANG_STP = 0.284

LIN_SPD = 0
ANG_SPD = 0

if __name__ == '__main__':

    rospy.init_node("move_turtlebot3")
    pb = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    tw  = Twist()
    key = GetChar()
        
    ch = "~"
        
    while not rospy.is_shutdown():
        
        ch = key.getch()
        
        if   ch == 'w':
            if MAX_LIN_SPD >= LIN_SPD + LIN_STP:
                LIN_SPD = LIN_SPD + LIN_STP
            else:
                LIN_SPD = MAX_LIN_SPD
        
        elif ch == 's':
            if MIN_LIN_SPD <= LIN_SPD - LIN_STP:
                LIN_SPD = LIN_SPD - LIN_STP
            else:
                LIN_SPD = MIN_LIN_SPD
        
        elif ch == 'a':
            if MAX_ANG_SPD >= ANG_SPD + ANG_STP:
                ANG_SPD = ANG_SPD + ANG_STP
            else:
                ANG_SPD = MAX_ANG_SPD
        
        elif ch == 'd':
            if MIN_ANG_SPD <= ANG_SPD - ANG_STP:
                ANG_SPD = ANG_SPD - ANG_STP
            else:
                ANG_SPD = MIN_ANG_SPD
        
        elif ch == ' ':
            LIN_SPD  =  0.0;    ANG_SPD = 0.0
        
        elif ch == '\x03':
            break
        
        else:   pass
        
        print "linear velocity = %s(m/sec),\tangular velocity = %s(rad/s)" %(LIN_SPD, ANG_SPD)
        tw.linear.x = LIN_SPD;  tw.angular.z = ANG_SPD
        pb.publish(tw)
        
    tw.linear.x = tw.angular.z = 0.0
    pb.publish(tw)
    
