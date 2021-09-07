#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import degrees, radians, pi

MAX_ANG_SPD = 2.84
ANG_SPD = 0.25 * MAX_ANG_SPD

class RotateTB3:
    def __init__(self):

        rospy.init_node("rotate_tb3")
        rospy.Subscriber("/tb3pose", Pose, self.cb_get_pose)
        self.pose = Pose()

    def cb_get_pose(self, msg):
        self.pose = msg
    

if __name__ == '__main__':
    
    try:
        rt = RotateTB3()
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        tw = Twist()
        
        angle = radians(input("input angle(deg) to rotate: "))
        
        print "%s(deg), %s(rad)" %(degrees(angle), angle)
        
        target  = rt.pose.theta + angle
        
        print "from %s to %s" %(rt.pose.theta, target)        
        
        if   target > rt.pose.theta:
        
            tw.angular.z =  ANG_SPD
            
            pub.publish(tw)
            while rt.pose.theta < target:
                pass
            
        elif target < rt.pose.theta:
        
            tw.angular.z = -ANG_SPD
            
            pub.publish(tw)
            while rt.pose.theta > target:
                pass
            
        else:   pass
        
        tw.angular.z = 0.0; pub.publish(tw); print "rotate end at %s" %(rt.pose.theta)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
