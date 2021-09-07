#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import degrees, radians, pi

MAX_ANG_SPD = 2.84
ANG_SPD = 0.5 * MAX_ANG_SPD

class MoveTB3:
    def __init__(self):

        rospy.Subscriber("/tb3pose", Pose, self.cb_get_pose)
        self.pose = Pose()

    def cb_get_pose(self, msg):
        self.pose = msg
        
    def rotate(self, angle):
           
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        tw = Twist()
        
        target  = self.pose.theta + angle
        
        print "from %s to %s" %(self.pose.theta, target)        
        
        if   target > self.pose.theta:
        
            tw.angular.z =  ANG_SPD
            
            pub.publish(tw)
            while self.pose.theta < target:
                pass
            
        elif target < self.pose.theta:
        
            tw.angular.z = -ANG_SPD
            
            pub.publish(tw)
            while self.pose.theta > target:
                pass
            
        else:   pass
        
        tw.angular.z = 0.0; pub.publish(tw); print "rotate end at %s" %(self.pose.theta)
    

if __name__ == '__main__':
    
    try:
        rospy.init_node("move_tb3")
        
        mt = MoveTB3()
        
        angle = radians(input("input angle(deg) to rotate: "))
        
        mt.rotate(angle)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
