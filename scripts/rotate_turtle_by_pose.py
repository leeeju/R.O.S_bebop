#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import degrees, radians, pi

MAX_ANG_SPD = 2.84
ANG_SPD = 0.25 * MAX_ANG_SPD

class RotateTurtle:
    def __init__(self):

        rospy.init_node("sub_turtle_pose")
        rospy.Subscriber("/turtle1/pose", Pose, self.cb_get_pose)
        self.pose = Pose()

    def cb_get_pose(self, msg):
        self.pose = msg
    

if __name__ == '__main__':
    
    try:
        pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        
        tw = Twist()
        rt = RotateTurtle()
        
        angle = radians(input("input angle(deg) to rotate: "))
        
        print "%s(deg), %s(rad)" %(degrees(angle), angle)
        
        #current = rt.get_org()
        target  = rt.pose.theta + angle
        
        print "from %s to %s" %(rt.pose.theta, target)
        
        
        if   rt.pose.theta >= 0 and target >= 0:
            '''
            <-+---------+------------------+-------+-------+------+-----+->
            -3.14  -3.14+Exc               0       T       C      T    3.14
            '''
            if target < pi: # target < 3.1415926535
                if   target > rt.pose.theta:    # cw : angular.z < 0
                    tw.angular.z =  ANG_SPD
                    while target > rt.pose.theta:
                        pub.publish(tw)
                elif target < rt.pose.theta:    # ccw: angular.z > 0
                    tw.angular.z = -ANG_SPD
                    while target < rt.pose.theta:
                        pub.publish(tw)
                else:   pass
            else:
                print "---"
                exceed = target - pi
                target = (pi - exceed) * -1.0
                
                tw.angular.z = -ANG_SPD
                while target < rt.pose.theta:
                    pub.publish(tw)
                
            #tw.angular.z = 0; pub.publish(tw)    
                 
        elif rt.pose.theta >= 0 and target <  0:
            '''
            <-+-------------+--------------+---------------+------------+->
            -3.14           T              0               C           3.14
            '''
        
            tw.angular.z = -ANG_SPD
            while target < rt.pose.theta:
                pub.publish(tw)
            #tw.angular.z = 0; pub.publish(tw)
        
            
        elif rt.pose.theta <  0 and target >= 0:
            '''
            <-+-------------+--------------+---------------+------------+->
            -3.14           C              0               T           3.14
            '''
            tw.angular.z =  ANG_SPD
            while target > rt.pose.theta:
                pub.publish(tw)
            #tw.angular.z = 0; pub.publish(tw)
        
        else:   #elif rt.pose.theta <  0 and target <  0:
            '''
            <-+-------+-------+------+-----+---------------------+------+->
            -3.14     T       C      T     0                (3.14-Exc) 3.14
            '''
            if target > -pi: # target < 3.1415926535
                if   target > rt.pose.theta:    # cw : angular.z < 0
                    tw.angular.z =  ANG_SPD
                    while target > rt.pose.theta:
                        pub.publish(tw)
                elif target < rt.pose.theta:    # ccw: angular.z > 0
                    tw.angular.z = -ANG_SPD
                    while target < rt.pose.theta:
                        pub.publish(tw)
                else:   pass
            else:
                print "---"
                exceed = abs(target) - pi
                target = pi - exceed
                
                tw.angular.z = ANG_SPD
                while target > rt.pose.theta:
                    pub.publish(tw)
        #else:   pass
        
        tw.angular.z = 0.0; pub.publish(tw); print "rotate end"
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
