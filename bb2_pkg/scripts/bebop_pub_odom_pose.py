#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from math import pi, degrees
from tf.transformations import euler_from_quaternion
from bb2_pkg.msg import Pos_XYZ_th

class OdomPose:

    def __init__(self):
        rospy.init_node('bb2_pub_odom_pose', anonymous = True)
        rospy.Subscriber('/bebop/odom', Odometry, self.get_odom_cb )
        self.pub = rospy.Publisher('/bb2_pose_odom', Pos_XYZ_th, queue_size = 1)
        
        self.xyzth_now = Pos_XYZ_th()
        self.theta_prv = 0.0
        self.theta_sum = 0.0
        
    def get_odom_cb(self, msg):
        self.xyzth_now.x  = msg.pose.pose.position.x
        self.xyzth_now.y  = msg.pose.pose.position.y
        self.xyzth_now.z  = msg.pose.pose.position.z
        
        theta = self.get_theta(msg)
        
        if   (theta - self.theta_prv) >  5.0: #  5.0(rad) =  286.479(deg)
            d_theta = (theta - self.theta_prv) - 2 * pi            
        elif (theta - self.theta_prv) < -5.0: # -5.0(rad) = -286.479(deg)
            d_theta = (theta - self.theta_prv) + 2 * pi
        else:
            d_theta = (theta - self.theta_prv)

        self.theta_sum    = self.theta_sum + d_theta
        self.theta_prv    = theta
        self.xyzth_now.th = self.theta_sum
        
        self.pub.publish(self.xyzth_now)
        self.print_xyzth(self.xyzth_now)         
        
    def get_theta(self, dat):
        q = (dat.pose.pose.orientation.x, dat.pose.pose.orientation.y, 
             dat.pose.pose.orientation.z, dat.pose.pose.orientation.w)
                                            # quart[0] = roll
        quart = euler_from_quaternion(q)    # quart[1] = pitch
        theta = quart[2]                    # quart[2] = yaw <----
        
    	# make theta within from 0 to 360 degree
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        return theta        
        
    def print_xyzth(self, msg):
        print "x = %s, y = %s, z = %s, th = %s" %(msg.x, msg.y, msg.z, degrees(msg.th))
    
if __name__ == '__main__':

    try:
        OdomPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
