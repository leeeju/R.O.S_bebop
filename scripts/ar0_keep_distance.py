#!/usr/bin/env python

import sys
import rospy
from turtlesim.msg import Pose
from math import degrees, radians, sin, cos, pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = int(sys.argv[1])

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.25
ANG_SPD = MAX_ANG_SPEED * 0.0625

DIST_TO_KEEP = 0.35
MARGIN = 0.05

'''
pub_tb3_pose2d.py
use class MoveTB3()
from tutorial.MoveTB3 import MoveTB3

'''

class MarkerPose:

    def __init__(self):
    
        rospy.init_node('keep_distance')#, anonymous = True)       
        rospy.Subscriber('/tb3pose', Pose, self.get_tb3_pose_cb )
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.pub_marker_pose2d_cb )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        self.ar_pose = Pose()
        self.tw = Twist()
        
        """   
                                                 ////////////| ar_marker |////////////
                y                      z         --------+---------+---------+--------
                ^  x                   ^                 |     R-0/|\R-0    R|
                | /                    |                 |       /0|0\       |
         marker |/                     | robot           |      /  |  \      |
                +------> z    x <------+                 |     /   |   \     |
                                      /                  |  dist   |  dist   |
                                     /                   |   /     |     \   |
                                    y                    |  /      |      \  |
                                                         | /       |       \0|
                                                         |/R-0    R|R    R-0\|
        pose.x = position.z                      (0 < O) x---------+---------x (0 > 0)
        pose.y = position.x              [0]roll         ^                   ^   
        theta  = euler_from_quaternion(q)[1]pitch        |                   |
                                         [2]yaw        robot               robot
        """
            
    def pub_marker_pose2d_cb(self, msg):
        
        pose2d = Pose()
        
        if len(msg.markers) != 0:
        
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                    print "target found"
                    '''
                    theta = self.get_marker_th(msg)
                    
                    if  (theta >  degrees(270)): 
                        pose2d.theta = theta - 2 * pi            
                    elif(theta < -degrees(270)):
                        pose2d.theta = theta + 2 * pi
                    else:
                        pose2d.theta = theta
                    '''
                    pose2d.x = msg.pose.pose.position.z
                    pose2d.y = msg.pose.pose.position.x
                    
                    self.ar_pose = pose2d
                    
                    dist = self.ar_pose.x
                    
                    print(dist)
                    
                    if   dist > 0.3:
                        self.tw.linear.x =  LIN_SPD
                    
                    elif dist < 0.2:
                        self.tw.linear.x = -LIN_SPD
                        
                    else: # (DIST_TO_KEEP - MARGIN) <= distance <= (DIST_TO_KEEP + MARGIN)
                        dist =  0
                    
                    self.pub.publish(self.tw)
                        
                    
                else:
                    self.tw.linear.x =  0;  self.pub.publish(self.tw)
        else:
            self.tw.linear.x =  0;  self.pub.publish(self.tw)
               
    def get_marker_th(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """ 
    
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        quart = euler_from_quaternion(q)
        theta = quart[1]
        
        if theta < 0:
            theta = theta + 2 * pi
        if theta > 2 * pi:
            theta = theta - 2 * pi

        return theta
    
    def get_tb3_pose_cb(self, msg):
        self.tb3_pose = msg
        
    def print_pose(self, pose2d):
        print "pose2d.x = %5s, pose2d.y = %5s, pose2d.theta = %6s" %(pose2d.x, pose2d.y, degrees(pose2d.theta))
          

if __name__ == '__main__':
    try:        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
