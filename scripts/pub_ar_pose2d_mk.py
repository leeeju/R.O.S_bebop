#!/usr/bin/env python

import sys
import rospy
from turtlesim.msg import Pose
from math import degrees, radians, sin, cos, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = int(sys.argv[1])

class MarkerPose:

    def __init__(self):
    
        rospy.init_node('pub_marker_pose2d')#, anonymous = True)        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.pub_marker_pose2d_cb )
        self.pub = rospy.Publisher('/marker_pose2d', Pose, queue_size = 10)
        
        #self.marker_pose2d = Pose()
        """   
                                                 ////////////| ar_marker |////////////
                y                      z         --------+---------+---------+--------
                ^  x                   ^                 |     R-0/|\R-0    R|
                | /                    |                 |       /0|0\       |
         marker |/                     | robot           |      /  |  \      |
                +------> z    x <------+                 |     /   |   \     |
                                      /                  |  dist   |  dist   |   R-0 == 90.degrees
                                     /                   |   /     |     \   |
                                    y                    |  /      |      \  |
                                                         | /       |       \0|
                                                         |/R-0    R|R    R-0\|
        pose.x = position.z                 [-]  (0 < O) x---------+---------x (0 > 0) [+]
        pose.y = position.x              [0]roll         ^                   ^   
        theta  = euler_from_quaternion(q)[1]pitch        |                   |
                                         [2]yaw        robot               robot
        """        
    def pub_marker_pose2d_cb(self, msg):
    
        pose2d = Pose()
        
        for msg in msg.markers:
        
            if msg.id == TARGET_ID:
            
                theta = self.get_marker_th(msg)
                
                if  (theta >  degrees(270)): 
                    pose2d.theta = theta - 2 * pi            
                elif(theta < -degrees(270)):
                    pose2d.theta = theta + 2 * pi
                else:
                    pose2d.theta = theta
                
                pose2d.x = msg.pose.pose.position.z
                pose2d.y = msg.pose.pose.position.x
                
                #self.marker_pose2d = pose2d
                self.pub.publish(pose2d)                
                self.print_pose(pose2d)       
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
    def get_marker_th(self, msg):
    
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        quart = euler_from_quaternion(q)
        theta = quart[1]
        
        if theta < 0:
            theta = theta + 2 * pi
        if theta > 2 * pi:
            theta = theta - 2 * pi

        return theta
    
        
    def print_pose(self, pose2d):
        print "pose2d.x = %5s, pose2d.y = %5s, pose2d.theta = %6s" %(pose2d.x, pose2d.y, degrees(pose2d.theta))
          

if __name__ == '__main__':
    try:        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
    
