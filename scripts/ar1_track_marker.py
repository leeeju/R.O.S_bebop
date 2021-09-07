#!/usr/bin/env python

import sys
import rospy
from turtlesim.msg import Pose
from math import degrees, radians, sin, cos, tan, pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from tutorial.MoveTB3 import MoveTB3

TARGET_ID = int(sys.argv[1])

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.25
ANG_SPD = MAX_ANG_SPEED * 0.0625

class MarkerPose:

    def __init__(self):
    
        rospy.init_node('track_marker')#, anonymous = True)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.pub_marker_pose2d_cb )    
        rospy.Subscriber('/tb3pose', Pose, self.get_tb3_pose_cb )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        self.ar_pose = self.tb3_pose = self.ref_pose = Pose()
        self.tw      = Twist()
        self.tb3     = MoveTB3()
        
        self.wise    = 1
        
        self.theta1  = self.theta2 = self.theta3 = 0.0        
        
        self.start_find   = True        
        self.found_target = False
        
        self.sq1_align_to_marker_end   = False
        self.sq1_1_found_target_1stime = False
        self.sq1_2_lost_target_1stime  = False
        self.sq2_turn_4_move2front_end = False
        self.sq3_move_to_front_end     = False
        self.sq4_turn_to_marker_end    = False
        self.sq5_approach_2_marker_end = False
        
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
        
        if self.start_find == True:
            self.tw.angular.z =  ANG_SPD
        else:
            self.tw.angular.z =  0
        self.pub.publish(self.tw)
        
        pose2d = Pose()
        
        if len(msg.markers) != 0:
        
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                    
                    theta = self.get_marker_th(msg)
                    
                    if   theta >  radians(270): 
                        pose2d.theta = theta - radians(360)            
                    elif theta < -radians(270):
                        pose2d.theta = theta + radians(360)
                    else:
                        pose2d.theta = theta
                    
                    pose2d.x = msg.pose.pose.position.z
                    pose2d.y = msg.pose.pose.position.x
                    
                    self.ar_pose = pose2d
                    
                    if self.sq1_align_to_marker_end == False:
                    
                        if self.sq1_1_found_target_1stime == False:
                            self.stop_finding();    rospy.sleep(0.5)
                            self.theta1 = self.tb3_pose.theta
                            print "sq1_1_found_target_1stime: %s" %(degrees(self.theta1))
                            self.sq1_1_found_target_1stime = True
                            self.start_finding()    # marker visible --> invisible
                        else:   self.start_finding()
                        
                    if self.sq1_align_to_marker_end == True and self.sq2_turn_4_move2front_end == False:
                        
                        if self.ref_pose.theta >= 0:
                            self.wise =  1;     Kp = 1.3
                        else:
                            self.wise = -1;     Kp = 0.9
                        
                        self.tb3.rotate((radians(90)-abs(self.ar_pose.theta)) * self.wise * Kp); rospy.sleep(.5)
                        self.sq2_turn_4_move2front_end = True
                        print "sq2_turn_4_move2front_end"   # marker invisible
                    
                    if self.sq4_turn_to_marker_end == True and self.sq5_approach_2_marker_end == False:
                        self.stop_move()
                        print "---"
                                       
                    
                else:
                    self.found_target = False
        else:
            if self.sq1_align_to_marker_end == False:
                if self.sq1_1_found_target_1stime == True and self.sq1_2_lost_target_1stime == False:
                    self.stop_finding();        rospy.sleep(0.5)
                    self.theta2 = self.tb3_pose.theta
                    print "sq1_2_lost_target_1stime: %s" %(degrees(self.theta2))
                    
                    self.theta3 = abs(abs(self.theta2) - abs(self.theta1)) * 0.5
                    print "  theta for align = %s" %(degrees(self.theta3))
                    
                    self.change_wise(); self.tb3.rotate(self.theta3 * self.wise)
                    self.stop_move();               rospy.sleep(0.5)
                    self.ref_pose = self.ar_pose    # <---- update self.ref_pose
                    self.sq1_2_lost_target_1stime = True
                    self.sq1_align_to_marker_end = True                   
                    print "sq1_align_to_marker_end" # maker visible
            # 3        
            if self.sq2_turn_4_move2front_end == True and self.sq3_move_to_front_end == False:
                if self.ref_pose.theta >= 0:
                    Kp = 1.75
                else:
                    Kp = 1.75
                self.tb3.straight(abs(self.ref_pose.y) * Kp);   rospy.sleep(0.5)
                self.sq3_move_to_front_end = True
                print "sq3_move_to_front_end"   # maker invisible
                    
            if self.sq3_move_to_front_end == True and self.sq4_turn_to_marker_end == False:
                
                if self.ref_pose.theta >= 0:
                    self.wise = -1;  Kp = 0.95
                else:
                    self.wise =  1;  Kp = 0.95
                
                self.tb3.rotate(radians(90) * self.wise * Kp);  rospy.sleep(0.5)
                self.sq4_turn_to_marker_end = True
                print "sq4_turn_to_marker_end"  # marker visible
               
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
            theta = theta + radians(360)
        if theta > radians(360):
            theta = theta - radians(360)

        return theta
    
    def get_tb3_pose_cb(self, msg):
        self.tb3_pose = msg
        
    def print_pose(self, pose2d):
        print "pose2d.x = %5s, pose2d.y = %5s, pose2d.theta = %6s" %(pose2d.x, pose2d.y, degrees(pose2d.theta))
    
    def stop_move(self):
        self.start_find = False; self.tw.angular.z = 0.0;     self.pub.publish(self.tw)
    
    def start_finding(self):
        self.start_find = True;  self.tw.angular.z = ANG_SPD; self.pub.publish(self.tw)
    
    def stop_finding(self):
        self.start_find = False; self.tw.angular.z = 0.0;     self.pub.publish(self.tw)
    
    def change_wise(self):
        self.wise *= -1
          

if __name__ == '__main__':
    try:        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
