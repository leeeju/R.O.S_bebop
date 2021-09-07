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

class TrackMarker:

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
        self.target_found = False
        
        self.sq1_found_target_1st_time = False  # stop / save tb3_pose.theta / rotate
        self.sq2_lost_target_1st_time  = False  # stop / save tb3_pose.theta / get angle for align to marker
        self.sq3_align_to_marker_end   = False  # align to marker / get ref_pose.theta
        self.sq4_turn_4_move2front_end = False  # rotate(90(deg) - ref_pose.theta)
        self.sq5_move_to_front_end     = False  # straight(ref_pose.y)
        self.sq6_turn_to_marker_end    = False  # rotate(90(deg)) to marker
        self.sq7_approach_2_marker_end = False  # approach to marker
        
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
        '''
        if self.start_find == True:
            self.tw.angular.z = ANG_SPD
        else:
            self.tw.angular.z = 0.0
        
        self.pub.publish(self.tw)
        '''
        pose2d = Pose()
        
        if len(msg.markers) != 0:
        
            for msg in msg.markers:
            
                if msg.id == TARGET_ID:
                    
                    self.target_found = True
                    
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
                else:
                    self.target_found = False
        else:
            self.target_found = False
               
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
        self.start_find = False; self.tw.angular.z = 0.0;     self.pub.publish(self.tw);    rospy.sleep(0.5)
    
    def stop_finding(self):
        self.start_find = False; self.tw.angular.z = 0.0;     self.pub.publish(self.tw);    rospy.sleep(0.5)
    
    def start_finding(self):
        self.start_find = True;  self.tw.angular.z = ANG_SPD; self.pub.publish(self.tw)
    
    def restart_finding(self):
        self.start_find = True;  self.tw.angular.z = ANG_SPD; self.pub.publish(self.tw)
    
    def change_wise(self):
        self.wise *= -1
    
    def get_tb3_theta(self):
        return self.tb3_pose.theta
    
    def get_angle_4_align(self):
        return -abs(abs(self.theta1) - abs(self.theta2))
    
    def align_2_marker(self):
        angle = self.get_angle_4_align()
        self.tb3.rotate(angle);     rospy.sleep(0.5)
    
    def get_ref_ar_pose(self):
        self.ref_pose = self.ar_pose
    '''    
    self.sq1_found_target_1st_time = False  # stop / save tb3_pose.theta / rotate
    self.sq2_lost_target_1st_time  = False  # stop / save tb3_pose.theta / get angle for align to marker
    self.sq3_align_to_marker_end   = False  # align to marker / get ref_pose.theta
    self.sq4_turn_4_move2front_end = False  # rotate(90(deg) - ref_pose.theta)
    self.sq5_move_to_front_end     = False  # straight(ref_pose.y)
    self.sq6_turn_to_marker_end    = False  # rotate(90(deg)) to marker
    self.sq7_approach_2_marker_end = False  # approach to marker
    '''     
    def track_marker(self):
    
        while not rospy.is_shutdown():
            '''
            if self.start_find == True:
                self.tw.angular.z = ANG_SPD
            else:
                self.tw.angular.z = 0.0
                
            self.pub.publish(self.tw)
            '''
            if self.target_found == True:
                print "target found"
                #1
                if self.sq1_found_target_1st_time == False: 
                    self.stop_finding();    self.theta1 = self.get_tb3_theta()
                    print "sq1_found_target_1st_time: %s" %(degrees(self.theta1))
                    self.sq1_found_target_1st_time = True
                    self.restart_finding()
                #4
                if self.sq3_align_to_marker_end == True and self.sq4_turn_4_move2front_end == False:
                    
                    if self.ref_pose.theta >= 0:
                        self.wise =  1
                    else:
                        self.wise = -1
                    
                    self.tb3.rotate((radians(90)-abs(self.ar_pose.theta))*self.wise); rospy.sleep(0.5)
                    self.sq4_turn_4_move2front_end = True
                    print "sq4_turn_4_move2front_end"
                #7    
                if self.sq4_turn_to_marker_end == True and self.sq5_approach_2_marker_end == False:
                    self.stop_move()
                    print "---"
                    
            elif self.target_found == False:
                print "target lost"
                #2
                if self.sq1_found_target_1st_time == True and self.sq2_lost_target_1st_time == False:
                    self.stop_finding();    self.theta2 = self.get_tb3_theta()
                    print "sq2_lost_target_1st_time: %s" %(degrees(self.theta2))
                    self.sq1_2_lost_target_1stime = True
                #3
                if self.sq2_lost_target_1st_time == True and self.sq3_align_to_marker_end == False:    
                    self.align_2_marker();  self.get_ref_ar_pose()
                    self.sq3_align_to_marker_end = True
                    print "self.sq3_align_to_marker_end"
                #5
                if self.sq4_turn_4_move2front_end == True and self.sq5_move_to_front_end == False:
                    
                    if self.ref_pose.theta >= 0:
                        Kp = 1.85
                    else:
                        Kp = 1.0
                    
                    self.tb3.straight(abs(self.ref_pose.y) * Kp);   rospy.sleep(0.5)
                    print "sq5_move_to_front_end"
                    self.sq5_move_to_front_end = True
                #6
                if self.sq5_move_to_front_end == True and self.sq6_turn_to_marker_end == False:
                
                    if self.ref_pose.theta >= 0:
                        self.wise = -1;  Kp = 0.85
                    else:
                        self.wise =  1;  Kp = 1.0
                    
                    self.tb3.rotate(radians(90) * self.wise * Kp);  rospy.sleep(0.5)
                    print "sq4_turn_to_marker_end"
                    self.sq6_turn_to_marker_end = True
            else:   pass
            
            rospy.spin()

if __name__ == '__main__':
    try:        
        tm = TrackMarker()
        
        while not rospy.is_shutdown():
            #'''
            if tm.start_find == True:
                tm.tw.angular.z = ANG_SPD
            else:
                tm.tw.angular.z = 0.0
                
            tm.pub.publish(tm.tw)
            #'''
            if tm.target_found == True:
                print "target found"
                #1
                if tm.sq1_found_target_1st_time == False: 
                    tm.stop_finding();    tm.theta1 = tm.get_tb3_theta()
                    print "sq1_found_target_1st_time: %s" %(degrees(tm.theta1))
                    tm.sq1_found_target_1st_time = True
                    tm.restart_finding()
                #4
                if tm.sq3_align_to_marker_end == True and tm.sq4_turn_4_move2front_end == False:
                    
                    if tm.ref_pose.theta >= 0:
                        tm.wise =  1
                    else:
                        tm.wise = -1
                    
                    tm.tb3.rotate((radians(90)-abs(tm.ar_pose.theta))*tm.wise); rospy.sleep(0.5)
                    tm.sq4_turn_4_move2front_end = True
                    print "sq4_turn_4_move2front_end"
                #7    
                if tm.sq4_turn_to_marker_end == True and tm.sq5_approach_2_marker_end == False:
                    tm.stop_move()
                    print "---"
                    
            else:
                print "target lost"
                #2
                if tm.sq1_found_target_1st_time == True and tm.sq2_lost_target_1st_time == False:
                    tm.stop_finding();    tm.theta2 = tm.get_tb3_theta()
                    print "sq2_lost_target_1st_time: %s" %(degrees(tm.theta2))
                    tm.sq1_2_lost_target_1stime = True
                #3
                if tm.sq2_lost_target_1st_time == True and tm.sq3_align_to_marker_end == False:    
                    tm.align_2_marker();  tm.get_ref_ar_pose()
                    tm.sq3_align_to_marker_end = True
                    print "tm.sq3_align_to_marker_end"
                #5
                if tm.sq4_turn_4_move2front_end == True and tm.sq5_move_to_front_end == False:
                    
                    if tm.ref_pose.theta >= 0:
                        Kp = 1.85
                    else:
                        Kp = 1.0
                    
                    tm.tb3.straight(abs(tm.ref_pose.y) * Kp);   rospy.sleep(0.5)
                    print "sq5_move_to_front_end"
                    tm.sq5_move_to_front_end = True
                #6
                if tm.sq5_move_to_front_end == True and tm.sq6_turn_to_marker_end == False:
                
                    if tm.ref_pose.theta >= 0:
                        tm.wise = -1;  Kp = 0.85
                    else:
                        tm.wise =  1;  Kp = 1.0
                    
                    tm.tb3.rotate(radians(90) * tm.wise * Kp);  rospy.sleep(0.5)
                    print "sq4_turn_to_marker_end"
                    tm.sq6_turn_to_marker_end = True
            #else:   pass
            
            rospy.spin()
        
    except rospy.ROSInterruptException:  pass
