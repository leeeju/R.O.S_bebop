#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from math import radians, degrees, pi, sqrt
from bb2_pkg.msg import Pos_XYZ_th

LIN_SPD = 0.125
ANG_SPD = 0.50
'''
드론의 움직임을 총괄 하는 코드 입니다 
'''
class MoveBB2:

    def __init__(self):
        rospy.Subscriber('/bb2_pose_odom', Pos_XYZ_th, self.get_pos_xyzth_cb)
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        
        self.empty_msg = Empty()
        self.xyzth_now = self.xyzth_org = Pos_XYZ_th()
        
        
    def get_pos_xyzth_cb(self, msg):
        self.xyzth_now = msg
        
        
    def print_xyzth(self, msg):
        print "x = %s, y = %s, z = %s, th = %s" %(msg.x, msg.y, msg.z, degrees(msg.th))
        
        
    def update_org(self):
        self.xyzth_org = self.xyzth_now
        
        
    def elapsed_dist(self):
        return sqrt(pow((self.xyzth_now.x - self.xyzth_org.x), 2) + pow((self.xyzth_now.y - self.xyzth_org.y), 2))
        
        
    def elapsed_angle(self):
        return abs(self.xyzth_now.th - self.xyzth_org.th)
        
        
    def elapsed_height(self):
        return abs(self.xyzth_now.z - self.xyzth_org.z)
      

    def move_xy(self, distance, tolerance):  # 대각선 xy의 대한 움직임
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.xy =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.xy = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.xy, 2), round(self.xyzth_org.xy, 2))
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.xy, 2), round(self.xyzth_now.xy, 2))        
    

    def move_x(self, distance, tolerance):  # x 축의 대한 이동
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_y(self, distance, tolerance):   # y 축의 대한 이동
        tw = Twist()
        
        if distance >= 0:                    # distance(+): move left
            tw.linear.y =  LIN_SPD
        else:                                # distance(-): move right
            tw.linear.y = -LIN_SPD
            
        self.update_org()                    # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.y = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_z(self, height, tolerance):   # z 축의 대한 이동
        tw = Twist()
        
        if height >= 0:	# height(+): ascend
            tw.linear.z =  LIN_SPD
        else:			# height(-): descend
            tw.linear.z = -LIN_SPD
        
        self.update_org()
        print "start from: %s" %(round(self.xyzth_org.z, 2))
        
        while self.elapsed_height() < abs(height) - abs(height) * tolerance:
            self.pub0.publish(tw)
            
        tw.linear.z =  0;  self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop to   : %s" %(round(self.xyzth_now.z, 2))
        
        
    def rotate(self, angle, tolerance):     #방향 전환의 대한 코드
        tw = Twist()
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD
        
        self.update_org()
        print "start from: %s" %(round(degrees(self.xyzth_org.th), 2))
        
        while self.elapsed_angle() < abs(angle) - abs(angle) * tolerance:
            self.pub0.publish(tw)
            
        tw.angular.z =  0;  self.pub0.publish(tw) # stop move
        rospy.sleep(2.5)
        print "stop to   : %s" %(round(degrees(self.xyzth_now.th), 2))
        
    
    def takeoff(self):   # 이륙
        self.pub1.publish(self.empty_msg);  print "takeoff"
        
   
    def landing(self):   # 착륙
        self.pub2.publish(self.empty_msg);  print "landing"
        
    
    def emergency(self):  # 비행유지
        self.pub3.publish(self.empty_msg);  print "emergency"
        

