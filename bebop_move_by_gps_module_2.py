#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, sys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from math import degrees, radians, pi, atan2
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged, \
                           Ardrone3PilotingStatePositionChanged       #GPS 신호(위성을 받아옴)
from haversine import haversine
from scipy import cos, sin, arctan, sqrt, arctan2
from bb2_pkg.MoveBB2 import MoveBB2


'''
    GPS for center of map  (  36.32793567300377, 127.42284217051652 )
    Parot-Sphinx start GPS (  48.87890000000000,   2.36778000000000 )
    diffrence              ( -12.550964327,     +125.055062171 )
'''
OFFSET_LAT = -12.358483     #-12.550964327
OFFSET_LON = 124.805327     #125.055062171
PI         = 3.14159265358979323846
DEG_PER_M  = 0.00001097872031629
LIN_SPD    = 1.0

'''                     (x2,y2) -> (lat2,lon2)
                               p2   slope:S, intercept:I
                               /|         
  x:latitude                  / |     S = (y2-y1)/(x2-x1)            S = (lon2-lon1)/(lat2-lat1)
                             /  |   
  y:longitude               /   |     y = Sx + I            
                           /    |   
                          /     |     I = y - Sx  
                         /      |     
                        /_______|       = y1 - S * x1
                       p1             
              (x1,y1) -> (lat1,lon1)    = y1 - (y2-y1)/(x2-x1) * x1   

                                        = lon1 - (lon2-lon1)/(lat2-lat1) * lat1

  when center is (a,b), equation of circle : pow((x-a),2) + pow((y-b),2) = pow(r,2)

'''

class MoveByGPS:

    def __init__(self):
        #rospy.init_node('bb2_move_to_gps', anonymous = True)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.cb_get_atti) #드론의 변화되는 고도(Altitude) 또는 방향의 정보를 구독한다. 여기서 Attitude는 고도(Altitude) 또는 방향을 나타내는 것으로 추론된다. 
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged',
                         Ardrone3PilotingStatePositionChanged,
                         self.cb_get_gps) #드론의 변화되는 위치(position)를 구독한다.

        self.atti_now =   0.0 # 방향(yaw) 값의 초기값으로 0.0을 준다.
        self.lati_now = 500.0 # 위도(latitude)의 초기값으로 500을 준다. 위도란, 적도로부터 남북으로 얼마나 떨어져 있는가를 나타내는 지표 정보이다.
        self.long_now = 500.0 # 경도(longitude)의 초기값으로 500을 준다. 경도란, 본초자오선(현재 케임브리지 천문대)으로부터 얼마나 떨어져 있는가를 나타내는 지표 정보이다.
        
        self.bearing_now = 0.0 #현재의 방향값을 0.0으로 설정한다.
        self.bearing_ref = 0.0 #방향값의 참조값(reference)을 0.0으로 설정한다.

        self.margin_angle  = self.deg2rad(10) #
        self.margin_radius = DEG_PER_M * 5.0 #
        #self.takeoff()
        rospy.sleep(3.0)

    def cb_get_atti(self, msg):
        self.atti_now = msg.yaw

    def cb_get_gps(self, msg):
            self.lati_now = msg.latitude  + OFFSET_LAT #보정값을 더해서 현재의 위도값을 얻는다.
            self.long_now = msg.longitude + OFFSET_LON #보정값을 더해서 현재의 경도값을 얻는다.
       
        #print("latitude = %s, longitude = %s" %(self.lati_now, self.long_now))

    def deg2rad(self, deg):
        return deg * PI / 180

    def rad2deg(self, rad):
        return rad * 180 / PI

    def get_bearing(self, p1_lati, p1_long, p2_lati, p2_long):
        #p1~, p2~ 등에서 원지점과 도착점이 각각 무엇인지 불명확하다. 
        
        P1_LAT = self.deg2rad(p1_lati) # p1_lati 위도를 라디안으로 바꾸어 P1_LAT에 저장한다.
        P2_LAT = self.deg2rad(p2_lati) # p2_lati 위도를 라디안으로 바꾸어 P2_LAT에 저장한다.

        LONG_DIFFERENCE = self.deg2rad(p2_long - p1_long) #원지점과 도착점의 경도의 차이를 LONG_DIFFERENCE에 저장한다.

        y = sin(LONG_DIFFERENCE) * P2_LAT
        x = cos(P1_LAT) * sin(P2_LAT) - sin(P1_LAT) * cos(P2_LAT) * cos(LONG_DIFFERENCE)

        return arctan2(y, x)    # return radian value. 도착점의 방향값을 반환한다.

    def rotate(self, lat1, lon1, lat2, lon2):
        tw = Twist()
        pb = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        bearing = self.get_bearing(lat1, lon1, lat2, lon2) #드론이 일정 지점에서 어떤 방향으로 돌려야 하는지 그 값을 얻는다.
        
        # bearing 값과 atti_now의 값의 차이를 구한다. atti_now는 현재의 야우(yaw) 값을 저장한 변수이다. 여기서 0을 기준으로 angle값에 양수를 주거나 음수를 주는 것으로 보아 0은 정면이고 음수나 양수는 오른쪽이나 왼쪽일 가능성이 높다.
        if   bearing - self.atti_now > 0.0:
            angle =  abs(bearing - self.atti_now)
        elif bearing - self.atti_now < 0.0:
            angle = -abs(bearing - self.atti_now)
        else:   pass

        current = self.atti_now
        target  = current + angle
        
        # retarget for case of passing +180 or -180
        if   target > PI:
            target = -PI + (target - PI)
        elif target < -PI:
            target =  PI + (target + PI)
        else:   pass
        
        print "start from: %s" %(self.rad2deg(current))
        
        if   current >= 0 and target >= 0:
            '''                                   |     T             C         T
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            print "case1"
            if   target > current: # 각주의 그림을 참고했을 때, 타켓(target)이 커렌트(current)보다 크다면, angular.z값을 양수로 넣어 주어야 하지 않을까?
                tw.angular.z = -0.2
                while target > self.atti_now:
                    pb.publish(tw); 
            elif target < current:
                tw.angular.z =  0.2
                while target < self.atti_now:
                    pb.publish(tw)
            else:   pass
        
        elif current <  0 and target <  0:
            '''     T     C             T         |                              
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            print "case2"
            if   target > current:
                tw.angular.z = -0.2
                while target > self.atti_now:
                    pb.publish(tw)
            elif target < current:
                tw.angular.z =  0.2
                while target < self.atti_now:
                    pb.publish(tw)
            else:   pass
        
        elif current <  0 and target >= 0:
            '''           C                       |                 T            
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            print "case3"
            tw.angular.z = -0.2
            while target > self.atti_now:
                pb.publish(tw)
            
        elif current >= 0 and target <  0:
            '''           T                       |                 C            
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            print "case4"
            tw.angular.z = 0.2
            while target < self.atti_now:
                pb.publish(tw)

        else:   pass

        tw.angular.z = 0.0; pb.publish(tw); rospy.sleep(2.0)        
        print "stop to   : %s" %(self.rad2deg(self.atti_now))

    def get_gps_now(self):
        return self.lati_now, self.long_now

    def check_route(self, lat2, lon2):

        lat_now, lon_now = self.get_gps_now()
        
        bearing = self.get_bearing(lat_now, lon_now, lat2, lon2)
        
        if bearing > self.bearing_ref - self.margin_angle and \
           bearing < self.bearing_ref + self.margin_angle:
           #즉, bearing이 self.bearing_ref의 오차 범위 sel.margin_angle 내에 있다면,
            return True #참(True)을 반환한다.
        else:
            return False
    
    
    def check_arrived(self, lat2, lon2):
        '''
        when center is (a,b), equation of circle : pow((x-a),2) + pow((y-b),2) = pow(r,2)
        pow((lat_now-lat2), 2) + pow((lon_now-lon2), 2) = pow(self.margin_radius, 2)
        self.margin_radius = sqrt(pow((lat_now-lat2), 2) + pow((lon_now - lon2), 2))
        '''
        lat_now, lon_now = self.get_gps_now()
        #오차 범위를 일정하게 정해준다. 오차 범위를 정해주기 위해서는 개인적으로 보기엔 배열을 활용해야 할 것 같다.
        radius = sqrt(pow((lat_now-lat2), 2) + pow((lon_now - lon2), 2))
        
        #도착 지점의 오차 범위에 도착했는지를 확인한다. 
        if radius < self.margin_radius:
        #self.margin_radius가 radius 내에 있다면
            return True #참을 반환하라.
        else:
            return False


    def move_to_target(self, lat1, lon1, lat2, lon2):
        
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)        
        tw  = Twist()
        
        while self.check_arrived(lat2, lon2) is False:
            
            if self.check_route(lat2, lon2) is True:
                tw.linear.x = LIN_SPD;  pub.publish(tw)
            else:
                tw.linear.x = 0;  pub.publish(tw); rospy.sleep(2.0)
                lat1, lon1 = self.get_gps_now()
                angle = self.get_bearing(lat1, lon1, lat2, lon2)
                self.bearing_ref = angle
                self.rotate(lat1, lon1, lat2, lon2)
                
        print "arrived to target gps position!!!"
        tw.linear.x = 0;  pub.publish(tw); rospy.sleep(2.0)

    def fly_to_target(self, p2_lati_deg, p2_long_deg):
        rospy.sleep(5)
        pb0 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 0)
        em = Empty()
        pb0.publish(em)
        
        p1_lati_deg = self.lati_now
        p1_long_deg = self.long_now
        print "p1(%s, %s), p2(%s, %s)" %(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)
        self.bearing_ref = self.get_bearing(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)
        atti_now        = self.atti_now
        print "current = %s, target = %s" %(self.rad2deg(atti_now), self.rad2deg(self.bearing_ref))  
        self.rotate(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)
        self.move_to_target(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)
 
    
