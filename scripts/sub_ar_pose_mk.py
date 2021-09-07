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
    
        rospy.init_node('pub_marker_pose2d', anonymous = True)        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_pose2d_cb )
        self.pub = rospy.Publisher('/marker_pose2d', Pose, queue_size = 10)
        
        self.marker_pose2d = Pose()        
                                     
          
    def marker_pose2d_cb(self, msg):
    
        pose2d = Pose()
        
        for msg in msg.markers:
            if msg.id == TARGET_ID:
                print "position.x = %s" %(msg.pose.pose.position.x)

                print "position.y = %s" %(msg.pose.pose.position.y)
                
                print "position.z = %s" %(msg.pose.pose.position.z)
                
                print "orientation.x = %s" %(msg.pose.pose.orientation.x)
                
                print "orientation.y = %s" %(msg.pose.pose.orientation.y)
                
                print "orientation.z = %s" %(msg.pose.pose.orientation.z)
                
                print "orientation.w = %s" %(msg.pose.pose.orientation.w)                
        
        print "---"    
     
      
  
if __name__ == '__main__':
    try:
        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
