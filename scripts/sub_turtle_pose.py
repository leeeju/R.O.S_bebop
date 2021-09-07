#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees

pose = Pose()

def cb_get_pose(msg):
    pose = msg
    print "%s %s %s" %(pose.x, pose.y, degrees(pose.theta))
    

if __name__ == '__main__':

    rospy.init_node("sub_turtle_pose")
    rospy.Subscriber("/turtle1/pose", Pose, cb_get_pose)
    
    rospy.spin()
    
    
