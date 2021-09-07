#!/usr/bin/env python

""" controller.py
Node for motor controllers.
Contains 2 controllers
1. X,Y location control
2. W rotation control
Note. The bebop will stay at a constant height
"""

import rospy
import potential_path as pp
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np

# An individual controller that controls a single component. One controller per algorithm.  This base class controller will output a constant speed.
class Controller(object):
    # axis is the storage place for the values returned by the controller. Initialize it with the appropriate length. 
    def __init__(self, axis=(0,0,0,0,0,0)):
        self.axis = axis

    # Update method is called whenever the speed is desired.
    def update(self, location=(0,0)):
        return self.constrain(self.axis)

    # Set the speeds for the base controller
    def set_speeds(axis):
        self.axis = axis

    # Defacto constrain method to keep values within the ranges demanded by bebop_autonomy
    def constrain(self, values):
        return np.clip(values, -1,1)

# Controller for the potential_path module
class PotentialController(Controller):
    # Either provide model, goal and obstacles, or use default
    def __init__(self, model=None, goal=None, obstacles=()):
        if model is not None:
            self.model = model
        elif goal is not None:
            self.model = pp.MapPotential(goal=goal, obstacles=obstacles)
        else:
            self.model = pp.MapPotential()
        super(PotentialController, self).__init__(axis=(0,0))

    # get gradient from model and constrain it.
    def update(self, location):
        axis = self.model.gradient(location)
        return super(PotentialController, self).update(location)
    
    def constrain(self, values):
        print("Yay child class method called")
        #TODO write potential controller specific constraints
        np.clip(values,-1,1)

# An aggregation of controllers. In this case: Location and Rotation.
# Pushes commands out to the cmd_pub_m node at 100ms intervals
class ControllerCommand(object):
    def __init__(self):
        rospy.init_node("controller_command")
        self.rate = rospy.Rate(10) #Rate of loop in Hz
        #Topic to change Location Controller
        rospy.Subscriber("location_controller", String, callback=self.change_controller, callback_args='location')
        #Topic to change Rotation Controller
        rospy.Subscriber("rotation_controller", String, callback=self.change_controller, callback_args='rotation')
        #Topic to track April Tag
        rospy.Subscriber("tag_pixel_location", Point, self.update_tag_pixel_coordinate)
        rospy.Subscriber("tag_location", Point, self.update_tag_location)

        #Publishing Velocities
        self.cmd_pub = rospy.Publisher("bebop/cmd_vel_set", Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = self.cmd_vel.linear.y = self.cmd_vel.linear.z = self.cmd_vel.angular.x = self.cmd_vel.angular.y = self.cmd_vel.angular.z = 0

        #Setting the Controllers
        self.location_controller_hover = Controller(0,0)
        self.location_controller = self.location_controller_hover
        self.rotation_controller_follow = PotentialController(
                goal=pp.GoalPotential(location=(320,184), z=0.05, kind='conic', d_threshold=50),
                obstacles=(pp.ObstaclePotential(location=(320,184),kind='circle', d_safe=50, r=180, n= 0.01, keep_out='in', max_gradient=3),))
        self.rotation_controller_search = Controller(axis=(0.25,))
        self.rotation_controller = self.rotation_controller_search
        self.tag_pixel_coordinate = (0,0)
    
    def change_controller(self, data, controller_type):
        if controller_type == 'location':
            if type(data) is String:
                if data.data == 'hover':
                    self.location_controller =  self.location_controller_hover
                elif data.data == 'follow':
                    pass
            #self.location_controller = data.something
            pass
        elif controller_type == 'rotation':
            if type(data) is String:
                if data.data == 'search':
                    self.rotation_controller = self.rotation_controller_search
                elif data.data == 'follow':
                    self.rotation_controller = self.rotation_controller_follow

    # Called when the apriltag location is parsed into (x,y) coordinates in pixels
    def update_tag_pixel_coordinate(self, data):
        self.tag_pixel_coordinate = (data.x, data.y)
    
    # Called when the apriltag location is determined relative to the odom reference frame
    def update_tag_location(self, data):
        self.tag_location = (data)

    # query controllers and publish commands at 100ms intervals 
    def run(self):
        while not rospy.is_shutdown():
            self.cmd_vel.linear.x, self.cmd_vel.linear.y = self.location_controller.update(self.position)
            self.cmd_vel.angular.z = self.rotation_controller.update(self.tag_pixel_coordinate)[0]
            self.cmd_pub.publish(self.cmd_vel)
            self.rate.sleep()

if __name__=="__main__":
    cc = ControllerCommand()
    cc.run()
