#!/usr/bin/env python

import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class ApriltagsLocator():
    def __init__(self, image=False):
        # Creates node apriltag_locator
        #subscribes to tag_detections (location of apriltags relative to camera), odom (odometery) and image_raw (video feed).
        #Publishes tag_location (position of the apriltag relative to the bebop's odometry) tag_pixel_location (location of the apriltags in pixels) and image_overlay (video feed with red circle over april tag)
        rospy.init_node("apriltags_locator")
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.process_apriltag)
        rospy.Subscriber("odom", Odometry, self.update_odom)
        self.pub_location = rospy.Publisher("tag_location", Point, queue_size = 1)
        self.pub_pixel = rospy.Publisher("tag_pixel_location", Point, queue_size = 1)
        self.tag_found_publisher = rospy.Publisher("tag_found", Bool, queue_size = 1)
        # Video feeds only active when specified in constructor
        if image:
            rospy.Subscriber("/bebop/image_raw", Image, self.image_overlay) #TODO Remove global path and replace with relative path.
            self.img_pub = rospy.Publisher("image_overlay", Image, queue_size=10)
            self.bridge = CvBridge()
            #Initialize Pixel Location
            self.xp = self.yp = 0

    # Run until node is stopped
    def run(self):
        rospy.spin()

    # Called when tag_detections from apriltag2_ros sends updates
    # Maps 3D apriltag location to pixel coordinates and publishes those coordinates to tag_location
    def process_apriltag(self, data):
        detections = data.detections
        # Only if the apriltag is detected
        if len(detections) > 0:
            # Alert the state machine that the tag has been found
            self.tag_found_publisher.publish(True)
            # Extract the position of the tag
            pose = detections[0].pose.pose.pose
            position = pose.position
            # TODO write calculation for position relative to Odom
            self.pub_location.publish(position)
            # Perspective Projection based upon a pinhole camera model
            f = 1 
            sx = 537.292878
            sy = 527.000348
            cx = 427.331854
            cy = 240.226888
            z = f
            y = f*position.y/position.z
            x = f*position.x/position.z
            #Pixel coordinates of the center of the AprilTag
            self.xp = int(sx*x+cx)
            self.yp = int(sy*y+cy)
            self.pub_pixel.publish(self.xp,self.yp,0)
        # Otherwise return origin
        else:
            self.tag_found_publisher.publish(False)
            self.pub_pixel.publish(0,0,0)
            self.pub_location.publish(0,0,0)

    #TODO write method and update self.pub_location
    def update_odom(self, data):
        pass

    # Called when new frame from the camera comes in
    # Overlays a red circle over the last known location of the apriltag
    def image_overlay(self, data):
        # Convert to openCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Draw a circle
        cv2.circle(cv_image, (self.xp, self.yp), 10, (0,0,255))
        # Convert back to ros image
        overlay_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.img_pub.publish(overlay_image)

if __name__=="__main__":
    # Launch Apriltags Locator. Use enable_image_overlay under the namespace apriltags_locator to set the image overlay option.
    al = ApriltagsLocator(image=rospy.get_param('apriltags_locator/enable_image_overlay', True))
    al.run()
