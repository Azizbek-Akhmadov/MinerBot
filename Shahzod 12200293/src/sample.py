#!/usr/bin/env python3

# Importing necessary libraries and modules
import rospy
import rospkg
import pathlib
from gazebo import Gazebo
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import time
import tf
import numpy as np

# Constants for image dimensions
IMG_WIDTH = 400
IMG_HEIGHT = 400


# Main class for controlling the TurtleBot
class Main():
    def __init__(self):
        # Initializing the ROS node
        rospy.init_node('turtlebot_main')
        rospy.loginfo('Starting up node...')

        # Setting the control rate to 20 Hz
        self.r = rospy.Rate(20)

        # Recording the start time for time-based operations
        self.start_time = time.time()

        # Registering a shutdown callback function
        rospy.on_shutdown(self.on_shutdown)

        # Initializing Gazebo interface, CvBridge for image conversion
        self.gazebo = Gazebo()
        self.bridge = CvBridge()

        # Getting ROS parameter values or using defaults
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.cam_frame = rospy.get_param('~cam_frame', '/camera/image_raw')
        self.scan_frame = rospy.get_param('~scan_frame', '/scan')

        # Creating publishers and subscribers
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_scan = rospy.Subscriber(self.scan_frame, LaserScan, self.scan_cb)
        self.odom = rospy.Subscriber(self.odom_frame, Odometry, self.odom_cb)
        self.cam = rospy.Subscriber(self.cam_frame, Image, self.image_cb)

        # Flag to log image resolution only once
        self.logged_res = False
        # Variables to store the latest readings
        self.latest_image = None
        self.latest_odom_reading = None
        self.latest_scan_reading = None

        # Moving the TurtleBot for 30 seconds
        mv = Twist()
        while (time.time() - self.start_time) < 30:
            mv.linear.x = 0.0
            mv.angular.z = 0.5
            self.cmd_vel.publish(mv)
            self.r.sleep()
        # Stopping the TurtleBot after 30 seconds
        self.stop()
        rospy.loginfo('Stopping...')
        # Resetting the Gazebo simulation
        self.gazebo.reset_sim()
        rospy.loginfo('Resetting...')

    # Callback function for laser scan messages
    def scan_cb(self, msg):
        rospy.logdebug('Receiving scanning message...')
        # Placeholder implementation, no processing done here
        self.latest_scan_reading = None
        self.gazebo.__un

    # Callback function for odometry messages
    def odom_cb(self, msg):
        rospy.logdebug('Receiving odometry message...')
        # Placeholder implementation, no processing done here
        self.latest_odom_reading = None

    # Callback function for image messages
    def image_cb(self, msg):
        # Converting ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Logging the image resolution once
        if not self.logged_res:
            height, width, _ = cv_image.shape
            rospy.loginfo(
                'Original image resolution: {} x {}'.format(width, height))
            self.logged_res = True
        # Resizing the image to a predefined resolution
        cv_image = cv2.resize(cv_image, (IMG_HEIGHT, IMG_WIDTH))
        # Storing the resized image
        self.latest_image = cv_image

    # Function to stop the TurtleBot
    def stop(self):
        self.cmd_vel.publish(Twist())

    # Callback function for node shutdown
    def on_shutdown(self):
        rospy.loginfo('Shutting down node...')


# Entry point of the program
if __name__ == '__main__':
    # Creating an instance of the Main class to start the TurtleBot control
    Main()
