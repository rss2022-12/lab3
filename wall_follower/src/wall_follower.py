#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
import math
from math import pi
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic","/vesc/ackermann_cmd_mux/input/navigation")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"

    def __init__(self):
        # Subscribe to scans and publish to DRIVE_TOPIC
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.adjust_drive)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.Kp = 7.0
        self.Ki = 0.086
        self.Kd = 0.094
        self.last_time = rospy.get_rostime()
        self.error = [0.0, 0.0]
        self.integral = 0.0
        self.last_steering_angle = 0.0

        #a publisher for our line marker
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        
        # publishers for plots
        self.time_data = rospy.Publisher('/time', Float32, queue_size=10)
        self.actual_data = rospy.Publisher('/actual', Float32, queue_size=10)
        self.desired_data = rospy.Publisher('/desired', Float32, queue_size=10)
        self.error_data = rospy.Publisher('/error', Float32, queue_size=10)

    def adjust_drive(self, scan):
        # Slice relevant LIDAR data
        angle_range = []
        angle_range.append(int((self.SIDE*4.0*pi/8.0 - scan.angle_min)/scan.angle_increment)) # best: 6/8, 1/8
        angle_range.append(int((self.SIDE*-1.0*pi/8.0 - scan.angle_min)/scan.angle_increment))
        range_min, range_max = min(angle_range), max(angle_range)

        thetas = scan.angle_min + np.array(list(range(range_min, range_max)))*scan.angle_increment
        rs = np.array(scan.ranges)[range_min:range_max]
        thetas = thetas[np.where(rs < np.mean(rs)+1.0)]
        rs = rs[np.where(rs < np.mean(rs)+1.0)]

        x = rs*np.cos(thetas)
        y = rs*np.sin(thetas)

        # Find regression line and get current distance
        slope = np.sum((x - np.mean(x))*(y - np.mean(y)))/np.sum((x - np.mean(x))**2)
        intercept = np.mean(y) - slope*np.mean(x)
        y_reg = intercept + slope*x
        look_ahead_t = 0.2
        curr_x = (look_ahead_t*self.VELOCITY)*np.cos(0.0)
        curr_y = (look_ahead_t*self.VELOCITY)*np.sin(0.0)
        wall_distance = abs(slope*curr_x - 1*curr_y + intercept)/math.sqrt(slope**2 + (-1)**2)

        # Update error terms
        curr_time = rospy.get_rostime()
        dt = curr_time.nsecs/(10.0**9) - self.last_time.nsecs/(10.0**9)
        self.last_time = curr_time
        self.error[1] = self.error[0]
        self.error[0] = wall_distance - self.DESIRED_DISTANCE
        self.integral = self.integral + self.error[0]*dt
        derivative = (self.error[0] - self.error[1])/dt

        # Initialize drive_msg
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.header.stamp = rospy.Time.now()

        # Set drive_msg speed parameters
        drive_msg.drive.speed = self.VELOCITY

        # Set drive_msg turning parameters
        drive_msg.drive.steering_angle = self.SIDE*(self.Kp*self.error[0] + self.Ki*self.integral + self.Kd*derivative)
        self.last_steering_angle = drive_msg.drive.steering_angle

        # Visusalize wall estimate
        VisualizationTools.plot_line(x, y_reg, self.line_pub, frame="/laser")
        
        # Publish drive message
        self.pub.publish(drive_msg)

        # Publish plot messages
        self.time_data.publish(float(rospy.get_rostime().nsecs)/(10.0**9.0))
        self.desired_data.publish(self.DESIRED_DISTANCE)
        self.actual_data.publish(wall_distance)
        self.error_data.publish(self.error[0])

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
