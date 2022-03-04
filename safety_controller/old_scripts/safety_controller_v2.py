#!/usr/bin/env python2

import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from scipy import stats
from sensor_msgs.msg import LaserScan
# from visualization_tools import *
from ackermann_msgs.msg import AckermannDriveStamped
from math import pi
from std_msgs.msg import Float32


class Safety_Controller():
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    DRIVE_TOPIC = rospy.get_param("safety_controller/drive_topic","/vesc/high_level/ackermann_cmd_mux/input/nav_0")
    SIDE = rospy.get_param("safety_controller/side")
    VELOCITY = rospy.get_param("safety_controller/velocity")
    DRIVING_COMMAND = rospy.get_param("safety_controller/driving_command")
    SAFETY_TOPIC = rospy.get_param("safety_controller/safety_topic")
    HARD_STOP_DISTANCE = 0.9
    #SLOW_STOP_DISTANCE = 1.0

    def __init__(self):
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC,AckermannDriveStamped,queue_size = 50)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.drive_sub = rospy.Subscriber(self.DRIVING_COMMAND,AckermannDriveStamped)
        self.speed_pub = rospy.Publisher('/speed_data', Float32, queue_size=10)
        self.time_pub = rospy.Publisher('/time_data', Float32, queue_size=10)
        self.distance_pub = rospy.Publisher('/distance_data', Float32, queue_size=10)
        self.prev_error = 0
        self.prev_speed = 0
        self.prev_steering = 0

    def callback(self, scan):
        # Slice relevant LIDAR data
        angle_range = []
        angle_range.append(int((pi/8.0 - scan.angle_min)/scan.angle_increment)) # best: 6/8, 1/8
        angle_range.append(int((-pi/8.0 - scan.angle_min)/scan.angle_increment))
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
        wall_distance = abs(intercept)/math.sqrt(slope**2 + (-1)**2)
        print(wall_distance)
        
        self.time_pub.publish(float(rospy.get_rostime().nsecs)/(10.0**9.0))
        self.distance_pub.publish(abs(wall_distance))

        if wall_distance < 0.9:
            drive_command = AckermannDriveStamped()
            drive_command.header.stamp = rospy.Time.now()
            drive_command.drive.speed = 0.0
            self.safety_pub.publish(drive_command)
            self.speed_pub.publish(0.0)
        else:
            self.speed_pub.publish(1.0)

if __name__== "__main__":
  rospy.init_node("safety_controller")
  safety_controller= Safety_Controller()
  rospy.spin()

