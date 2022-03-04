#!/usr/bin/env python2

import numpy as np

#HELPFUL: https://answers.ros.org/question/265660/how-to-create-ackermanndrivestamped-messages-in-python/
import rospy
import math
from rospy.numpy_msg import numpy_msg
from scipy import stats
from sensor_msgs.msg import LaserScan
from visualization_tools import *
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollower():
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"

    def __init__(self):
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=50)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.prev_error = 0


    def callback(self, data):
        data_ranges = data.ranges
        wall_fit = self.create_marker(data)
        steering_angle = self.PD(data_ranges,wall_fit)
        self.send_ackermann(steering_angle)

    def create_marker(self,data):

        ranges = data.ranges
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_change = data.angle_increment

        angles = [angle_min + i*angle_change for i in range(len(ranges))]

        # Get the half of the ranges/angles lists that are relevant for desired side to follow
        relevant_data = ranges[0:50] if self.SIDE == -1 else ranges[50:100]
        relevant_angles = angles[0:50] if self.SIDE == -1 else angles[50:100]

        # Determine how close the closest reading is
        closest_index = relevant_data.index(min(relevant_data))
        closest_point_angle = relevant_angles[closest_index]

        targeted_ranges = np.array(relevant_data[max(0, closest_index - 16): min(50, closest_index + 16)])
        targeted_angles = np.array(relevant_angles[max(0, closest_index - 16): min(50, closest_index + 16)])

        x_vals = np.cos(targeted_angles) * targeted_ranges
        y_lines = np.sin(targeted_angles) * targeted_ranges

        wall_fit = np.polyfit(x_vals, y_lines, 1)
        return wall_fit

    def PD(self,data_ranges, wall_fit):
        # fit_wpv = np.polyfit(x_vals, y_lines, 1, full=1)

        if (min(data_ranges[47:53]) > 2):
            wall_angle = math.atan2(wall_fit[0], 1.0)
            dist_to_wall = wall_fit[1] * math.cos(wall_angle)
        else:
            wall_angle = - self.SIDE * math.pi / 2.0
            dist_to_wall = 0
        rospy.loginfo("wall:%s, dist:%s",wall_angle,dist_to_wall)

        error = dist_to_wall - self.SIDE * self.DESIRED_DISTANCE
        error += wall_angle * 0.5

        Kp = 1.0
        Kd = 10.0
        d_error = error - self.prev_error
        steering_angle = Kp*error + Kd*d_error

        steering_angle = min(max(steering_angle, -math.pi / 2), math.pi / 2)
        steering_angle += wall_angle
        self.prev_error = error
        rospy.loginfo("angel:%s",steering_angle)
        return steering_angle

    def send_ackermann(self,steering_angle):
        response = AckermannDriveStamped()
        response.header.stamp = rospy.Time.now()
        response.drive.speed = self.VELOCITY
        response.drive.steering_angle = steering_angle
        self.pub.publish(response)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
