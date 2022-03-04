#!/usr/bin/env python2

import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from scipy import stats
from sensor_msgs.msg import LaserScan
from visualization_tools import *
from ackermann_msgs.msg import AckermannDriveStamped


class Safety_Controller():
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic","/vesc/high_level/ackermann_cmd_mux/input/nav_0")
    # SIDE = rospy.get_param("safety_controller/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DRIVING_COMMAND = rospy.get_param("/vesc/high_level/ackermann_cmd_mux/output")
    SAFETY_TOPIC = rospy.get_param("/vesc/low_level/ackermann_cmd_mux/input/safety")
    HARD_STOP_DISTANCE = 0.5
    SLOW_STOP_DISTANCE = 1.0

    def __init__(self):
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC,AckermannDriveStamped,queue_size = 50)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.drive_sub = rospy.Subscriber(self.DRIVING_COMMAND,AckermannDriveStamped)
        self.prev_error = 0
        self.prev_speed = 0
        self.prev_steering = 0

    def callback(self, data):
        ranges = data.ranges
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_change = data.angle_increment

        angles = [angle_min + i * angle_change for i in range(len(ranges))]

        # Get the front ranges/angles lists that are relevant to watch the front
        relevant_data = ranges[33:66]
        relevant_angles = angles[33:66]

        # Determine how close the closest reading is
        closest_index = relevant_data.index(min(relevant_data))
        closest_point_angle = relevant_angles[closest_index]

        targeted_ranges = np.array(
            relevant_data[max(0, closest_index - 16): min(len(relevant_data), closest_index + 16)])
        targeted_angles = np.array(
            relevant_angles[max(0, closest_index - 16): min(len(relevant_data), closest_index + 16)])

        # changes to cartesian values
        x_vals = np.cos(targeted_angles) * targeted_ranges
        y_lines = np.sin(targeted_angles) * targeted_ranges

        wall_fit = np.polyfit(x_vals, y_lines, 1)

        if (min(ranges[47:53]) > 2):
            wall_angle = math.atan2(wall_fit[0], 1.0)
            dist_to_wall = wall_fit[1] * math.cos(wall_angle)
        else:
            wall_angle = - self.SIDE * math.pi / 2.0
            dist_to_wall = 0

        hard_stop_error = dist_to_wall - self.HARD_STOP_DISTANCE
        slow_stop_error = dist_to_wall - self.SLOW_STOP_DISTANCE
        hard_dist_to_stop = self.calculate_PD(hard_stop_error)
        slow_dist_to_stop = self.calculate_PD(slow_stop_error)

        if hard_dist_to_stop <= 0:
            self.send_ackermann(0)
        elif slow_dist_to_stop <= 0:
            self.send_ackermann(.5)

    def calculate_PD(self,error):
      Kp = 1.0
      Kd = 10.0

      d_error = error - self.prev_error
      dist_to_stop = Kp * error + Kd * d_error
      self.prev_error = error
      return dist_to_stop

    def send_ackermann(self, drive_speed):
        drive_command = AckermannDriveStamped()
        drive_command.header.stamp = rospy.Time.now()
        drive_command.drive.steering_angle = self.drive_sub.drive.steering_angle
        drive_command.drive.steering_angle_velocity= self.drive_sub.drive.steering_angle_velocity
        drive_command.drive.acceleration=self.drive_sub.drive.acceleration

        if drive_speed == .5:
            drive_command.drive.speed *= self.drive_sub.drive.speed
        else:
            drive_command.drive.speed = drive_speed


        self.safety_pub.publish(drive_command)

if __name__== "__main__":
  rospy.init_node("safety_controller")
  safety_controller= Safety_Controller()
  rospy.spin()

