#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class Fwd:
  DRIVE_TOPIC=rospy.get_param("wall_follower/drive_topic","/vesc/ackermann_cmd_mux/input/nav_1")
  SCAN_TOPIC=rospy.get_param("wall_follower/scan_topic")
  VELOCITY=1

  def __init__(self):
    self.fwd_pub= rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped,  queue_size =50)
    self.sub = rospy.Suscriber(self.SCAN_TOPIC, LaserScan, self.send_fwd)


  def send_fwd(self,data):
    drive_command = AckermannDriveStamped()
    drive_command.header.stamp = rospy.Time.now()
    drive_command.drive.speed=self.VELOCITY

    self.fwd_pub.publish(drive_command)



if __name__=="__main__":
  rospy.init_node("fwd_goer")
  go_f=Fwd()
  rospy.spin()
 



