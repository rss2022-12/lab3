#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

DRIVE_TOPIC=rospy.get_param("safety_controller/drive_topic","/vesc/ackermann_cmd_mux/input/navigation")
SCAN_TOPIC=rospy.get_param("safety_controller/scan_topic")
VELOCITY=0.7

def go_fwd():
    rospy.init_node('go_fwd_publisher', anonymous=True)
    fwd_pub= rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped,  queue_size =50)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        drive_command = AckermannDriveStamped()
        drive_command.header.stamp = rospy.Time.now()
        drive_command.drive.speed=VELOCITY
        fwd_pub.publish(drive_command)
        rate.sleep()



if __name__=="__main__":
    try:
        go_fwd()
    except rospy.ROSInterruptException:
        pass
