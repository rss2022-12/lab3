#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from math import sin,cos
from visualization_tools import *


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic","vesc/ackermann_cmd_mux/input/navigation")
    SIDE = rospy.get_param("wall_follower/side")
    
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"

    def __init__(self):
        # TODO:
        #a publisher for steering 
        self.steer_pub = rospy.Publisher(self.DRIVE_TOPIC,AckermannDriveStamped, queue_size=1)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        #a subscriber to get the laserscan data
        rospy.Subscriber(self.SCAN_TOPIC,LaserScan, self.laser_callback)

    
    def laser_callback(self,data):
        steering=AckermannDriveStamped()
        steering.drive.speed=self.VELOCITY
        steering.drive.steering_angle=self.get_steering_angle(data)
        self.steer_pub.publish(steering)

    
    def get_steering_angle(self,data):
        self.SIDE = rospy.get_param("wall_follower/side")
        ranges=data.ranges
        thetas=np.arange(data.angle_min,data.angle_max,data.angle_increment)
        
        
        x=np.array([ranges[i]*cos(thetas[i]) for i in range(len(ranges))])
        y=np.array([ranges[i]*sin(thetas[i]) for i in range(len(ranges))])
        
  
       #regression depending on side         
        
        if self.SIDE==-1:
            xnew=x[0:int(len(x)/2)+15]
            ynew=y[0:int(len(x)/2)+15]
            arr=ynew<2.8*self.DESIRED_DISTANCE
            # print(arr)
                    
            regression=np.polyfit(xnew[arr],ynew[arr],deg=1)
            m=regression[0]
            b=regression[1]
            VisualizationTools.plot_line(xnew[arr], m*xnew[arr]+b, self.line_pub, frame="/laser")
        
        else:
            xnew=x[int(len(x)/2)-8:-5]
            ynew=y[int(len(x)/2)-8:-5]
            arr=ynew<2.3*self.DESIRED_DISTANCE
            
                    
            regression=np.polyfit(xnew[arr],ynew[arr],deg=1)
            m=regression[0]
            b=regression[1]
            VisualizationTools.plot_line(xnew[arr], m*xnew[arr]+b, self.line_pub, frame="/laser")
        
     #distance between wall and car
        distance=b/np.sqrt((m**2+1))
        
     #error signs and additional control parameter
        if self.SIDE==-1:
           error=self.DESIRED_DISTANCE-np.abs(distance)
           if distance<0.5:
               newcontrol=0.05
           else:
               newcontrol=0
        else:
           error=-self.DESIRED_DISTANCE+np.abs(distance)
           if distance<0.5:
               newcontrol=-0.05
           else:
               newcontrol=0
               
        #PD constants depending on speed
        if self.VELOCITY>2:
            P=3
            D=30
        else:
            P=4
            D=1
        
        try:
            error_pre
        
        except NameError:
            error_pre=0
        
        scantime=1.0/20.0
        d_error=(error-error_pre)/scantime
        P_control=P*error
        D_control=D*d_error
                
        error_pre=error
        
        return (P_control+D_control+newcontrol)

        
        

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
