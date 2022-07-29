#!/usr/bin/env python
import rospy
import roslib
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf
import math
from math import sin, cos, pi,tan, atan2
import numpy as np
from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt
from scipy import interpolate

from localmap import localmap



#***********************************************************************    
def mappublisher(m,height, width, resolution,morigin):
    msg = OccupancyGrid()
    msg.header.frame_id='map'
    msg.info.resolution = resolution
    msg.info.width      = math.ceil(width/resolution)
    msg.info.height     = math.ceil(height/resolution)
    msg.info.origin.position.x=-morigin[0]
    msg.info.origin.position.y=-morigin[1]
    msg.data=m  
    mappub.publish(msg)



 class Laser(Node):

    def __init__(self):
        super().__init__('laser')
        self.publisher_ = self.create_publisher(LaserScan, '/laser/scan', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar = LidarX2("/dev/ttyUSB0")
        self.i = 0
        if not self.lidar.open():
            print("Cannot open lidar")
        self.height, self.width, self.resolution=10,10,0.05
        self.morigin=[width/2.0,height/2.0]
        self.m=localmap(height, width, resolution,morigin)
        self.pose=[0.0,0.0,0.0]

    def timer_callback(self):
        lidar_measurements = self.lidar.getMeasures()
        if len(lidar_measurements) > 0:
            distances = []
            angles = []
            for point in lidar_measurements:
                angles.append(point.angle)
                distances.append(0.001 * point.distance)
            self.m.updatemap(distances,
                             0,                     # Min angle
                             3.1415,                # Max angle
                             3.1415/len(distances), # Angle increment
                             0.01,                  # Min dist
                             20,                    # Max dist
                             self.pose)
            
            msg = OccupancyGrid()
            msg.header.frame_id='map'
            msg.info.resolution = self.resolution
            msg.info.width      = math.ceil(self.width/self.resolution)
            msg.info.height     = math.ceil(self.height/self.resolution)
            msg.info.origin.position.x=-self.morigin[0]
            msg.info.origin.position.y=-self.morigin[1]
            msg.data=self.m  
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    laser_node = Laser()

    rclpy.spin(laser_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
