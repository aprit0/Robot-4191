#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
# import tf
import math
#from pylab import *
#from itertools import groupby
#from operator import itemgetter
#import matplotlib.pyplot as plt
#from scipy import interpolate

from LidarSrc.localmap import localmap
from LidarSrc.LidarX2 import LidarX2

'''
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
'''


class Laser(Node):

    def __init__(self):
        super().__init__('laser')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map/occupancy', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar = LidarX2("/dev/ttyUSB0")
        self.i = 0
        if not self.lidar.open():
            print("Cannot open lidar")
        self.height, self.width, self.resolution=10,10,0.05
        self.min_distance = 0.05 # distance from the lidar to ignore
        self.morigin=[self.width/2.0,self.height/2.0]
        self.m=localmap(self.height, self.width, self.resolution,self.morigin)
        self.pose=[0.0,0.0,0.0]

    def timer_callback(self):
        lidar_measurements = self.lidar.getMeasures()
        if len(lidar_measurements) > 0:
            distances = []
            angles = []
            for point in lidar_measurements:
                dist = point.distance
                if dist > self.min_distance:
                    angles.append(point.angle * 3.1415 / 180)
                    distances.append(0.001 * point.distance)
            print(len(distances))
            self.m.updatemap(list(zip(angles,distances)),
                             0.01,                  # Min dist
                             20,                    # Max dist
                             self.pose)
            data = list(self.m.localmap)
            #print(self.m.localmap)
            print(type(self.m.localmap))
            msg = OccupancyGrid()
            msg.header.frame_id='map'
            msg.info.resolution = self.resolution
            msg.info.width      = math.ceil(self.width/self.resolution)
            msg.info.height     = math.ceil(self.height/self.resolution)
            msg.info.origin.position.x=-self.morigin[0]
            msg.info.origin.position.y=-self.morigin[1]
            msg.data = [int(i) for i in data]  
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
