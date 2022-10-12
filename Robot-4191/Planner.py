import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool, Int32MultiArray
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import math
from math import cos, sin
import numpy as np
import time
import pyastar2d
from astar_python.astar import Astar
# from Lidar.LidarSrc.localmap import localmap
from Lidar.LidarSrc.LidarX2 import LidarX2
from Utils.lidar_utils import from_odometry, pad_map
from config import map_min, map_dimension, map_resolution



class SAM(Node):
    def __init__(self):
        super().__init__('plan')
        self.pub_path = self.create_publisher(Path, '/SAM/path', 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/sam/goal', self.update_goal, 10)
        self.sub_pose = self.create_subscription(Odometry, '/robot/odom', self.update_odom, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/SAM/map', self.sub_map, 10)
        self.goal = [None, None]
        self.pose = [None, None]
        self.map_dimension, self.map_resolution = map_dimension, map_resolution

        self.dist_2_goal = _points = lambda pose, goal: abs(math.dist(pose, goal))
        self.m = None


    def update_goal(self, msg):
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y
        print('update goal', self.goal)
        if type(self.m) != type(None):
            self.path_callback()  # pub path



    def path_callback(self):
        if self.pose[0] != None:
            dist = self.dist_2_goal(self.pose[:2], self.goal)
            print('================', dist)
            if dist > 0.1:
                print('++++++++++==update_path')
                self.get_path()

    def sub_map(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_size = msg.info.width
        self.morigin = self.pose[0] - msg.info.origin.position.x
        data = np.array(msg.data)
        self.m = data.reshape((self.map_size, self.map_size))
        print('Map update')


    def generate_path(self, waypoints):
        print('creating new path')
        new_path = Path()
        new_path.header.frame_id = 'map'
        for i in range(waypoints.shape[0]):
            way = self.pixel_to_pose(waypoints[i])
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(way[0])
            pose_stamped.pose.position.y = float(way[1])
            pose_stamped.pose.position.z = float(0.0)

            new_path.poses.append(pose_stamped)
        self.path = new_path

    def pose_to_pixel(self, pose):
        # pose maps from - map_dimension : map_dimension
        map = lambda old_value, old_min, old_max, new_min, new_max: ((old_value - old_min) / (old_max - old_min)) * (
                    new_max - new_min) + new_min
        pixel_x = map(pose[0], self.morigin + self.pose[0], -self.morigin + self.pose[0],
                      (self.map_dimension / self.map_resolution) - 1, 0)
        pixel_y = map(pose[1], self.morigin + self.pose[1], -self.morigin + self.pose[1],
                      (self.map_dimension / self.map_resolution) - 1, 0)
        return int(pixel_x), int(pixel_y)

    def pixel_to_pose(self, pixel):
        map = lambda old_value, old_min, old_max, new_max, new_min: ((old_value - old_min) / (old_max - old_min)) * (
                    new_max - new_min) + new_min
        pose_x = map(pixel[0], 0, (self.map_dimension / self.map_resolution) - 1, self.morigin + self.pose[0],
                     -self.morigin + self.pose[0])
        pose_y = map(pixel[1], 0, (self.map_dimension / self.map_resolution) - 1, self.morigin + self.pose[1],
                     -self.morigin + self.pose[1])
        return pose_x, pose_y

    def get_path(self):
        # fix for translation
        print('G/P:', self.goal, self.pose)
        body_offset = 0.1
        pixel_x, pixel_y = self.pose_to_pixel([self.goal[0], self.goal[1]])  # + body_offset])
        origin_x, origin_y = self.pose_to_pixel([self.pose[0], self.pose[1]])  # + body_offset])
        print('Origins: ', origin_x, origin_y, pixel_x, pixel_y, ' || ', self.map_dimension / self.map_resolution)
        self.m[pixel_x - 1: pixel_x + 1, pixel_y - 1: pixel_y + 1] = int(5)
        self.m[origin_x, origin_y] = int(5)
        map_arr = self.m
        print('Map value', map_arr[pixel_x, pixel_y])
        print('pose value: ', map_arr[origin_x, origin_y])
        astar = Astar(map_arr)
        waypoints = np.array(astar.run((origin_x, origin_y), (pixel_x, pixel_y)))
        # waypoints = np.array(pyastar2d.astar_path(np.float32(map_arr),(pixel_x, pixel_y),(origin_x, origin_y) ))#, allow_diagonal=True), dtype=np.float32)

        print('Waypoint shape: ', waypoints.shape)
        if len(waypoints.shape) == 2:
            self.generate_path(waypoints)
            self.pub_path.publish(self.path)
        else:
            print('Invalid path')
            return 0

    def update_odom(self, msg):
        '''
        odom['x'] = 0
        odom['y'] = 0
        odom['dx'] = 0
        odom['theta'] = 0
        odom['dtheta'] = 0
        '''
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]
        self.vel = [odom['dx'], odom['dtheta']]
        self.t_1 = time.time()


def main(args=None):
    rclpy.init(args=args)
    ros_node = SAM()
    rclpy.spin(ros_node)

    rclpy.shutdown()
    rclpy.spin(ros_node)


if __name__ == '__main__':
    main()
