import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import math
from math import cos, sin
import numpy as np
import time
# import pyastar2d
from astar_python.astar import Astar
# from Lidar.LidarSrc.localmap import localmap
from Lidar.LidarSrc.LidarX2 import LidarX2
from Utils.lidar_utils import from_odometry, pad_map
from config import map_min, map_dimension, map_resolution

'''
Simultaneous (Localisation) and Mapping
Aim: Obstacle avoidance and path planning for the robot
Subscribes: 
- Goal - PoseStamed: [X, Y]
- Pose - Odometry: [X, Y, 0, 0, 0, Theta]
Publishes:
- Occupancy Grid - OccupancyGrid: [n, m]
- Path - Path: [n, 2]
Written by Aidan Pritchard
'''


class SAM(Node):

    def __init__(self):
        super().__init__('sam')
        self.sub_goal = self.create_subscription(PoseStamped,
                                                 '/goal', self.update_goal, 10)
        self.sub_pose = self.create_subscription(Odometry,
                                                 '/robot/odom', self.update_odom, 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/SAM/map', 10)
        self.pub_path = self.create_publisher(Path, '/SAM/path', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Setup Lidar
        self.lidar = LidarX2("/dev/ttyUSB0")
        while not self.lidar.open():
            print("Cannot open lidar")

        # Setup map
        self.map_dimension, self.map_resolution = map_dimension, map_resolution
        self.min_distance = map_min  # distance from the lidar to ignore
        self.morigin = self.map_dimension / 2.0
        self.map_size = int(self.map_dimension / self.map_resolution)
        self.m = np.ones((self.map_size, self.map_size), dtype = int)

        # Setup path
        self.path = Path()

        # Setup Robot
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.vel = [0.0, 0.0, 0.0]  # dx, dy, dtheta
        self.goal = [0.0, 0.0]  # x, y


    def update_goal(self, msg):
        print('update goal')
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y

    def update_odom(self, msg):
        '''
        odom['x'] = 0
        odom['y'] = 0
        odom['dx'] = 0
        odom['theta'] = 0
        odom['dtheta'] = 0
        '''
        print('update pose')
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]
        self.vel = [odom['dx'], odom['dtheta']]

    def timer_callback(self):
        t_0 = time.time()
        self.get_map()
        self.publish_map()
        print('time: map: {:.5}'.format(time.time() - t_0))
        self.get_path()

    def get_map(self, padding=True):
        # heuristic = [0.001 * self.min_distance, 0, -0.001 * self.min_distance]
        lidar_measurements = self.lidar.getMeasures()
        # Reset map for each iteration
        new_m = np.full((self.map_size, self.map_size), 0.0)
        if len(lidar_measurements) > 0:
            distances = []
            for point in lidar_measurements:
                new_dist = 0.001 * point.distance
                if self.map_dimension/2 > new_dist > self.min_distance:
                    # 2 legs forward, offset = 0, 1 leg forward, offset = 180
                    offset = -180.0 # Sets the correct orientation of lidar wrt to the robot
                    new_angle = -(offset + point.angle)* np.pi / 180
                    new_angle -= self.pose[2] # new_angle is now in world frame
                    if new_angle > np.pi:
                        new_angle -= 2 * np.pi
                    elif new_angle < np.pi:
                        new_angle += 2 * np.pi
                    
                    # convert (d, theta) -> (x, y) in world frame
                    x_robot, y_robot = new_dist*cos(new_angle) + self.pose[0], new_dist*sin(new_angle) + self.pose[1]
                    # convert robot frame to map frame
                    x_map, y_map = self.pose_to_pixel([x_robot, y_robot])
                    if self.map_size > x_map > 0 and self.map_size > y_map > 0:
                        new_m[x_map, y_map] = 100
            # Add padding to points
            self.m = pad_map(new_m, pad_val=50, null_value=100, min_blob=2)
            self.m[self.m == 0.0] = 10
            self.m[self.m == 100] = None
            print('Lidar points: ',len(distances))

    def get_path(self):
        map_arr = self.m
        # fix for translation
        print('G/P:', self.goal, self.pose)
        pixel_x, pixel_y = self.pose_to_pixel(self.goal)
        origin_x, origin_y = self.pose_to_pixel(self.pose[:2])
        print('Origins: ',origin_x, origin_y, pixel_x, pixel_y, ' || ', self.map_dimension/self.map_resolution)
        print('Map value', map_arr[pixel_x, pixel_y])
        astar = Astar(map_arr)
        waypoints = np.array(astar.run((origin_x, origin_y),(pixel_x, pixel_y)))
        print('Waypoint shape: ', waypoints.shape)
        if len(waypoints.shape) ==  2:
            self.generate_path(waypoints)
            self.pub_path.publish(self.path)
        else:
            print('Invalid path')
            return 0

    def pose_to_pixel(self, pose):
        # pose maps from - map_dimension : map_dimension
        map = lambda old_value, old_min, old_max, new_min, new_max: ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        pixel_x = map(pose[0], -self.morigin + self.pose[0], self.morigin + self.pose[0], 1, (self.map_dimension/self.map_resolution)-1)
        pixel_y = map(pose[1], -self.morigin + self.pose[1], self.morigin + self.pose[1], (self.map_dimension/self.map_resolution)-1, 1)
        return int(pixel_x), int(pixel_y)

    def pixel_to_pose(self, pixel):
        map = lambda old_value, old_min, old_max, new_max, new_min: ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        pose_x = map(pixel[0], 1, (self.map_dimension/self.map_resolution)-1, self.morigin + self.pose[0], -self.morigin + self.pose[0])
        pose_y = map(pixel[1], 1, (self.map_dimension/self.map_resolution)-1, self.morigin + self.pose[1], -self.morigin + self.pose[1])
        return pose_x, pose_y

    def generate_path(self, waypoints):
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

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.map_resolution
        msg.info.width = math.ceil(self.map_size)
        msg.info.height = math.ceil(self.map_size)
        msg.info.origin.position.x = float(-self.morigin + self.pose[0])
        msg.info.origin.position.y = float(-self.morigin + self.pose[1])
        data = self.m.reshape((self.map_size * self.map_size,))
        data[np.isnan(data)] = 100
        msg.data = [int(i) for i in data]
        self.pub_map.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ros_node = SAM()

    rclpy.spin(ros_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
