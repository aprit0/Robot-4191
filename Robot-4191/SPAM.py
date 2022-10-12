from rclpy.node import Node
import rclpy
from std_msgs.msg import Int16, Bool, Int32MultiArray, Header
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
        self.pub_goal = self.create_publisher(PoseStamped,
                                                 '/sam/goal', 10)
        self.pub_turn = self.create_publisher(Int32MultiArray, '/SAM/turn', 10)
        self.sub_goal_reached = self.create_subscription(Bool, '/Controller/msg', self.listener_callback2, 10)
        self.sub_goal_reached
        timer_period = 0.1  # seconds
        # self.timer_map = self.create_timer(timer_period, self.timer_callback)
        # self.timer_path = self.create_timer(0.3, self.path_callback)

        # Setup Lidar
        self.lidar = LidarX2("/dev/ttyUSB0")
        while not self.lidar.open():
            print("Cannot open lidar")

        # Setup map
        self.map_dimension, self.map_resolution = map_dimension, map_resolution
        self.min_distance = 0.05#map_min  # distance from the lidar to ignore
        self.morigin = self.map_dimension / 2.0
        self.map_size = int(self.map_dimension / self.map_resolution)
        self.m = np.ones((self.map_size, self.map_size), dtype = int)

        # Setup path
        self.path = Path()

        # Setup Robot
        self.pose = [0.001, -0.15, np.pi/2]  # x, y, theta
        self.vel = [0.0, 0.0, 0.0]  # dx, dy, dtheta
        self.goal = [None, None]  # x, y
        self.loops = False # Single run only
        
        #Functions
        self.dist_2_goal = _points = lambda pose, goal: abs(math.dist(pose, goal))

        # Parameters
        self.turn = [0, 0, 0, 0, 0] #0, FR, BR, BL, FL: Count of obstacles within body length in range
        self.goal_reached = True

    def goal_pub(self):
        # message turns to True when waypoint_reached is True
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        self.pub_goal.publish(msg)
    def update_goal(self, msg):
        print('Update Goal:', self.goal, msg.pose.position.x, self.goal_reached)
        if float(msg.pose.position.x) == float(100):
            # No goal, start search
            self.turn = self.turn[1:]
            # calculate next goal using self.turn
            m_vals = {self.turn[i]:i for i in range(len(self.turn))}  
            direction = m_vals[min(m_vals.keys())]
            print('*Hacker voice, Im in', self.goal, self.turn)
            time.sleep(0.5)
            #0 : FL 
            #1 : FR 
            #2 : BR 
            #3 : 
            # BL:np.pi - np.pi/4
            # BR: np.pi + np.pi/4
            # FR: - np.pi/4
            # FL: np.pi/4
            new_ang = 0
            dist_from_robot = 0.25
            if direction == 0: 
                new_ang =  np.pi/4 # back right?
            elif direction == 1:
                new_ang = - np.pi/4
            elif direction == 2: # behind
                new_ang = np.pi + np.pi/4
            else:
                new_ang = np.pi - np.pi/4  # back left?
            angle_world = self.pose[2] + new_ang
            x = dist_from_robot * np.cos(angle_world) + self.pose[0]
            y = dist_from_robot * np.sin(angle_world) + self.pose[1]
            #if self.goal == [None, None]: # goal was reset(ie achieved)
            self.goal = [x,y]

            print(f'NO GOAL -- -\ndirection: {direction}\ngoal: {self.goal}\nquads: {self.turn}') 
        else:
            self.goal[0] = msg.pose.position.x
            self.goal[1] = msg.pose.position.y
            print('update goal', self.goal)
        self.timer_callback()

    def listener_callback2(self, msg):
        print('RESET GOAL')
        self.goal = [None, None]
        self.goal_reached = True

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

    def timer_callback(self):#, msg):
        #if msg and self.loops == False: #only run if msg == True
        t_0 = time.time()
        self.get_map()
        self.publish_map()
        print('time: map: {:.5}'.format(time.time() - t_0))
        self.loops = True
            

    def get_map(self, padding=True):
        # heuristic = [0.001 * self.min_distance, 0, -0.001 * self.min_distance]
        lidar_measurements = self.lidar.getMeasures()
        # Reset map for each iteration
        new_m = np.full((self.map_size, self.map_size), 0.0)
        quadrants = [0, 0, 0, 0, 0]
        if len(lidar_measurements) > 0:
            distances = []
            for point in lidar_measurements:
                new_dist = 0.001 * point.distance
                if new_dist > self.min_distance:
                    if new_dist < 0.5:
                        if point.angle < 90:
                            quadrants[3] += 1
                        elif point.angle < 180:
                            quadrants[2] += 1
                        elif point.angle < 270:
                            quadrants[1] += 1
                        else:
                            quadrants[4] += 1
                    # 2 legs forward, offset = 0, 1 leg forward, offset = 180
                    offset = 90 # Sets the correct orientation of lidar wrt to the robot
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
            map_value = 5
            pad_value = 1
            self.m = new_m#pad_map(new_m, map_value=map_value, pad_value=pad_value, null_value=100, min_blob=3)
            self.m[self.m == 0.0] = map_value
            self.m[self.m == 100] = None
            # Quadrant process
            print('Quadrants: ', quadrants)
            self.turn = quadrants
            


    def pose_to_pixel(self, pose):
        # pose maps from - map_dimension : map_dimension
        map = lambda old_value, old_min, old_max, new_min, new_max: ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        pixel_x = map(pose[0], self.morigin + self.pose[0], -self.morigin + self.pose[0], (self.map_dimension/self.map_resolution)-1, 0)
        pixel_y = map(pose[1], self.morigin + self.pose[1], -self.morigin + self.pose[1], (self.map_dimension/self.map_resolution)-1, 0)
        return int(pixel_x), int(pixel_y)

    def pixel_to_pose(self, pixel):
        map = lambda old_value, old_min, old_max, new_max, new_min: ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        pose_x = map(pixel[0], 0, (self.map_dimension/self.map_resolution)-1, self.morigin + self.pose[0], -self.morigin + self.pose[0])
        pose_y = map(pixel[1], 0, (self.map_dimension/self.map_resolution)-1, self.morigin + self.pose[1], -self.morigin + self.pose[1])
        return pose_x, pose_y

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
        print('GOAL------', self.goal)
        if self.goal[0] != None:
            print('GOAL------', self.goal)
            self.goal_pub()

    def goal_pub(self):
        # message turns to True when waypoint_reached is True
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        self.pub_goal.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    ros_node = SAM()
    rclpy.spin(ros_node)

    rclpy.shutdown()
    rclpy.spin(ros_node)


if __name__ == '__main__':
    main()
