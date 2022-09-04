# General imports
from gpiozero import RotaryEncoder
import numpy as np
import time
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# Script imports
from Utils.utils import to_odometry


class ODOM(Node):
    '''
    Aim: 
    - Calculate pose
    Publishes:
    /robot/odom: Odometry
    '''

    def __init__(self):
        # Ros intialisation
        super().__init__('odometry')
        self.publisher_ = self.create_publisher(Odometry, '/robot/odom', 10)
        timer_period = 0.1  # 0.05 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Intialise
        self.encoder_right = RotaryEncoder(a=20, b=21, max_steps=10000)
        self.encoder_left = RotaryEncoder(a=5, b=6, max_steps=10000)

        # Params
        self.radius = 0.05468 * 0.5
        self.dist_b_wheels = 0.2208 - 0.002# 0.05:- 0.0063 # Magic number for yaw
        self.encoder_steps = 12
        self.offset = 0.013333333#*1.0189 # gear offset likely 0.013
        self.step_theta = 2 * np.pi / self.encoder_steps # degrees motor has rotated
        self.step_dist = self.radius * self.offset * self.step_theta
       # self.step_dist = 2* self.radius*np.pi/self.encoder_steps
      #  self.step_dist = 2.5 / 2 * 2 * self.radius * np.pi / (11 * 90.895) # Depreciated but for reference

        # Variables
        self.last_time = 0
        self.wheel_left = 0
        self.wheel_right = 0
        self.pose = [0., 0., np.pi / 2.]  # x, y, theta
        self.twist = [0., 0., 0.]  # dx, dy, dtheta
        self.last_time = time.time()
        self.counter = 0
        self.t_0 = time.time()

    def timer_callback(self):
        self.get_pose()
        odom = {'x': self.pose[0],
                'y': self.pose[1],
                'dx': self.twist[0],
                'dy': self.twist[1],
                'theta': self.pose[2],
                'dtheta': self.twist[2]}
        msg = to_odometry(odom)
        self.publisher_.publish(msg)
        if time.time() - self.t_0 > 0.1:
            print('X, Y, Th', self.pose)
            #print('Steps', self.encoder_left.steps)
            self.t_0 = time.time()

    def get_pose(self):
        x_old = self.pose[0]
        y_old = self.pose[1]
        theta_old = self.pose[2]
        dist_left = self.encoder_left.steps * 0.0002 # * self.step_dist
        dist_right = self.encoder_right.steps * 0.0002 # * self.step_dist
        delta_time = time.time() - self.last_time

        theta = theta_old + (dist_right - dist_left) / ( self.dist_b_wheels)
        if theta > np.pi:
            theta -= 2*np.pi
        elif theta < -np.pi:
            theta += 2*np.pi
        x = x_old + ((dist_left + dist_right) / 2) * np.cos(theta)
        y = y_old + ((dist_left + dist_right) / 2) * np.sin(theta)
        dx = (x - x_old) / delta_time
        dy = (y - y_old) / delta_time
        dtheta = (theta - theta_old) / delta_time
        # update pose and twist
        self.pose = [x, y, theta]
        self.twist = [dx, dy, dtheta]

        self.encoder_left.steps = 0
        self.encoder_right.steps = 0
        self.last_time = time.time()
        self.wheel_left += dist_left
        self.wheel_right += dist_right


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ODOM()
    rclpy.spin(minimal_publisher)
   # minimal_subscriber = CONTROLLER()
   # rclpy.spin(minimal_subscriber)

   # minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
