# General imports
import numpy as np
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# Script imports
from Utils.utils import from_odometry
from Utils.utils import Motor


class CONTROLLER(Node):
    '''
    Aim:
    - Travel to waypoints by getting pose and path and driving motors
    Subscribes:
    - /robot/odom: Odometry
    - (Path): Added later
    Publishes:
    - ... Nothing?
    '''

    def __init__(self):
        super().__init__('controller')
        # Instantiate objects
        self.sub_odom = self.create_subscription(Odometry, '/robot/odom', self.listener_callback, 10)
        self.sub_odom  # prevent unused variable warning

        self.motor_right = Motor(22, 23)
        self.motor_left = Motor(27, 24)

        # Functions
        self.dist_between_points = lambda pose, goal: abs(math.dist(pose, goal))
        self.angle_between_points = lambda pose, goal: np.arctan2(goal[1] - pose[1], goal[0] - pose[0])

        # Variables
        self.pose = [0., 0., 0.]  # x, y, theta
        self.goal = [0., 0.]  # x, y
        self.state = {'Turn': 0}

        # Params
        self.dist_from_goal = 0.05
        self.max_angle = np.pi / 10  # Maximum offset angle from goal before correction
        self.min_angle = np.pi / 14  # Maximum offset angle from goal after correction

        self.main()

    def main(self):
        '''
        Aim: take in waypoint, travel to waypoint
        '''
        while True:
            self.drive(0, 0)  # Stops robot
            goal_x, goal_y = input('Enter destination: x, y')
            self.goal = [goal_x, goal_y]
            while self.dist_between_points(self.pose[:2], self.goal) > self.dist_from_goal:
                angle_to_rotate = self.calculate_angle_from_goal()
                print('Dist2Goal: {:.3f} || Ang2Goal: {:.3f}'.format(self.dist_between_points(self.pose[:2], self.goal),
                                                                     math.degrees(angle_to_rotate)))
                # Check if we need to rotate or drive straight
                if (self.state['Turn'] == 0 and abs(angle_to_rotate) > self.max_angle) or self.state['Turn'] == 1:
                    # Drive curvy
                    self.state['Turn'] = 1
                    self.drive(direction=np.sign(angle_to_rotate))
                    if abs(angle_to_rotate) < self.min_angle:
                        self.state['Turn'] = 0
                elif self.state['Turn'] == 0:
                    # Drive straight
                    self.drive(direction=0)
                else:
                    print('BOI YO DRIVING BE SHITE', self.state['Turn'], angle_to_rotate)

    def listener_callback(self, msg):
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]

    def calculate_angle_from_goal(self):
        angle_to_rotate = self.angle_between_points(self.pose, self.goal) - self.pose[2]
        # Ensures minimum rotation
        if angle_to_rotate < -np.pi:
            angle_to_rotate += 2 * np.pi
        if angle_to_rotate > np.pi:
            angle_to_rotate -= 2 * np.pi
        return angle_to_rotate

    def drive(self, direction=0, value=1):
        if value == 0:
            # Stop the robot
            self.motor_left.stop()
            self.motor_right.stop()
        elif direction == 1:
            # Turn left
            self.motor_left.backward(value)
            self.motor_right.forward(value)
        elif direction == -1:
            # Turn right
            self.motor_left.forward(value)
            self.motor_right.backward(value)
        elif direction == 0:
            # Drive forwards
            self.motor_left.forward(value)
            self.motor_right.forward(value)
        else:
            print('BOI YO TURNING IS SHITE', direction)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CONTROLLER()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
