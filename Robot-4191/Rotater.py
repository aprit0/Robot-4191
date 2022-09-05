# General imports
import numpy as np
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped 

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
        self.sub_goal = self.create_subscription(PoseStamped,
                                                                 '/goal', self.get_loops, 10)
        self.motor_right = Motor(22, 23)
        self.motor_left = Motor(27, 24)

        # Functions
        self.dist_between_points = lambda pose, goal: abs(math.dist(pose, goal))
        self.angle_between_points = lambda pose, goal: np.arctan2(goal[1] - pose[1], goal[0] - pose[0])

        # Variables
        self.pose = [0., 0., 0.]  # x, y, theta
        self.goal = [0., 0.]  # x, y
        self.state = {'Turn': 0}
        #self.waypoints = [[0.2, 0.2], [0.2 ,0.45], [0.40, 0.45], [0.2, 0.2], [0.4, 0.2], [0.3, 0.45], [0.3, 0], [0, 0]]
        self.waypoints = []

        # Params
        self.dist_from_goal = 0.05
        self.max_angle = np.pi / 36  # Maximum offset angle from goal before correction
        self.min_angle = np.pi / 18  # Maximum offset angle from goal after correction
        self.look_ahead = 0.3 # How far ahead to look before finding a waypoint
        self.counter = 0
        
    def get_loops(self, msg):
        print(self.pose)
        self.goal = [1, 1]
        self.drive(self.goal[1])
        time.sleep(1)
        self.t_0 = time.time()

    def main(self):
        '''
        Aim: take in waypoint, travel to waypoint
        '''
        error = 5 * 0.017
        zero = 90 * (np.pi/180)

        if zero - error > self.pose[2] or self.pose[2] >  zero + error:
            #print(time.time() - self.t_0,self.pose[2], zero)
            self.t_0 = time.time()
            self.drive(self.goal[1])
        else:
            # Destination reached
            print('Goal achieved', self.pose[2])
            self.drive(0, 0)  # Stops robot
            time.sleep(0.5)

    def listener_callback(self, msg):
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]
        self.main()

    def calculate_angle_from_goal(self):
        angle_to_rotate = self.angle_between_points(self.pose, self.goal) - self.pose[2]
        # Ensures minimum rotation
        if angle_to_rotate < -np.pi:
            angle_to_rotate += 2 * np.pi
        if angle_to_rotate > np.pi:
            angle_to_rotate -= 2 * np.pi
        return angle_to_rotate

    def drive(self, direction=0, value=0.05):
        
        if value == 0:
            # Stop the robot
            self.motor_left.stop()
            self.motor_right.stop()
        elif direction == 1:
            # Turn right
            self.motor_left.backward(value)
            self.motor_right.forward(value)
        elif direction == -1:
            # Turn left
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

    controller = CONTROLLER()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
