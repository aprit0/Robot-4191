# General imports
import numpy as np
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry

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
        self.sub_map = self.create_subscription(Path, '/SAM/path', self.get_goal, 10)
        self.sub_map
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
        self.look_ahead = 0.2 # How far ahead to look before finding a waypoint
        self.i = 0
        
    def read_waypoints(self, msg):
        self.waypoints = []
        for waypoint in msg.poses:
            self.waypoints.append([waypoint.pose.position.x, waypoint.pose.position.y])
            
    def get_goal(self, msg):
        #goal_x, goal_y = input('Enter destination: x, y').split()
        #goal_x, goal_y = [float(goal_x),float(goal_y)]
        #testing multiple waypoints now, then waypoints will be individually found via ROS
        self.read_waypoints(msg)
        while(len(self.waypoints) > 1 and self.dist_between_points(self.pose[:2], self.waypoints[0]) < self.look_ahead):
            print('way len', len(self.waypoints))
            self.waypoints.pop(0)    

        self.goal = self.waypoints[0]

    def main(self):
        '''
        Aim: take in waypoint, travel to waypoint
        '''
        #while True:
        
        angle_to_rotate = self.calculate_angle_from_goal()
        print(self.pose[0], self.pose[1], math.degrees(self.pose[2]))
        print('Dist2Goal: {:.3f} || Ang2Goal: {:.3f}'.format(self.dist_between_points(self.pose[:2], self.goal),math.degrees(angle_to_rotate)))
        if self.dist_between_points(self.pose[:2], self.goal) > self.dist_from_goal:
            # Check if we need to rotate or drive straight
            if (self.state['Turn'] == 0 and abs(angle_to_rotate) > self.max_angle) or self.state['Turn'] == 1:
                # Drive curvy
                self.state['Turn'] = 1
                #self.drive(direction = 0)
                self.drive(direction=np.sign(angle_to_rotate))
                if abs(angle_to_rotate) < self.min_angle:
                    self.state['Turn'] = 0
            elif self.state['Turn'] == 0:
                # Drive straight
                self.drive(direction=0)
            else:
                print('BOI YO DRIVING BE SHITE', self.state['Turn'], angle_to_rotate)
        else:
            # Waypoint reached
            if len(self.waypoints) == 1 or len(self.waypoints) == 0:
                # Destination reached
                print('Goal achieved')
                self.drive(0, 0)  # Stops robot
            else:
                #look for next waypoint
                self.waypoints.pop(0)
                self.goal = self.waypoints[0]

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

    def drive(self, direction=0, value=1):
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
