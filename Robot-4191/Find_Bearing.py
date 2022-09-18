import numpy as np
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int16

# Script imports
from Utils.utils import from_odometry

class FIND_BEARING:
    """
    This class is used to find the location of a bearing with respect to the world.
    Inputs:
    - Odometry of the robot
    - Servo angle
    - Potentially if the waypoint is reached or something that tells the script to find the next bearing. It  could also be timed

    Outputs:
    - Publish the waypoint calculated (bearing_pose = [x,y])

    Constants:
    - Camera image size (width most importantly)
    - Camera intrinsics
    """

    def __init__(self):
        super().__init__('find_bearing')

        # Publish the waypoint
        self.publisher = self.create_publisher(Float32MultiArray, '/Bearing/msg', 10)

        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/robot/odom', self.listener_callback1, 10)
        self.sub_odom  # prevent unused variable warning
        # servo sub - must add servo code to subscribe to :))))
        self.sub_servo = self.create_subscription(Servo_Angle, '/robot/servo_angle', self.listener_callback2, 10)
        self.sub_servo  # prevent unused variable warning

        # Initialisations
        self.servo = 0 # servo subscription
        self.servo_angle = 0 # servo value when images are taken
        self.pose = [0., 0., 0.] # odometry subscription
        self.robot_pose = [0., 0., 0.] # robot pose when images are taken
        self.bearing_height_img = radius * 2 # radius get from alex

        # Constants
        self.camera_matrix = [[], [], []] # ToDo: Fill it in
        self.focal_length = self.camera_matrix[0][0]
        self.half_image_width = self.camera_matrix[0][2]
        self.true_bearing_height = 0.019 # 19mm
        self.waypoint = [0, 0]

        self.countdown = 0

        # take photo every 5 seconds # ToDo: change to take photos when we want it to - timed just for now
        if time.time() - self.countdown > 5:
            main()

    def camera(self):
        # Alex's code to take a photo and find the bearing in the image
        # ToDo: for a scenario with multiple bearings in the image scattered - for calculation purposes,
        #  pick the easiest bearing to reach
        # Check the servo angle, if looking straight ahead, pick the bearing most central
        # if looking left, pick the rightmost bearing to minimise turning required
        # if looking right, pick the leftmost bearing
        # alternatively, if there are multiple close bearings, could pick the center one to aim for and we try to
        # collect all in one drive
        # not sure which method is best as I dont know how wide the camera view is, but pls output values
        # for a single bearing

        # 1. take photo

        # 2. save current odom and servo angles
        self.robot_pose = self.pose
        self.servo_angle = self.servo

        # do the camera crap

        # outputs: pixel_location = center x position where 0 is the left side of the image to match my calcs
        # bearing_radius = pretty self explanatory

        #in the event there are no bearings in the image
        if no bearings found: # ToDO: fill in the variable
            pixel_location = False
            bearing_radius = False

        return pixel_location, bearing_radius

    def listener_callback1(self, msg):
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]

    def listener_callback2(self, msg):
        self.servo = msg

    def waypoint_pub(self):
        # message turns to True when waypoint_reached is True
        msg = Float32MultiArray()
        msg.data = self.waypoint
        self.publisher.publish(msg)

    def main(self):
        """
        Aim: take a photo of the surroundings, output the location of a bearing in the image
        """

        # find the pixel location and size
        pixel_location, bearing_radius = camera()

        # break out of function if there are no bearings in the image
        if not pixel_location:
            return

        bearing_height_img = 2*bearing_radius

        # find the pixel angle wrt center of camera image
        pixel_angle = np.arctan((self.half_image_width - pixel_location) / self.focal_length)  # if negative, to the right of center, else to the left

        # total angle wrt the world
        angle_world = self.robot_bearing + self.servo_angle + pixel_angle
        # check the sign for servo_bearing, could be negative

        dist_from_robot = (self.focal_length * self.true_bearing_height) / bearing_height_img
        x = dist_from_robot * np.cos(angle_world) + self.robot_pose[0]
        y = dist_from_robot * np.sin(angle_world) + self.robot_pose[1]

        bearing_pose = [x, y]
        self.waypoint = bearing_pose

        # publish the waypoint
        self.waypoint_pub()

        # reset the count
        self.countdown = time.time()


def main(args=None):
    rclpy.init(args=args)

    find_bearing = FIND_BEARING()
    rclpy.spin(find_bearing)

    find_bearing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
