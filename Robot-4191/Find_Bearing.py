import numpy as np
import cv2
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray, Int16, Header
from geometry_msgs.msg import PoseStamped


# Script imports
from Utils.utils import from_odometry

def locate_bearings(image):
    
        #image = cv2.imread('image_{}.jpg'.format(n), 0)
        image = cv2.rotate(image, cv2.ROTATE_180)        
        edges = cv2.Canny(image=image, threshold1=300, threshold2=600) # Canny Edge Detection
        kernel = np.ones((3,3),np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=4)
        circles1 = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20, \
                    param1=20, param2=6, minRadius=5, maxRadius=30)
        return circles1 #(x,y,radius)

class FIND_BEARING(Node):
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
        self.publisher = self.create_publisher(PoseStamped, '/Bearing/goal', 10)
        self.pub_img = self.create_publisher(Int16MultiArray, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.image_pub)
        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/robot/odom', self.listener_callback1, 10)
        self.sub_odom  # prevent unused variable warning

        # Initialisations
        self.cap = cv2.VideoCapture(0)
        
        self.servo = 0 # servo subscription
        self.servo_angle = 0 # servo value when images are taken
        self.pose = [0., 0., 0.] # odometry subscription
        self.robot_pose = [0., 0., 0.] # robot pose when images are taken
        radius = 0.1 # ----------------------radius get from alex
        self.bearing_height_img = radius * 2 

        # Constants
        pixel_width = 480
        pixel_height = 320
        x = pixel_width/2
        y = pixel_height/2
        pixel_size = pixel_width/(3.6736*10**(-3)) #meters
        f = (3.04*10**(-3))/pixel_size #pixels
        self.camera_matrix = [[f, 0, x], [0, f, y], [0, 0, 1]] 
        #self.focal_length = self.camera_matrix[0][0]
        #self.half_image_width = self.camera_matrix[0][2]
        self.true_bearing_height = 0.019 # 19mm
        self.waypoint = [0, 0]

        self.countdown = 0

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
        ret, frame = self.cap.read()
        # 2. save current odom and servo angles
        self.robot_pose = self.pose
        self.servo_angle = self.servo

        # do the camera crap
        circle = locate_bearings(frame)
        pixel_location = circle[0] #x only
        bearing_radius = circle[2]

        #in the event there are no bearings in the image
        # if no bearings found: # ToDO: fill in the variable
        #    pixel_location = False
        #    bearing_radius = False

        return pixel_location, bearing_radius
    
    def image_pub(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            msg = Int16MultiArray()
            img = np.array(frame).astype(np.int16)
            print(img.shape)
            img = img.flatten()
            msg.data = [int(i) for i in img]
            self.pub_img.publish(msg)

        # Display the message on the console
        self.get_logger().info('Publishing video frame')
        
    def listener_callback1(self, msg):
        odom = from_odometry(msg)
        self.pose = [odom['x'], odom['y'], odom['theta']]

    def listener_callback2(self, msg):
        self.servo = msg

    def waypoint_pub(self):
        # message turns to True when waypoint_reached is True
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self.waypoint
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


def main(args=None):
    rclpy.init(args=args)

    find_bearing = FIND_BEARING()
    rclpy.spin(find_bearing)

    find_bearing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
