import numpy as np
import cv2
import time
import math
# Ros imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray, Int16, Header, Bool
from geometry_msgs.msg import PoseStamped
from gpiozero import AngularServo


# Script imports
from Utils.utils import from_odometry

# def locate_bearings(image):
    
#         #image = cv2.imread('image_{}.jpg'.format(n), 0)
#         image = cv2.rotate(image, cv2.ROTATE_180)        
#         edges = cv2.Canny(image=image, threshold1=300, threshold2=750) # Canny Edge Detection
#         kernel = np.ones((3,3),np.uint8)
#         edges = cv2.dilate(edges, kernel, iterations=4)
#         circles1 = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20, \
#                     param1=20, param2=6, minRadius=5, maxRadius=30)
#         return circles1 #(x,y,radius)

def locate_bearings(image):
    import cv2
    import numpy as np
    bearing_locations = []
    #image = cv2.imread('image_{}.jpg'.format(n), 0)
    image = cv2.rotate(image, cv2.ROTATE_180)        
    edges = cv2.Canny(image=image, threshold1=300, threshold2=750) # Canny Edge Detection
    edges2 = cv2.Canny(image=image, threshold1=200, threshold2=90)

    kernel = np.ones((3,3),np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=4)
    edges2 = cv2.dilate(edges2, kernel, iterations=1)

    circles1 = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20, \
                    param1=20, param2=6, minRadius=5, maxRadius=30)
    circles2 = cv2.HoughCircles(edges2, cv2.HOUGH_GRADIENT, 1, 26, \
                        param1=100, param2=8.5, minRadius=5, maxRadius=15)

    if circles2 is not None and circles1 is not None:
        for c1 in circles1[0,:]:
            for c2 in circles2[0,:]:
                dist = np.sqrt(pow(c1[0]-c2[0], 2) + pow(c1[1]-c2[1], 2))
                if dist < (c1[2]+c2[2])/2:
                    cv2.circle(image,(c2[0],c2[1]),c2[2],(255,0,255), 2)
                    bearing_locations.append(c2)
    #cv2.imshow('Canny Edge Detection', edges)
    #cv2.imshow('Image', image)
    if (cv2.waitKey(0) & 0xFF) == ord('q'):
        cv2.destroyAllWindows()
    return bearing_locations #, image

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

        # Publish the goal
        self.publisher = self.create_publisher(PoseStamped, '/Bearing/goal', 10)
        self.pub_img = self.create_publisher(Int16MultiArray, 'video_frames', 10)
        #self.timer = self.create_timer(0.001, self.image_pub)
        #run main
        self.timer = self.create_timer(0.1, self.main)
        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/robot/odom', self.listener_callback1, 10)
        self.sub_odom  # prevent unused variable warning
        self.sub_goal_reached = self.create_subscription(Bool, '/Controller/msg', self.listener_callback2, 10)
        self.sub_goal_reached

        # Initialisations
        self.cap = cv2.VideoCapture(0)
        
        self.servo = AngularServo(13, min_angle=-90, max_angle=90)
        self.servo_angle = 0  # servo value when images are taken
        #self.servo_control()
        self.pose = [0., 0., 0.] # odometry subscription
        self.robot_pose = [0., 0., 0.] # robot pose when images are taken
        self.goal_reached = True

        # Constants
        pixel_width = 480
        pixel_height = 320
        x = pixel_width/2
        y = pixel_height/2
        pixel_size = (3.6736*10**(-3))/pixel_width #meters
        f = (3.04*10**(-3))/pixel_size #pixels
        self.camera_matrix = [[f, 0, x], [0, f, y], [0, 0, 1]] 
        self.focal_length = self.camera_matrix[0][0]
        self.half_image_width = self.camera_matrix[0][2]
        self.true_bearing_height = 0.019 # 19mm
        self.goal = [0, 0]
    
        #self.countdown = 0

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

        # do the camera crap
        circles = locate_bearings(frame)
        print(circles)
        try:
            circles = circles[0]
        except:
            circles = []
        #in the event there are no bearings in the image
        if len(circles) == 0:
            pixel_location = False
            bearing_radius = False
        else:
            circle = circles[0] # double array

            pixel_location = circle[0] #x only
            bearing_radius = circle[2]


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
        print('I heard: ',msg)
        if msg:
            self.goal_reached = True

    def goal_pub(self):
        # message turns to True when waypoint_reached is True
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        self.publisher.publish(msg)

    def servo_control(self):
        ang = np.interp(self.servo_angle, [-90, 55], [-90, 90])  
        self.servo.angle = ang

    def servo_search(self):
        print('-------------------------------')
        servo_max = 90 #55
        servo_min = -90
        servo_step = 30 # degree
        old_ang = self.servo_angle
        print(old_ang)
        if old_ang == servo_max:
            old_ang = servo_min
        else:
            old_ang += servo_step
        old_ang = old_ang if old_ang < servo_max else servo_max
        self.servo_angle = old_ang
        self.servo_control() 

    def main(self):
        """
        Aim: take a photo of the surroundings, output the location of a bearing in the image
        """
        #only run if the goal is reached
        if not self.goal_reached:
            print('False')
            return 0
        print('Searching')
        self.image_pub()
        # find the pixel location and size
        pixel_location, bearing_radius = self.camera()
        # break out of function if there are no bearings in the image
        if not pixel_location:
            #self.servo_search()
            return 0
        #else:
        #    self.servo_angle = 0
        #    self.servo_control()
        bearing_height_img = 2*bearing_radius

        # find the pixel angle wrt center of camera image
        pixel_angle = np.arctan((self.half_image_width - pixel_location) / self.focal_length)  # if negative, to the right of center, else to the left

        # total angle wrt the world
        angle_world = self.robot_pose[2] + self.servo_angle + pixel_angle
        # check the sign for servo_bearing, could be negative

        dist_from_robot = (self.focal_length * self.true_bearing_height) / bearing_height_img
        x = dist_from_robot * np.cos(angle_world) + self.robot_pose[0]
        y = dist_from_robot * np.sin(angle_world) + self.robot_pose[1]

        bearing_pose = [x, y]
        self.goal = bearing_pose
        print('goal',self.goal)

        # publish the waypoint
        self.goal_pub()
        
        #don't run main again until the next goal is reached
        self.goal_reached = False


def main(args=None):
    rclpy.init(args=args)

    find_bearing = FIND_BEARING()
    rclpy.spin(find_bearing)

    find_bearing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
