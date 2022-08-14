import numpy as np
#import rclpy
#from rclpy.node import Node
#from gpiozero import RotaryEncoder
#from std_msgs.msg import *
#import time

#to check:
# encoder steps are set to zero after each calculation
# encoder steps and velocity are dependent on each other sooooooo
class Odometry:

    def __init__(self, velocity, steer_angle, left_encoder_steps, right_encoder_steps, delta_time): #delete left and right encoder steps for ROS implementation
        """
        Inputs: velocity or angle (not both)
        The code will use these inputs to tell the motors to move so that the velocity or angle change are achieved
        Outputs: x, y, theta, dx, dy, dtheta - odometry data
        It is assumed that the velocity is in the y direction - forwards.
        """

        #can use the below values within any function in the class
        self.velocity = velocity
        self.steer = steer_angle
        self.left = left_encoder_steps
        self.right = right_encoder_steps
        self.time = delta_time
        # super().__init__('encoder')
        # self.publisher_ = self.create_publisher(Float32, 'encoder_rpm', 10)
        # timer_period = 0.05  # seconds
        # self.encoder = RotaryEncoder(a=21, b=20, max_steps=10000)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.t_0 = time.time()

    def find_position(self, x_old, y_old, theta_old, step_size):
        #take the previous position as an input as well as time elapsed
        if self.velocity != 0:
            dist = step_size*self.right #step_size*encoder steps taken
            theta = theta_old #theta_old remains the same as there is no rotation
            dtheta = 0 #no change in angle
            x = x_old + dist*np.cos(theta)
            y = y_old + dist*np.sin(theta)
            dx = (x - x_old)/self.time #change in distance over time
            dy = (y - y_old)/self.time

        else: #self.steer !=0
            x = x_old
            y = y_old
            dx = 0 #no change in location
            dy = 0
            theta = theta_old + self.steer #assuming the steer angle is wrt the previous angle
            dtheta = (theta - theta_old)/self.time

        print(f"x = {x:2f}, y = {y:2f}, theta = {theta:2f}, dx = {dx:2f}, dy = {dy:2f}, dtheta = {dtheta:2f}")

        return x, y, theta, dx, dy, dtheta

    def travel(self):
        pass#inputs =
        #ask the robot to move

    def change_angle(self):
        pass#inputs =
        #ask the robot to rotate


    # def timer_callback(self):
    #     msg = Float32()
    #     t_delta = time.time() - self.t_0
    #     print('t_delta ', t_delta)
    #     rpm = (self.encoder.steps / 48 / t_delta) * 60 #multiplied by 60 to get revs/min as opposed to revs/sec
    #     msg.data = rpm
    #     self.publisher_.publish(msg)
    #     self.encoder.steps = 0
    #     self.t_0 = time.time()


# def main(args=None):
#     rclpy.init(args=args)
#
#     encoder_node = Encoder()
#
#     rclpy.spin(encoder_node)
#
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    #main()
    #stuff to test the code
    velocity = 0.1 # between 0 and 1 (1 = max velocity of motor)
    steer_angle = 0 # rad
    delta_time = 3 # seconds
    left_encoder_steps = 50 #dependent on velocity need to deal with thissssssssssssssss
    right_encoder_steps = 50 #dependent on velocity need to deal with thissssssssssssssss
    test = Odometry(velocity, steer_angle, left_encoder_steps, right_encoder_steps, delta_time)
    wheel_radius = 3
    step_size = wheel_radius * ((2 * np.pi) / 48) #48 encoder steps for the wheel
    x_old = 0
    y_old = 0
    theta_old = np.pi/2 #the first theta must be pi/2 so that y is in the forward direction.
    test.find_position(x_old, y_old, theta_old, step_size)
