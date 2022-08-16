import numpy as np
#import rclpy
#from rclpy.node import Node
from gpiozero import RotaryEncoder
from gpiozero import Motor
#from std_msgs.msg import *
import time

#to check:
# encoder steps are set to zero after each calculation
class Odometry:

    def __init__(self, velocity, steer_angle, delta_time):
        """
        Inputs: velocity or angle (not both)
        The code will use these inputs to tell the motors to move so that the velocity or angle change are achieved
        Outputs: x, y, theta, dx, dy, dtheta - odometry data
        It is assumed that the velocity is in the y direction - forwards.
        """

        #can use the below values within any function in the class
        self.velocity = velocity
        self.steer = steer_angle
        self.time = delta_time
        # super().__init__('encoder')
        # self.publisher_ = self.create_publisher(Float32, 'encoder_rpm', 10)
        # timer_period = 0.05  # seconds
        # self.encoder = RotaryEncoder(a=21, b=20, max_steps=10000)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.t_0 = time.time()

    def find_position(self, x_old, y_old, theta_old, step_size, left_encoder_steps, right_encoder_steps, length):

        #assuming encoder steps are negative for moving backwards
        dist_left = step_size*left_encoder_steps #step_size*encoder steps taken
        dist_right = step_size*right_encoder_steps

        x = x_old + ((dist_left + dist_right) / 2) * np.cos(theta_old)
        y = y_old + ((dist_left + dist_right) / 2) * np.sin(theta_old)
        theta = theta_old + (dist_right - dist_left) / length
        dx = (x - x_old) / self.time
        dy = (y - y_old) / self.time
        dtheta = (theta - theta_old) / self.time

        print(f"x = {x:6f}, y = {y:6f}, theta = {theta:6f}, dx = {dx:6f}, dy = {dy:6f}, dtheta = {dtheta:6f}")

        #reset encoder steps
        left_encoder_steps = 0
        right_encoder_steps = 0

        return x, y, theta, dx, dy, dtheta

    def travel(self):
        pass
        #ask the robot to move in the direction and speed given by velocity
        #MOTOR1 = LEFT (FROM FACING AWAY FROM US)
        #MOTOR2 = RIGHT
        motor1 = Motor(forward=17,backward=18) #params are the pin placements - forward and backwards TO EDIT
        motor2 = Motor(forward=4, backward=14)

        #CODE SOMETHING TO DETERMINE THE VELOCITY IN RATIO OF 0 TO 1 SO WE CAN TELL IT HOW FAST TO GO
        velocity = 0.5

        if self.velocity > 0: # move forward
            motor1.forward(speed=velocity) #speed is between 0 and 1
            motor2.forward(speed=velocity)
        else: #move backward
            motor1.backward(speed=velocity)
            motor2.backward(speed=velocity)

        if #some requirement tells the code that the motors need to stop
            motor1.stop()
            motor2.stop()




    def change_angle(self, length, wheel_radius):
        pass
        #ask the robot to rotate in the direction given by steer angle
        diff = self.steer*length
        dist_to_rotate = diff/2 #let left and right motors move an equal amount of steps to rotate
        encoder_steps_to_rotate = dist_to_rotate/(wheel_radius*np.sin(2*np.pi/48))
        if self.steer > 0: #rotate left
            motor1.backward(0.5)
            motor2.forward(0.5)

        else: #rotate right
            motor1.forward(0.5)
            motor2.backward(0.5)

        if (left_encoder_steps == encoder_steps_to_rotate) and (right_encoder_steps == encoder_steps_to_rotate):
            motor1.stop()
            motor2.stop()



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
    left_encoder_steps = 51
    right_encoder_steps = 50
    test = Odometry(velocity, steer_angle, delta_time)
    wheel_radius = 3
    length = 5 #distance between wheels
    step_size = wheel_radius * ((2 * np.pi) / 48) #48 encoder steps for the wheel
    x_old = 0
    y_old = 0
    theta_old = np.pi/2 #the first theta must be pi/2 so that y is in the forward direction.

    #test the code
    for i in range(10):
        x_old, y_old, theta_old, dx, dy, dtheta = test.find_position(x_old, y_old, theta_old, step_size, left_encoder_steps, right_encoder_steps, length)
