rom gpiozero import RotaryEncoder
from gpiozero import Motor
import numpy as np
import time

class Odometry:
  '''
  Aim: 
   - Calculate pose
  '''
    def __init__(self):
        # Intialise
        self.motor_left = Motor(forward=23,backward=22) 
        self.motor_right = Motor(forward=27, backward=24)
        self.encoder_left = RotaryEncoder(a=21, b=20, max_steps=10000)
        self.encoder_right = RotaryEncoder(a=5, b=6, max_steps=10000)
        
        # Params
        self.radius = 0.1
        self.dist_between_wheels = 1
        self.step_size = self.radius * ((2 * np.pi) / 48)
        
        # Variables
        self.last_time = 0
        self.pose = [0., 0., 0.] # x, y, theta
        self.twist = [0., 0., 0.] # dx, dy, dtheta
        
    
    def get_pose(self):
        x_old = self.pose[0]
        y_old = self.pose[1]
        theta_old = self.pose[2]
        
        dist_left = self.encoder_left.steps * self.step_size
        dist_right = self.encoder_right.steps * self.step_size
        x = x_old + ((dist_left + dist_right) / 2) * np.cos(theta_old)
        y = y_old + ((dist_left + dist_right) / 2) * np.sin(theta_old)
        theta = theta_old + (dist_right - dist_left) / length
        dx = (x - x_old) / self.last_time
        dy = (y - y_old) / self.last_time
        dtheta = (theta - theta_old) / self.last_time
        
        self.pose = [x, y, theta]
        self.twist = [dx, dy, dtheta]
        
        self.encoder_left.steps = 0
        self.encoder_right.steps = 0
        
        print(f"x = {x:6f}, y = {y:6f}, theta = {theta:6f}, dx = {dx:6f}, dy = {dy:6f}, dtheta = {dtheta:6f}")

        
        
if __name__ == "__main__":
  odo = Odometry()
  while(True):
    odo.get_pose()
    time.sleep(0.1)
