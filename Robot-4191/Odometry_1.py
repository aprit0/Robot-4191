from gpiozero import RotaryEncoder
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
        self.encoder_right = RotaryEncoder(a=20, b=21, max_steps=10000)
        self.encoder_left = RotaryEncoder(a=5, b=6, max_steps=10000)
        
        # Params
        self.radius = 0.05468 * 0.5
        self.dist_b_wheels = 0.2208
        self.step_size =2.5/2* 2 * self.radius * np.pi / (11*90.895) 
        
        # Variables
        self.last_time = 0
        self.pose =[round(0.,2),round(0.,2), round(np.pi/2.,4)] # x, y, theta
        self.twist =[0., 0., 0.] # dx, dy, dtheta
        self.last_time = time.time()
        self.counter = 0
        

    def get_pose(self):#delta_time):
        x_old = self.pose[0]
        y_old = self.pose[1]
        theta_old = self.pose[2]
        self.counter += self.encoder_left.steps
        dist_left = self.encoder_left.steps * self.step_size
        dist_right = self.encoder_right.steps * self.step_size

        delta_time = time.time() - self.last_time
        x = x_old + ((dist_left + dist_right) / 2) * np.cos(theta_old)
        y = y_old + ((dist_left + dist_right) / 2) * np.sin(theta_old)
        theta = theta_old + (dist_right - dist_left) / self.dist_b_wheels
        dx = (x - x_old) / delta_time
        dy = (y - y_old) / delta_time
        dtheta = (theta - theta_old) / delta_time

        #update pose and twist
        self.pose = [round(x,2),round(y,2), round(theta,4)]
        self.twist = [dx, dy, dtheta]
        
        self.encoder_left.steps = 0
        self.encoder_right.steps = 0
        #print(self.counter)
        #print(f"x = {x:.6f}, y = {y:.6f}, theta = {theta:.6f}, dx = {dx:.6f}, dy = {dy:.6f}, dtheta = {dtheta:.6f}")
        self.last_time = time.time()
        
        
if __name__ == "__main__":
  odo = Odometry()
  delta_time = 0
  t_0 = time.time()
  while(True):
    delta_time = time.time() - t_0
    odo.get_pose() #delta_time)
    t_0 = time.time()
    time.sleep(0.1)

