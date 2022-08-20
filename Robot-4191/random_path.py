from Utils.utils import Motor
from Velocity_Ctrl import Velocity_Ctrl
from Odometry_1 import Odometry
import time
import numpy as np

#random code to generate a path from A to B where steer angle corrects asynchronous motors

odo = Odometry()
motor_right = Motor(22, 23)
motor_left = Motor(27, 24)
ctrl = Velocity_Ctrl(motor_left,motor_right)

x_final = 0.10
y_final = 0.50
vel = 1
odo.get_pose()
[x_current,y_current,theta_init] = odo.pose #read from odometry code
#print initial location
print(odo.pose) # theta should be pi/2

#find direct path to final point

theta = round(np.arctan2(y_final,x_final),4)
print(theta)
#add to while loop if recalculating angle to final location - currently they are constant

theta1 =round(theta - np.pi/8,4) #right angle bound
theta2 =round(theta + np.pi/8,4) #left angle bound
if (theta_init > theta):
    #turn right by theta_init-theta
    direction = -1
    ctrl.steer_angle_infinite(direction) #-ve turns right
    theta_current = theta_init
    while (theta_current > theta + np.pi/16):
       odo.get_pose()
       time.sleep(0.1)
       [x_current,y_current,theta_current]=odo.pose
    ctrl.stop()
elif (theta_init < theta):
    direction = 1
    #turn left
    ctrl.steer_angle_infinite(direction)
    theta_current = theta_init
    while (theta_current < theta - np.pi/16):
       odo.get_pose()
       time.sleep(0.1)
       [x_current,y_current,theta_current] = odo.pose
print(odo.pose)
theta=theta_current
print(theta)
#must ensure thetassssssssssssssssss and x,y are calculated from odometry code - must accessssssssssssssssssssssssssssssssssss
while (x_current < x_final or y_current < y_final): 
    print("Entered loop")
    #can change vel if the robot is getting closer to target

    #robot must continue travelling until goal is reached
    ctrl.vel_infinite(vel)
    while (theta < theta2 and theta > theta1 and (x_current < x_final or y_current < y_final)):
        odo.get_pose()
        time.sleep(0.1)
        [x_current,y_current,theta] = odo.pose
    ctrl.stop()
    print(odo.pose)
    if (theta > theta2): #has turned too far left
        ctrl.steer_angle_infinite(-1)
        while (theta > theta1 + np.pi/16):
            #ctrl.steer_angle_infinite()
            odo.get_pose()
            time.sleep(0.1)
            [x_current,y_current,theta] = odo.pose
        ctrl.stop()
    elif (theta < theta1- np.pi/2): #has turned too far right
        ctrl.steer_angle_infinite(1)
        while (theta < theta2):
            #ctrl.steer_angle_infinite()
            odo.get_pose()
            time.sleep(0.1)
            [x_current, y_current, theta] = odo.pose
        ctrl.stop()
    print(odo.pose)
#print final location
print(odo.pose)

    #theta_current = theta
