from Utils.utils import Motor
import Velocity_Ctrl
import Odometry_1
import time


#random code to generate a path from A to B where steer angle corrects asynchronous motors

ctrl = Velocity_Ctrl()
odo = Odometry()
motor_right = Motor(22, 23)
motor_left = Motor(27, 24)

x_current = 0
y_current = 0
x_final = 0
y_final = 0.7
vel = 1
odo.get_pose()
theta_current = odo.pose[2] #read from odometry code
#print initial location
print(odo.pose) # theta should be pi/2
print(theta_current)
#add to while loop if recalculating angle to final location - currently they are constant
theta = theta_current #currently unsure where to calculate theta
theta1 = theta - np.pi/8 #right angle bound
theta2 = theta + np.pi/8 #left angle bound

#must ensure thetassssssssssssssssss and x,y are calculated from odometry code - must accessssssssssssssssssssssssssssssssssss
while (x_current != x_final and y_current != y_final):

    #can change vel if the robot is getting closer to target

    #robot must continue travelling until goal is reached
    ctrl.velocity_infinite(vel)
    while (theta < theta2 and theta > theta1 and x_current != x_final and y_current != y_final):
        odo.get_pose()
        [x_current,y_current,theta] = odo.pose
    ctrl.stop()

    if (theta > theta2): #has turned too far left
        while (theta > theta1)
            ctrl.steer_angle_infinite()
            [x_current,y_current,theta] = odo.pose
        ctrl.stop()
    elif (theta < theta1): #has turned too far left
        while (theta < theta2)
            ctrl.steer_angle_infinite()
            [x_current, y_current, theta] = odo.pose
        ctrl.stop()

#print final location
print(odo.pose)

    #theta_current = theta