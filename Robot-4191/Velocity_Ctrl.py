from Utils.utils import Motor
import time
'''
TODO:
-test the velocity at different speeds from 0 to 1 and enter results in excel so the actual velocity can be calculated
-test steer angle as well so that we can convert a time to an angle
'''
class Velocity_Ctrl:

    def __init__(self,motor_left,motor_right):
        self.motor_left=motor_left
        self.motor_right=motor_right

    def velocity(self,vel):

        # convert the velocity (0-1) to an actual velocity after testing

        t_0 = time.time()
        while (time.time() - t_0 < 1.5): #time to drive can be an input if needed
            if (vel>0):
                self.motor_left.forward(vel)
                self.motor_right.forward(vel)
            else: #vel<0 (backwards)
                self.motor_left.backward(abs(vel))
                self.motor_right.backward(abs(vel))
        self.motor_left.stop()
        self.motor_right.stop()

    def vel_infinite(self, vel):

        if (vel > 0):
            self.motor_left.forward(vel)
            self.motor_right.forward(vel)
        else:  # vel<0 (backwards)
            self.motor_left.backward(abs(vel))
            self.motor_right.backward(abs(vel))


    def steer_angle(self,steer):

        #convert the steer - currently a time to an angle after testing

        t_0 = time.time()
        while (time.time() - t_0 < abs(steer)): #the speed of turning must remain constant, while the time allowed to turn changes for different angles
            if (steer > 0): #turn left
                self.motor_left.backward(1)
                self.motor_right.forward(1)
            else: #turn right
                self.motor_left.forward(1)
                self.motor_right.backward(1)
        self.motor_left.stop()
        self.motor_right.stop()

    def steer_angle_infinite(self, steer):

        if (steer > 0):  # turn left
            self.motor_left.backward(1)
            self.motor_right.forward(1)
        else:  # turn right
            self.motor_left.forward(1)
            self.motor_right.backward(1)


    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

if __name__ == "__main__":

    motor_right = Motor(22, 23)
    motor_left = Motor(27, 24)
    ctrl = Velocity_Ctrl(motor_left,motor_right)

    state = True
    while (state == True):
        #ask user if they want to travel straight or turn
        ans0 = 0
        while(ans0 != '1' and ans0 != '2'):
            ans0 = input("Would you like to travel straight or turn? \n1: straight\n2: turn\n")

        #ask user to input a velocity or steer angle
        if (ans0 == '1'):
            #ask for velocity
            vel = 5 #an invalid input to enter loop

            while (vel > 1  or vel < -1):
                vel = input("Enter a velocity (-1 to 1)\n")
                vel = float(vel)

            ctrl.velocity(vel)

        elif (ans0 == '2'):
            #ask for a steer angle
            steer = input("Please enter a steer angle in degrees:\n")

            ctrl.steer_angle(float(steer))

        #ask user if they would like to continue
        ans1 = input("Would you like to continue? [y/n]\n")

        if (ans1 == 'n' or ans1 == 'N'):
            state = False



