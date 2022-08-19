from Utils.utils import Motor
import time
'''
TODO:
-test the velocity at different speeds from 0 to 1 and enter results in excel so the actual velocity can be calculated
-test steer angle as well so that we can convert a time to an angle
'''


def velocity(vel):

    # convert the velocity (0-1) to an actual velocity after testing

    t_0 = time.time()
    while (time.time() - t_0 < 3): #time to drive can be an input if needed
        print(time.time()-t_0)
        motor_left.forward(vel)
        motor_right.forward(vel)
    motor_left.stop()
    motor_right.stop()


def steer_angle(steer):

    # convert the steer - currently a time to an angle after testing

    t_0 = time.time()
    while (time.time() - t_0 < abs(steer)): #the speed of turning must remain constant, while the time allowed to turn changes for different angles
        if (steer > 0): #turn left
            motor_left.backward(1)
            motor_right.forward(1)
        else: #turn right
            motor_left.forward(1)
            motor_right.backward(1)

if __name__ == "__main__":

    motor_right = Motor(22, 23)
    motor_left = Motor(27, 24)

    state = True
    while (state == True):
        #ask user if they want to travel straight or turn
        ans0 = input("Would you like to travel straight or turn? \n1: straight\n2: turn\n")

        #ask user to input a velocity or steer angle
        if (ans0 == '1'):
            #ask for velocity
            vel = input("Enter a velocity (0 to 1):\n")

            #error check
            while (vel > '1' or vel < '0'):
                vel = input("Enter a velocity (0 to 1)\n")

            velocity(float(vel))

        elif (ans0 == '2'):
            #ask for a steer angle
            steer = input("Please enter a steer angle in degrees:\n")

            steer_angle(float(steer))

        #ask user if they would like to continue
        ans1 = input("Would you like to continue? [y/n]\n")

        if (ans1 == 'n' or 'N'):
            state = False


