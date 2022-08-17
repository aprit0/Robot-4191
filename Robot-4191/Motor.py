from gpiozero import PhaseEnableMotor as Motor
import time
motor_right = Motor(23, 22)
motor_left = Motor(24, 27)
t_0 = time.time()
while (time.time() - t_0 < 2):
    # motor_left.forward(0.5)
    motor_right.backward(1)

