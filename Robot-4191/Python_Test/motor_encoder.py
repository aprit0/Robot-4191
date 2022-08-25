
from gpiozero import Motor, RotaryEncoder
import time
motor = Motor(forward=4, backward=14, pwm=1)
encoder = RotaryEncoder(a=21, b=20, max_steps=100000)
SPEED = 0.5 # in percentage
TIME = 0.1
while True:
    for i in [j * 0.1 for j in range(10)]:
        t_0 = time.time()
        while time.time() - t_0 < TIME:
            motor.forward(i)
        steps = encoder.steps
        t_1 = time.time() - t_0
        rpm = (steps / 48 / t_1 ) * 60
        print('PWM %: ', i, ' RPM: ', rpm)
        encoder.steps = 0
