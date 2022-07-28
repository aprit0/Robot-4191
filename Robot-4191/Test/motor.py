
from gpiozero import Motor
from time import sleep

motor = Motor(forward=4, backward=14)
speeds = [i*0.1 for i in range(1, 10)]
while True:
    for i in speeds:
        motor.forward(i)
        print('forward', i)
        sleep(1)
        motor.backward(i)
        print('backward', i)
        sleep(1)
    
