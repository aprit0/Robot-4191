#notsorry

from gpiozero import AngularServo, Servo
from time import sleep
import numpy as np
 
servo = AngularServo(13, min_angle=-90, max_angle=90)
# servo=Servo(13)
ang = 45
while True:
    ang = int(input('angle, [-90, 90]'))
    ang = np.interp(ang, [-90, 55], [-90, 90])
    # centre point = 35 degrees, positive is CCW, negative CW

    servo.angle = int(ang)
 
#while True:
#    servo.angle = -90
#    sleep(2)
#    servo.angle = -45
 #   sleep(2)
#    servo.angle = 0
#    sleep(2)
#    servo.angle = 45
#    sleep(2)
#    servo.angle = 90
#    sleep(2)
#servo = Servo(13)
#while True:
#    servo.mid()
#    sleep(0.5)
#    servo.min()
#    sleep(1)
#    servo.mid()
#    sleep(0.5)
#    servo.max()
#    sleep(1)


#from gpiozero import Servo
#from gpiozero.tools import sin_values
#from signal import pause
    
#servo = Servo(13)
#servo.source = sin_values()
#servo.source_delay = 0.1
#pause()
