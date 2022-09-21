#notsorry

from gpiozero import AngularServo, Servo
from time import sleep
 
servo=Servo(26)
val=-1
while True:
    servo.value = val
    sleep(0.1)
    val = val -2*val
    #if val>1:
    #    val=-1
 
#servo = AngularServo(13, min_pulse_width = 0.00006, max_pulse_width=0.00023)
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
