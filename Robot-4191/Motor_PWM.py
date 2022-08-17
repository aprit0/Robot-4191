from gpiozero import PhaseEnableMotor as Motor
from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as LED
import time
#motor_right = Motor(23, 22)
#motor_left = Motor(24, 27)
pwm = PWM(27)
led = LED(24)
t_0 = time.time()
led.on()
pwm.value = 1  
while (time.time() - t_0 < 2):
    pwm.on()
    # motor_left.forward(0.5)
    # motor_right.backward(1)

