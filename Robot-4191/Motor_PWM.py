from Utils.utils import Motor
import time
motor_right = Motor(22, 23)
motor_left = Motor(27, 24)
t_0 = time.time()
while (time.time() - t_0 < 3):
    motor_left.forward(1)
    motor_right.forward(1)
#t_1 = time.time()
#while (time.time() - t_1 < 2.5):
 #   motor_left.backward(0.5)
  #  motor_right.forward(0.5)
#t_2 = time.time()
#while (time.time() - t_2 < 2.5):
 #   motor_left.forward(0.5)
 #   motor_right.backward(0.5)
#t_3 = time.time()
#while (time.time() - t_3 < 1.5):
 #   motor_left.backward(0.5)

