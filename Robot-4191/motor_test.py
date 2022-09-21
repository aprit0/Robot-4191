from Utils.utils import Motor

motor_right = Motor(22,23)
motor_left = Motor(27,24)


while True:
    motor_left.forward(1)
    motor_right.forward(1)

