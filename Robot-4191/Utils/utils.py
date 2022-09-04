from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math
from scipy.ndimage import label
import numpy as np
from time import sleep

from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as LED


class Motor:
    def __init__(self, pin_pwm, pin_led):
        self.pwm = PWM(pin_pwm)
        self.led = LED(pin_led)
        self.delay = 0.0001

    def forward(self, value=1):
        self.pwm.value = value
        self.led.on()
        self.pwm.on()
        sleep(self.delay)

    def backward(self, value=1):
        self.pwm.value = value
        self.led.off()
        self.pwm.on()
        sleep(self.delay)

    def stop(self):
        self.pwm.off()
        sleep(self.delay)


def to_odometry(odom):
    '''
    Sample:
    odom = {}
    odom['x'] = 0
    odom['y'] = 0
    odom['dy'] = 0
    odom['theta'] = 0
    odom['dtheta'] = 0
    '''
    quaternion = Quaternion()
    quaternion.z = math.sin(odom['theta'] / 2.0)
    quaternion.w = math.cos(odom['theta'] / 2.0)
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = odom['x']
    msg.pose.pose.position.y = odom['y']
    msg.pose.pose.orientation = quaternion
    msg.child_frame_id = "base_link"
    msg.twist.twist.linear.x = odom['dx']
    msg.twist.twist.angular.z = odom['dtheta']
    return msg


def from_odometry(msg):
    odom = {}
    odom['x'] = msg.pose.pose.position.x
    odom['y'] = msg.pose.pose.position.y
    quat = msg.pose.pose.orientation
    euler = math.atan2(2.0 * (quat.w * quat.z),
                       1.0 - 2.0 * (quat.z ** 2))
    odom['theta'] = euler
    odom['dx'] = msg.twist.twist.linear.x
    odom['dtheta'] = msg.twist.twist.angular.z
    return odom


def pad_map(arr, pad_val=80, null_value=1, min_blob=2):
    size = arr.shape[0]
    heuristic = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    structure = np.array([
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0]])
    labeled, n_blob = label(arr, structure)
    print('N_blob', n_blob)
    # only pad large blobs
    for i in range(1, n_blob + 1):
        mask = np.ma.where(i == labeled)
        mask = list(zip(mask[0], mask[1]))
        if len(mask) > min_blob:
            for point in mask:
                # pad out using heuristic
                for j in heuristic:
                    new_index = np.add(point, j)
                    if (new_index[0] < size and new_index[1] < size) and (
                            arr[new_index[0], new_index[1]] != null_value):
                        # pad
                        arr[new_index[0], new_index[1]] = pad_val
    return arr
