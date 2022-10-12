from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math
from scipy.ndimage import label
import numpy as np

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
    msg.child_frame_id = "map"
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

def euclidian(input):
    [x, y] = input
    return ((x**2)+(y**2))**0.5

def pad_map(arr, map_value = 0, pad_value=20, null_value=1, min_blob=2):
    pad_val = pad_value
    pixel_pad = 7 # number of pixels to pad
    size = arr.shape[0]
    heuristic = [[1,0], [-1, 0]]#[[1, 0], [0, 1], [-1, 0], [0, -1]] #+ [[1,1], [1,-1], [-1,1], [-1,-1]]
    h_0 = heuristic
    for i in range(1, pixel_pad):
        h_0.extend([[j[0] + np.sign(j[0]), j[1] + np.sign(j[1])] for j in heuristic])
    h_0.extend([[0,-1], [0,1], [0, 2], [0, -2], [0, 3], [0, -3]])#, [1,1], [-1, -1], [0, -4], [0, 4], [0, 5], [0, -5], [-2, -2], [2, 2], [-3, -3], [4, 4], [-4, -4], [-5, -5], [5, 5], [-6, -6]])

    # print(h_0)
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
                arr[point[0] - pad_value:point[0] + pad_value, point[1] - pad_value:point[1] + pad_value] = int(null_value)
        else:
            for point in mask:
                arr[point] = map_value
    return arr
