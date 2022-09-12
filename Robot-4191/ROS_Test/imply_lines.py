import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import math


def imply_lines(im, show=False):
    '''
    Aim: Draw Hough lines only as the boundary of the map
    Receives map as type np.array
    Returns map as type np.array
    '''
    map = np.array(im.copy())
    # Preprocess
    map[map != 100] = 0
    map[map == 100] = 1
    # Get all possible lines
    lines = cv2.HoughLines(map, rho=1, theta=np.pi / 180, threshold=6)
    strong_lines = get_strong_lines(map, lines)
    inter_lines = get_intersecting_lines(strong_lines)
    perp_lines = get_perpendicular_lines(inter_lines)
    print('inter_lines:', inter_lines)
    print('perp_lines', perp_lines)
    for rho, theta in perp_lines:
        _, map = check_map(rho, theta, map, draw=True)
    map1 = np.clip(map + im, 0, 100)
    if show:
        fig, ax = plt.subplots(1, 3)
        ax[0].imshow(im)
        ax[1].imshow(map)
        ax[2].imshow(map1)
        plt.show()
    return map1

def get_perpendicular_lines(lines):
    euc = lambda x0, y0, x1, y1: ((y1 - y0) ** 2 + (x1 - x0) ** 2) ** 0.5
    inverse = []  # in case of correct lines but inverse selection
    perp_lines = []
    for i in lines:
        inverse.append(i)
        if len(lines[i]) == 2:
            [x0, y0], [x1, y1] = lines[i][0][:2], lines[i][1][:2]
            out = euc(x0, y0, x1, y1)
            if np.isclose(out, 47, atol=5):
                perp_lines.append(i)
        elif len(lines[i]) > 2:
            for j in range(len(lines[i]) - 1):
                for k in range(1, len(lines[i])):
                    [x0, y0], [x1, y1] = lines[i][j][:2], lines[i][k][:2]
                    out = euc(x0, y0, x1, y1)
                    if np.isclose(out, 47, atol=5):
                        perp_lines.append(i)
    # Invert selection
    if len(perp_lines) == 2 and 4 >= len(inverse) >= 3:
        perp_inverse = []
        for i in inverse:
            if i not in perp_lines:
                perp_inverse.append(i)
        return perp_inverse
    else:
        return perp_lines


def get_intersecting_lines(strong_lines):
    lines = {}
    for i in range(strong_lines.shape[0]):
        for j in range(1, strong_lines.shape[0]):
            ln0 = list(strong_lines[i][0])
            ln1 = list(strong_lines[j][0])
            if ln0 == ln1 or ln0 == [0, 0] or ln1 == [0, 0]:
                pass
            else:
                coord = intersection(ln0, ln1)
                print('inter coord', coord)
                if coord != [] and 80 > coord[0] > 0 and 80 > coord[1] > 0:
                    lines = add_2_dict(lines, tuple(ln0), coord)
                    lines = add_2_dict(lines, tuple(ln1), coord)
    return lines


def get_strong_lines(map, lines, max_lines=10):
    strong_lines = np.zeros([max_lines, 1, 2])
    n2 = 0  # count how many strong lines added
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            # theta -= np.pi if rho < 0 else 0
            rho *= -1 if rho < 0 else 1
            if i == 0:
                strong, _ = check_map(rho, theta, map, draw=False)
                if strong:
                    strong_lines[n2] = strong
                    n2 += 1
                else:
                    pass
            else:
                closeness_rho = np.isclose(rho, strong_lines[0:n2, 0, 0], atol=15)
                closeness_theta = np.isclose(theta, strong_lines[0:n2, 0, 1], atol=20 * np.pi / 180)
                closeness = np.all([closeness_rho, closeness_theta], axis=0)
                if not any(closeness) and n2 < max_lines:
                    # check if lines are parallel or perpendicular
                    strong, _ = check_map(rho, theta, map, draw=False)
                    if strong:
                        strong_lines[n2] = strong
                        n2 += 1
    return strong_lines


def check_map(rho, theta, map, draw=False):
    '''
    Aim: draw lines on map and check if valid
    '''
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
    (out, _, _) = cv2.clipLine((0, 0, map.shape[0], map.shape[1]), pt1, pt2)  # out checks if point is in image
    if out:
        if draw:
            map = cv2.line(map, pt1, pt2, 100, 1)
        return [rho, theta], map
    else:
        return [], map


def intersection(line1, line2, ):
    '''
    Aim: Only return lines which intersect
    '''
    [rho1, theta1] = line1
    [rho2, theta2] = line2
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    theta1 = theta1 if theta1 != 0 else theta1 + 1e-10
    theta2 = theta2 if theta2 != 0 else theta2 + 1e-10
    m1 = -(np.cos(theta1) / np.sin(theta1))
    m2 = -(np.cos(theta2) / np.sin(theta2))
    if not math.isinf(m1) and not math.isinf(m2):
        m0 = abs(math.atan(abs(m2 - m1) / (1 + m2 * m1))) * (180 / np.pi)
        return [x0, y0, m0]
    else:
        return []


def add_2_dict(dikt, key, value=[]):
    # helper function to add values to dicts
    if key in dikt:
        if value not in dikt[key]:
            dikt[key].append(value)
    else:
        dikt[key] = [value]
    return dikt

class LINE(Node):

    def __init__(self):
        super().__init__('LINE')
        # self.sub_goal = self.create_subscription(OccupancyGrid, '/SAM/map', self.get_lines, 10)
        self.pub_map1 = self.create_publisher(OccupancyGrid, '/SAM/map', 10)
        self.timer_map = self.create_timer(0.5, self.get_lines)

    def get_lines(self):#, msg):
        map = np.load('map.npy').astype(np.uint8)
        # map = np.array(msg.data)
        width = int(map.shape[0])
        # map = map.reshape((width, width))
        t_0 = time.time()
        new_map = imply_lines(map, show=False)
        print('TIME: ', time.time() - t_0)
        self.publish_map(new_map)

    def publish_map(self, map):
        msg = OccupancyGrid()
        map_size = map.shape[0]
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.05
        msg.info.width = math.ceil(map_size)
        msg.info.height = math.ceil(map_size)
        msg.info.origin.position.x = float(-1)
        msg.info.origin.position.y = float(-1)
        data = map.reshape((map_size * map_size,))
        data[np.isnan(data)] = 100
        msg.data = [int(i) for i in data]
        self.pub_map1.publish(msg)



# if __name__ == '__main__':
def main(args=None):
    rclpy.init(args=args)
    ros_node = LINE()
    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # im = np.load('map.npy').astype(np.uint8)
    # imply_lines(im)
    main()
