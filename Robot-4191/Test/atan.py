import math
import numpy as np

pose = [1, 1, np.pi/2]
goal = [-1, -1]

angle_between_points = lambda pose, goal: np.arctan2(goal[1] - pose[1], goal[0] - pose[0])
angle_to_rotate = angle_between_points(pose, goal) - pose[2]
# Ensures minimum rotation
if angle_to_rotate < -np.pi:
    angle_to_rotate += 2 * np.pi
if angle_to_rotate > np.pi:
    angle_to_rotate -= 2 * np.pi
# Turning left is a positive value, turning right is a negative value of angle
print(math.degrees(angle_to_rotate))