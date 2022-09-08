import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
from scipy import ndimage
from PIL import Image

def add_map(rho, theta, map, strong_lines):
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    (out, _, _) = cv2.clipLine((0, 0, map.shape[0], map.shape[1]), pt1, pt2) # out checks if point is in image
    if out:
        cv2.line(map, pt1, pt2, 100, 1)
        print(pt1, pt2, rho, theta)
        strong_lines[n2] = [rho, theta]
    return map, strong_lines

map = np.load('map.npy').astype(np.uint8)
map[map != 100] = 0
map[map == 100] = 1
fig, ax = plt.subplots(1,3)
ax[0].imshow(map)
# plt.show()
lines = cv2.HoughLines(map, rho=1, theta=np.pi/180, threshold=8)
max_lines = 10
strong_lines = np.zeros([max_lines,1,2])
n2 = 0 # count how many strong lines added
if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        # theta -= np.pi if rho < 0 else 0
        rho *= -1 if rho < 0 else 1
        if i == 0:
            map, strong_lines = add_map(rho, theta, map, strong_lines)
            n2 += 1
        else:
            closeness_rho = np.isclose(rho, strong_lines[0:n2, 0, 0], atol=15)
            closeness_theta = np.isclose(theta, strong_lines[0:n2, 0, 1], atol=45 * np.pi / 180)
            closeness = np.all([closeness_rho, closeness_theta], axis=0)
            if not any(closeness) and n2 < max_lines:
                # check if lines are parallel or perpendicular
                map, strong_lines = add_map(rho, theta, map, strong_lines)
                n2 += 1
thetas = strong_lines[:n2, 0, 1]
thetas[np.nonzero(thetas)]
print(thetas)
thetas[thetas >= np.pi/2] -= np.pi
thetas[thetas >= np.pi/4] -= np.pi/2
thetas[thetas <= -np.pi/4] += np.pi/2
print(thetas)
theta_offset = np.mean(thetas) * 180 / np.pi
print(theta_offset)
map_rotated = ndimage.rotate(map, theta_offset)
ax[1].imshow(map)
ax[2].imshow(map_rotated)
plt.show()



# All the changes made in the input image are finally
# written on a new image houghlines.jpg