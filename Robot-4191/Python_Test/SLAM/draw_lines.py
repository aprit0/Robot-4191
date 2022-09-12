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
        # print(pt1, pt2, rho, theta)
        strong_lines[n2] = [rho, theta]
    return map, strong_lines

def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
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
    if math.isinf(m1) or math.isinf(m2):
        m0 = np.nan
    else:
        m0 = abs(math.atan(abs(m2 - m1) / (1 + m2 * m1))) * (180 / np.pi)
    if not np.isnan(m0): # and np.isclose(abs(m0), np.pi/2, 30 * np.pi/2):
        return [x0, y0, m0]
    else:
        return []
im = np.load('map_1.npy').astype(np.uint8)
map = im.copy()
map[map != 100] = 0
map[map == 100] = 1
fig, ax = plt.subplots(1,3)
ax[0].imshow(map)
# plt.show()
lines = cv2.HoughLines(map, rho=1, theta=np.pi/180, threshold=6)
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
            closeness_theta = np.isclose(theta, strong_lines[0:n2, 0, 1], atol=20 * np.pi / 180)
            closeness = np.all([closeness_rho, closeness_theta], axis=0)
            if not any(closeness) and n2 < max_lines:
                # check if lines are parallel or perpendicular
                map, strong_lines = add_map(rho, theta, map, strong_lines)
                n2 += 1

# find out if intersect is ~1.2m
lines = {}
def add_2_dict(dikt, key, value = []):
    if key in dikt:
        if value not in dikt[key]:
            dikt[key].append(value)
    else:
        dikt[key] = [value]
    return dikt
for i in range(strong_lines.shape[0]):
    for j in range(1, strong_lines.shape[0]):
        ln0= list(strong_lines[i][0])
        ln1 = list(strong_lines[j][0])
        if ln0 == ln1 or ln0 == [0,0] or ln1 == [0,0]:
            pass
        else:
            coord = intersection(ln0, ln1)
            # print(coord)
            if coord != [] and 80 > coord[0] > 0 and 80 > coord[1] > 0:
                lines = add_2_dict(lines, tuple(ln0), coord)
                lines = add_2_dict(lines, tuple(ln1), coord)
euc = lambda x0, y0, x1, y1: ((y1 - y0) ** 2 + (x1 - x0) ** 2) ** 0.5
print(lines)
perp_lines = []
for i in lines:
    if len(lines[i]) == 2:
        [x0, y0], [x1, y1] = lines[i][0][:2], lines[i][1][:2]
        out = euc(x0, y0, x1, y1)
    elif len(lines[i]) >= 2:
        for j in range(len(lines[i])-1):
            for k in range(1, len(lines[i])):
                [x0, y0], [x1, y1] = lines[i][j][:2], lines[i][k][:2]
                out = euc(x0, y0, x1, y1)

    else:
        out = 0
    if np.isclose(out, 47, atol=5):
        perp_lines.append(i)
    if out:
        print('out', out, [x0, y0], [x1, y1])

thetas = [i[1]*180/np.pi for i in perp_lines]
thetas = [i - 90 if i > 45 else i for i in thetas]
print(thetas)
theta_offset = np.mean(thetas)
print(theta_offset)
map_rotated = ndimage.rotate(map, theta_offset)
ax[2].imshow(map_rotated)
ax[1].imshow(map)
ax[0].imshow(im)
plt.xlabel('Rotated by: {:.2f} degrees'.format(theta_offset))
plt.show()



# All the changes made in the input image are finally
# written on a new image houghlines.jpg