# teleoperate the robot and perform SLAM
# will be extended in following milestones for system integration

# basic python packages
import numpy as np
import cv2 
import os, sys
import time
t0 = time.time()
print("Camera Thresholding")
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
ret, frame = cap.read()
cv2.imwrite('image_test.jpg', frame)
cap.release()
img_big = frame
img_big = cv2.flip(img_big, 0)
## Image Manipulation
#img_big = cv2.imread("light5.jpg")
scale_percent = 20 # percent of original size
width = int(img_big.shape[1] * scale_percent / 100)
height = int(img_big.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img_big, dim, interpolation = cv2.INTER_AREA)

## Binary Threshold
grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#cv2.imshow('Grayscale', grayscale)
(T, threshInv) = cv2.threshold(grayscale, 60, 255,
	cv2.THRESH_BINARY_INV)
threshInv = cv2.dilate(threshInv, None, iterations=1)
#cv2.imshow("Threshold Binary Inverse", threshInv)

## Colour Range Threshold
BALL_MIN = np.array([0, 0, 30],np.uint8)
BALL_MAX = np.array([80, 250, 180],np.uint8)

hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)


# h, s, v = cv2.split(hsv_img)
# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
# h = clahe.apply(h)
# clahe2 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
# s = clahe.apply(s)
# clahe3 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
# v = clahe.apply(v)
# hsv_image = cv2.merge([h, s, v])
# hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)
#cv2.imshow("HSV CLAHE", hsv_image)

frame_threshed = cv2.GaussianBlur(hsv_img,(9,9),cv2.BORDER_DEFAULT)
frame_threshed = cv2.inRange(frame_threshed, BALL_MIN, BALL_MAX)
frame_threshed = cv2.erode(frame_threshed, None, iterations=2)
frame_threshed = cv2.dilate(frame_threshed, None, iterations=2)
#cv2.imshow("HSV", hsv_img)
#cv2.imshow("Threshold HSV", frame_threshed)


gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray,(5,5),0)
gray = cv2.medianBlur(gray,5)
gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,29,3.5)
kernel = np.ones((3,3),np.uint8)
gray = cv2.erode(gray,kernel,iterations = 1)
gray = cv2.dilate(gray,kernel,iterations = 2)
circles1 = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, \
                    param1=80, param2=23, minRadius=0, maxRadius=0)
gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
#if circles1 is not None:
#    for i in circles1[0,:]:        # draw the outer circle
#        cv2.circle(gray,(i[0],i[1]),i[2],(0,0,255), 1)
#cv2.imshow("StackOverflow Way", gray)

composed_img = threshInv & frame_threshed
composed_img = cv2.dilate(composed_img, None, iterations=3)
#circles = np.uint16(np.around(circles))

width = int(img.shape[1])
height = int(img.shape[0])
pt1 = (int(width/2), height)        
pt2 = (int(width/2), 0)
cv2.line(img, pt1, pt2, (255,0,255), 1)

## Circles
circles = cv2.HoughCircles(composed_img,cv2.HOUGH_GRADIENT,1,40,
                            param1=80,param2=11,minRadius=0,maxRadius=90)
if circles is not None:
    for i in circles[0,:]:        # draw the outer circle
        cv2.circle(img,(i[0],i[1]),i[2],(0,0,255),1)        # draw the center of the circle
        center_coordinates = (i[0], i[1])
        pt3 = (int(center_coordinates[0]), int(center_coordinates[1]-i[2]))
        pt4 = (int(center_coordinates[0]), int(center_coordinates[1]+i[2]))
        cv2.line(img, pt3, pt4, (255,0,255), 1)
#cv2.putText(img=, text='Hello', org=(150, 250), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=3, color=(0, 255, 0),thickness=3)
print(time.time()-t0)
#cv2.imshow('detected circles', img)
#cv2.imshow('composed image', composed_img)
#cv2.waitKey(0)
if (cv2.waitKey(0) & 0xFF) == ord('q'):
    cv2.destroyAllWindows()
