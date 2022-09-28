import cv2
import time
counter = 0
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
while True:
        ret, frame = cap.read()
        x = input('Take image with 0')
        if x == '0':
                cv2.imwrite('test_img/image_{}_{}.jpg'.format(counter, time.time()), frame)
                print('Took image')
                counter += 1
cap.release()
