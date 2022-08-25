import cv2
import time

def capture(i):
    cap = cv2.VideoCapture('~/Documents', cv2.CAP_V4L)
    time.sleep(1)
    ret,frame = cap.read()
    cv2.imwrite('%d_%s.jpg' % i,time.time(),frame)
    cap.release()

for i in range(5):
    capture(i)
    print(i)
