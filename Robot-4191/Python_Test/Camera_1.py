import cv2
counter = 0
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
while True:
        x = input('Take image with 0')
        if x == '0':
                
                ret, frame = cap.read()
                cv2.imwrite('test_img/image_{}.jpg'.format(counter), frame)
                print('Took image')
                counter += 1
cap.release()
