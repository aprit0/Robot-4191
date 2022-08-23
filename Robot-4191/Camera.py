from gpiozero import Button
from picamera import PiCamera
from datetime import datetime
from signal import pause

camera = PiCamera()

def capture(i):
    timestamp = datetime.now().isoformat()
    camera.start_preview()
    sleep(5)
    camera.capture('~/Documents/%d%s.jpg' % i, timestamp)
    camera.stop_preview()

for i in range(10):
    capture(i)
    print(i)
