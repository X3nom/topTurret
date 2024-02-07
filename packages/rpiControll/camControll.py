import cv2 as cv
from picamera2 import Picamera2
import numpy as np

picam2 = Picamera2()
picam2.start()

while True:
    frame = picam2.capture_array()
    cv.imshow("test",frame)
    cv.waitKey(1)