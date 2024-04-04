import cv2 as cv
import numpy as np
import sys
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import Cam

cap = Cam.vCap(0)
while True:
    frame = cap.read()

    hsvFrame = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    cv.imshow("frame",hsvFrame)
    cv.waitKey(1)