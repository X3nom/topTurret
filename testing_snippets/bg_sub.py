import cv2 as cv
import numpy as np

import sys
sys.path.append('../topTurret') #import from parent dir

from packages.rpiControll import Cam






cap = cv.VideoCapture(0)#Cam.vCap(0)


r, frame = cap.read() #get initial frame
prev_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

while True:
    
    r, frame = cap.read()


    frame_gra = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    


    diff = frame_gra - prev_frame
    diff = np.abs(diff)

    diff = cv.GaussianBlur(diff, (7, 7), 0)
    diff = cv.GaussianBlur(diff, (7, 7), 0)
    diff = cv.GaussianBlur(diff, (7, 7), 0)

    #diff[diff<230] = 0


    cv.imshow("diff", diff)
    cv.waitKey(10)

    prev_frame = frame_gra.copy()