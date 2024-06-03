import cv2 as cv
import numpy as np

import sys
sys.path.append('../topTurret') #import from parent dir

from packages.rpiControll import Cam

cap = Cam.vCap(0)

frame = cap.read()


track_window = (frame.shape[0]//2,frame.shape[1]//2,100,100)


roi = frame[track_window[1]:track_window[1]+track_window[3], track_window[0]:track_window[0]+track_window[2]]
hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
mask = cv.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
roi_hist = cv.calcHist([hsv_roi],[0],mask,[180],[0,180])
cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)
 
# Setup the termination criteria, either 10 iteration or move by at least 1 pt
term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 100, 1 )
 
while(1):
    frame = cap.read()

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    dst = cv.calcBackProject([hsv],[0],roi_hist,[0,180],1)

    # apply camshift to get the new location
    ret, track_window = cv.CamShift(dst, track_window, term_crit)

    # Draw it on image
    pts = cv.boxPoints(ret)
    pts = np.int0(pts)
    img2 = cv.polylines(frame,[pts],True, 255,2)
    cv.imshow('img2',img2)
    cv.waitKey(1)
