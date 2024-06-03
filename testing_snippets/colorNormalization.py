import cv2 as cv
import numpy as np
import sys
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import Cam

from matplotlib import pyplot as plt
import time

COLOR = ["b", "g", "r"]
plt.ion()

figure, ax = plt.subplots()#figsize=(10, 8))

colRange = [[69,255,255],
            [0,94,0]]

cap = Cam.vCap(0)

while True:
    frame = cap.read()
    frame_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    #frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    #hist = cv.calcHist(frame, [1], None, [256],[0,256])
    '''
    for i,col in enumerate(COLOR):
        histr = cv.calcHist([frame_hsv],[i],None,[256],[0,256])
        plt.plot(histr,color = col)
        #plt.xlim([0,256])
        '''
    ormask = cv.inRange(frame_hsv, np.array(colRange[1]), np.array(colRange[0]))
    frame_hsv[:, :, 2] = cv.equalizeHist(frame_hsv[:, :, 2])
    mask = cv.inRange(frame_hsv, np.array(colRange[1]), np.array(colRange[0]))

    norm_frame = cv.cvtColor(frame_hsv, cv.COLOR_HSV2BGR)
        
    cv.imshow("origin", frame)
    cv.imshow("equ", norm_frame)
    # cv.imshow("mask",mask)
    # cv.imshow("orig mask",ormask)
    cv.waitKey(1)

    '''
    figure.canvas.draw()
    figure.canvas.flush_events()
    ax.cla()
    '''
    