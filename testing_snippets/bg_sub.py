import cv2 as cv
import numpy as np
import queue

import sys
sys.path.append('../topTurret') #import from parent dir

from packages.rpiControll import Cam






cap = cv.VideoCapture(1)#Cam.vCap(0)


diff_buffer = queue.Queue(100)

r, frame = cap.read() #get initial frame
prev_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

while True:
    
    r, frame = cap.read()


    frame_gra = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    


    diff = frame_gra - prev_frame
    diff = np.abs(diff)

    diff = cv.GaussianBlur(diff, (7, 7), 0)

    # if diff_buffer.full(): diff_buffer.get()
    # diff_buffer.put(diff.copy())


    # diff_avg = diff.copy()
    # i = 0
    # for d in list(diff_buffer.queue):
    #     diff_avg = diff_avg+d
    #     i += 1
    # diff_avg = diff_avg/i

    # diff_avg = diff-diff_avg


    #diff[diff<230] = 0


    cv.imshow("diff", diff)
    cv.waitKey(10)

    prev_frame = frame_gra.copy()