import sys
sys.path.append('../topTurret') #import from parent dir

from packages.rpiControll import Cam
import cv2 as cv
import numpy as np

cap = Cam.vCap(0)
if not cap.isOpen():
    raise
fshape = cap.read().shape

coor = [0,0]
mid = [ int(x/2) for x in fshape ]

fov = [41,66]

pxDeg = [mid[i]/fov[i] for i in range(2)]

while True:
    frame = cap.read()
    
    cv.imshow("window", frame)
    cv.waitKey(1)