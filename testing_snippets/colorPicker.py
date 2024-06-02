import cv2 as cv
import numpy as np
import sys
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import Cam

def trackbarFn(x):
    return

def onClick(event,x,y,flags,param):
    global BARVALS, hsvFrame
    if event == cv.EVENT_LBUTTONDBLCLK:
        col = hsvFrame[y,x]
        print(col)
        i = 0
        for v in BARVALS:
            cv.setTrackbarPos(f"{v[0]} <", "bars", col[i]-20)
            cv.setTrackbarPos(f"> {v[0]}", "bars", col[i]+20)
            i += 1

        '''
        name = "{} <"
        if cv.getTrackbarPos("Low/Hig","bars") == 0:
            name = "> {}"
        i = 0
        for v in BARVALS:
            cv.setTrackbarPos(name.format(v[0]), "bars", col[i])
            i += 1 '''


cv.namedWindow('bars')

BARVALS = (('H', 179),
           ('S', 255),
           ('V', 255))
barImg = np.zeros([30,30,3], np.uint8)
for v in BARVALS: #create bars and set max bars to max
    cv.createTrackbar(v[0]+" <",'bars',0,v[1], trackbarFn)
for v in BARVALS:
    cv.createTrackbar("> "+v[0],'bars',0,v[1], trackbarFn)
for v in BARVALS:
    cv.setTrackbarPos("> "+v[0], "bars", v[1])

cv.createTrackbar("set Off/On", "bars", 0,1, trackbarFn)
cv.createTrackbar("Low/Hig", "bars", 0,1, trackbarFn)
#cv.imshow("bars", barImg)


colRange = [[0,0,0],
            [179,255,255]]

cap = Cam.vCap(0)

#blobDet = cv.SimpleBlobDetector.

frame = cap.read()
cv.imshow("image", frame)
cv.setMouseCallback("image", onClick)
while True:

    colRange = [[cv.getTrackbarPos("H <","bars"),cv.getTrackbarPos("S <","bars"), cv.getTrackbarPos("V <","bars")],
                [cv.getTrackbarPos("> H","bars"),cv.getTrackbarPos("> S","bars"), cv.getTrackbarPos("> V","bars")]]

    frame = cap.read()

    hsvFrame = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    hsvFrame_equ = hsvFrame.copy()
    hsvFrame_equ[:, :, 2] = cv.equalizeHist(hsvFrame_equ[:, :, 2]) #EQUALIZE

    equalised_rgb = cv.cvtColor(hsvFrame_equ, cv.COLOR_HSV2BGR)
    
    mask = cv.inRange(hsvFrame, np.array(colRange[0]), np.array(colRange[1]))
    invMask = cv.bitwise_not(mask)

    outFrame = cv.bitwise_and(equalised_rgb, frame, mask=mask)
    
    underlay = np.zeros(frame.shape)
    underlay[:] = (0,0,255)

    #underlay = cv.bitwise_and(underlay,underlay, mask=invMask)

    #outFrame = cv.addWeighted(underlay,1,outFrame,1,1)
    

    cv.imshow("image",frame)
    cv.imshow("mask",outFrame)
    cv.waitKey(1)