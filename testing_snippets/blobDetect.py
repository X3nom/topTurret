import cv2 as cv
import numpy as np
import sys
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import Cam

cv.SimpleBlobDetector.Params
blob_params = cv.SimpleBlobDetector_Params()
 
blob_params.filterByColor = False

# Filter by area (value for area here defines the pixel value)
blob_params.filterByArea = True
blob_params.minArea = 1
blob_params.maxArea = 5000000

# Filter by circularity
blob_params.filterByCircularity = False
blob_params.minCircularity = 0.75
 
# Filter by convexity
blob_params.filterByConvexity = False
blob_params.minConvexity = 0.2
     
# Filter by inertia ratio
blob_params.filterByInertia = False
blob_params.minInertiaRatio = 0.01
 
# Creating a blob detector using the defined parameters
blob_detector = cv.SimpleBlobDetector_create(blob_params)


colRange = [[179,255,255],
            [162,169,106]]
cap = Cam.vCap(0)
persistant = np.zeros([cap.size[0], cap.size[1], 3], np.uint8)
while True:
    frame = cap.read()
    #frame = cv.imread(r"C:\Users\Panchártek Jakub\Downloads\akjfdl;kafsj;ldgh.jpg")
    

    hsvFrame = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsvFrame, np.array(colRange[1]), np.array(colRange[0]))
    #mask = cv.GaussianBlur(mask, (11,11), 0)

    #persistant, contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #mask[mask!=0] = 255

    #mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

    keypoints = blob_detector.detect(mask)

    frame = cv.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv.imshow("mask", mask)
    cv.imshow("frame", frame)
    #cv.imshow("pers", persistant)
    cv.waitKey(1)