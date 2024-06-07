import cv2 as cv
import numpy as np
import sys
from rpiControll import Cam


class TeamDetector():
    def __init__(self, teams) -> None:
        cv.SimpleBlobDetector.Params
        blob_params = cv.SimpleBlobDetector_Params()
        
        blob_params.filterByColor = False

        # Filter by area (value for area here defines the pixel value)
        blob_params.filterByArea = True
        blob_params.minArea = 100
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
        self.blob_detector = cv.SimpleBlobDetector_create(blob_params)


        self.teams = teams
    
    def detect_team(self, img):
        hsv_img = cv.cvtColor(img,cv.COLOR_BGR2HSV)

        for team in self.teams:
            mask = cv.inRange(hsv_img, np.array(team["lower"]), np.array(team["upper"]))
            #correct for small imperfections
            mask = cv.GaussianBlur(mask, (9,9), 0)
            mask[mask!=0] = 255

            #mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

            keypoints = self.blob_detector.detect(mask)




cap = Cam.vCap(0)
persistant = np.zeros([cap.size[0], cap.size[1], 3], np.uint8)
while True:
    frame = cap.read()
    
    #frame = cv.imread(r"C:\Users\Panchártek Jakub\Downloads\akjfdl;kafsj;ldgh.jpg")

    hsvFrame = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

    mask = cv.inRange(hsvFrame, np.array(colRange[1]), np.array(colRange[0]))
    #correct for small imperfections
    mask = cv.GaussianBlur(mask, (9,9), 0)
    mask[mask!=0] = 255

    #mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)

    keypoints = blob_detector.detect(mask)

    frame_out = frame.copy()
    frame_out = cv.drawKeypoints(frame_out, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    for kp in keypoints:
        #print(kp.size)
        cv.putText(frame_out, str(int(kp.size)), [int(x) for x in kp.pt], cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)

    
    cv.imshow("mask", mask)
    cv.imshow("frame", frame_out)
    #cv.imshow("pers", persistant)
    cv.waitKey(1)