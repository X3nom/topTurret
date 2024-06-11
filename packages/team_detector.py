import cv2 as cv
import numpy as np
import sys
# from rpiControll import Cam #TODO: RM


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
        MIN_SIZE_TO_DETECTION_SIZE = 1/12000
        MAX_SIZE_TO_DETECTION_SIZE = 1/1

        hsv_img = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        total_size = img.size # used for armband size to detection size ratio

        # hsvFrame_equ = hsv_img
        # hsvFrame_equ[:, :, 2] = cv.equalizeHist(hsvFrame_equ[:, :, 2]) #EQUALIZE

        best_team_name = "Unknown"
        best_team_size = 0

        for team in self.teams:
            # get color in range
            mask = cv.inRange(hsv_img, np.array(team["lower"]), np.array(team["upper"]))

            #correct for small imperfections
            mask = cv.GaussianBlur(mask, (9,9), 0)
            mask[mask!=0] = 255

            #mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
            # find blobs
            keypoints = self.blob_detector.detect(mask)

            frame_out = cv.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) #TODO: rm


            for kp in keypoints: #filter out good kp
                if kp.size/total_size >= MIN_SIZE_TO_DETECTION_SIZE and kp.size/total_size <= MAX_SIZE_TO_DETECTION_SIZE:
                    if best_team_size < kp.size:
                        best_team_size = kp.size
                        best_team_name = team["name"]

            # cv.imshow("img",frame_out)
            # cv.waitKey(0)
        return best_team_name


# d = TeamDetector([
#         {
#             "name": "red",
#             "upper": [179,255,255],
#             "lower": [162,169,106]
#         },
#         {
#             "name": "blue",
#             "upper": [123,255,191],
#             "lower": [106,174,52]
#         },
#         {
#             "name": "yellow",     
#             "upper": [47, 255, 255],
#             "lower": [17, 0, 132]
#         }])

# cap = cv.VideoCapture(0)
# while True:
#     ret, frame = cap.read(0)
#     tm = d.detect_team(frame)
#     print(tm)