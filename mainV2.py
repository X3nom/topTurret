import cv2 as cv
import numpy as np
from ultralytics import YOLO
from threading import Thread
import time
from packages import sort
from packages.rpiControll import Cam
from packages import team_json_loader
from types import NoneType

try: from packages.rpiControll import servoController_external as servoController
except: print("servo controll could not be loaded")





class VideoProcessor():
    def __init__(self, vcap, model='./Yolo_weights/yolov8n.pt') -> None:
        self.vCap = Cam.vCap(vcap)

        self.prev_frame = self.vCap.read()
        self.frame = self.vCap.read()

        
        self.model = YOLO(model) # load up neural network model
        self.sort = sort.Sort(30,1) # load Sort for indexing

        


    def next_frame(self): # advance to next frame
        self.prev_frame = self.frame.copy()
        self.frame = self.vCap.read()


    def find_movement(self):
        pass


    def run_yolo(self, frame):
        'run YOLO'
        detection = self.model(frame, True)
        people = np.empty((0,5))

        for detected in detection: 
            boxes = detected.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                if int(box.cls[0]) == 0 and len(frame[y1:y2,x1:x2]) > 0:
                    person_arr = np.array([x1,y1,x2,y2,box.conf[0].cpu().numpy()]) #get data about detection into format required by sort
                    people = np.vstack((people,person_arr))

        sort_ret = self.sort.update(people) #sends data about detections to sort, sort tryes to associate people from previous frames with new detections gives them IDs

        for res in sort_ret:
            x1,y1,x2,y2,Id = res
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)


            if True:
                # graphics for visual validation of data
                cv.rectangle(frame,(x1,y1),(x2,y2),(255,0,255),2)
                cv.putText(frame,str(int(Id)),np.array([x1,y2-10]),cv.FONT_HERSHEY_SIMPLEX,1,(255,0,255),2,cv.LINE_AA)


    def camshift(self, people):
        pass

    def lucas_kanade():
        pass


    def track():
        pass




class LucasKanade():
    def __init__(self, frame) -> None:

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
            qualityLevel = 0.005,
            minDistance = 10,
            blockSize = 5 )
        
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize = (15, 15),
            maxLevel = 200,
            criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        
        
        # Take first frame and find corners in it
        old_frame = frame
        self.old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

        # self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
        
        self.p0 = {-1 : cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)} # np.ndarray((0,1,2))
        self.areas = {-1 : [0, 0, self.old_gray.shape[0], self.old_gray.shape[1]]}

        # print(self.p0)


    def run(self, frame, draw_frame=None):
        self.frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        for id in self.p0.keys():

            if len(self.p0[id]) < 1: self.find_good_features_in_area(self.frame, self.areas[id], id)  # self.p0[id] = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params) #if not enough points, find new features
            
            # calculate optical flow
            p1, st, err = cv.calcOpticalFlowPyrLK(self.old_gray, self.frame_gray, self.p0[id], None, **self.lk_params)
            
            # Select good points
            if p1 is not None:
                good_new = p1[st==1]
                good_old = self.p0[id][st==1]
            

            if type(draw_frame) != NoneType:
                for pt in good_new:
                    x, y = pt
                    cv.circle(draw_frame, [int(x), int(y)], 3, (255, 0, 255), 2)

            # Now update the previous frame and previous points
            self.old_gray = self.frame_gray.copy()
            self.p0[id] = good_new.reshape(-1, 1, 2)



    
    def find_good_features_in_area(self, frame, area, id=-1): # IDK WHAT THE FUCK AM I DOING
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        mask = np.zeros([frame.shape[0], frame.shape[1], 1], np.uint8)

        mask[area[1]:area[3],area[0]:area[2]] = 255
        # cv.imshow("mask",mask)
        # cv.waitKey(1)

        found_p = cv.goodFeaturesToTrack(frame_gray, mask=mask, **self.feature_params)
        
        
        try:
            self.p0[id] = found_p
            self.areas[id] = area
        except:
            self.p0.update({id, found_p})
            self.areas.update({id, area})

        # print(self.p0)







if __name__ == "__main__":

    
    vid_process = VideoProcessor(0)

    lk = LucasKanade(vid_process.frame)
    lk.find_good_features_in_area(vid_process.frame, [0,300,100,400], -1)
    lk.find_good_features_in_area(vid_process.frame, [600,200,800,400], 0)

    teams_config = team_json_loader.TeamConfig()
    # teams_config.
    

    # MAIN LOOP ------------
    while True:
        vid_process.next_frame()
        draw_frame = vid_process.frame.copy()

        # vid_process.run_yolo(vid_process.frame)
        lk.run(vid_process.frame, draw_frame)

        

        cv.imshow("window", draw_frame)
        cv.waitKey(1)