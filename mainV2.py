import cv2 as cv
import numpy as np
from ultralytics import YOLO
from threading import Thread
import time
from packages import sort
from packages.rpiControll import Cam
from packages import team_json_loader
from types import NoneType
import math

try: from packages.rpiControll import servoController_external as servoController
except: print("servo controll could not be loaded")




class Person():
    def __init__(self, Id, box) -> NoneType:
        self.Id = Id
        self.box = box
        self.team = "Unknown" #defalut team
        self.team_occurencies = {self.team:0}
        self.p0 = np.ndarray([0,1,2])
        self.ttl = 6000

    
    
    def compute_team(self):
        max_team = self.team_occurencies[self.team]
        for team in self.team_occurencies.keys():
            if self.team_occurencies[team] > max_team:
                max_team = team
        self.team = max_team



class Tracker():
    def __init__(self, init_frame, model='./Yolo_weights/yolov8n.pt') -> None:      
        self.model = YOLO(model) # load up neural network model
        self.sort = sort.Sort(30,1) # load Sort for indexing

        self.found_people = {}

        self.lk = LucasKanade(init_frame, self.found_people, track_full_frame=False)




    def find_people(self, frame):
        'run YOLO'
        detection = self.model.predict(frame, True)
        people = np.empty((0,5))

        for detected in detection: 
            boxes = detected.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                box_center = [x1+(x1-x2)//2, y1+(y1-y2)//2]  # calculate box center

                if int(box.cls[0]) == 0 and len(frame[y1:y2,x1:x2]) > 0:
                    '''
                    #TODO: check Id based on n of lk keypoints insIde (Id=Id of most frequent keypoints insIde box)
                    closest_center = np.array([-1,-1])
                    closest_dst = -1
                    closest_Id = -1
                    for Id in self.found_people.keys():
                        center_of_mass = self.lk.calculate_center_of_mass(Id)[0] # find closest average point with Id to the box
                        

                        if math.sqrt( (box_center[0]-center_of_mass[0])**2 + (box_center[1]-center_of_mass[1])**2 ) < closest_dst or closest_dst == -1:
                            closest_center = center_of_mass
                            closest_dst = math.sqrt( (box_center[0]-center_of_mass[0])**2 + (box_center[1]-center_of_mass[1])**2 )
                            closest_Id = Id

                    if closest_center[0] > x1 and closest_center[0] < x2 and closest_center[1] > y1 and closest_center[1] < y2: # if closest Id point is insIde box
                        person_arr = np.array([x1,y1,x2,y2,closest_Id])
                        people = np.vstack((people,person_arr))

                    
                    else:
                        random_Id = 0
                        while random_Id in self.found_people.keys(): random_Id += 1 #iterate until not taken Id is found
                        person_arr = np.array([x1,y1,x2,y2,random_Id])
                        people = np.vstack((people,person_arr))
                        '''
                    
                    person_arr = np.array([x1,y1,x2,y2,box.conf[0].cpu().numpy()]) #get data about detection into format required by sort
                    people = np.vstack((people,person_arr))

                    '''
        for person in people:
            x1,y1,x2,y2,Id = person
            x1, y1, x2, y2, Id = int(x1), int(y1), int(x2), int(y2), int(Id)

            try: # try to update box or add person if not yet found
                self.found_people[Id].box = [x1, y1, x2, y2]
            except:
                self.found_people.update({Id : Person(Id, [x1, y1, x2, y2])})
                self.lk.find_good_features_in_area(frame, self.found_people[Id].box, Id)

        '''
        sort_ret = self.sort.update(people) #sends data about detections to sort, sort tryes to associate people from previous frames with new detections gives them IDs
        
        for res in sort_ret:
            x1,y1,x2,y2,Id = res
            x1, y1, x2, y2, Id = int(x1), int(y1), int(x2), int(y2), int(Id)


            try: # try to update box or add person if not yet found
                self.found_people[Id].box = [x1, y1, x2, y2]
            except:
                self.found_people.update({Id : Person(Id, [x1, y1, x2, y2])})
                self.lk.find_good_features_on_person(frame, Id)


            if True:
                # graphics for visual valIdation of data
                cv.rectangle(frame,(x1,y1),(x2,y2),(255,0,255),2)
                cv.putText(frame,str(int(Id)),np.array([x1,y2-10]),cv.FONT_HERSHEY_SIMPLEX,1,(255,0,255),2,cv.LINE_AA)
        



    def update_lk_features(self, frame, minimum_pt_treshold=6, force_update=False):
        for Id in self.found_people.keys():

            if len(self.found_people[Id].p0) < minimum_pt_treshold or force_update: # if Id does not exist or has less than n keypoints existing, find new features
                self.lk.find_good_features_on_person(frame, Id)

    
    def run_lk(self, frame, draw_frame=None):
        self.lk.run(frame, draw_frame=draw_frame)
    

    def update_boxes(self):
        for Id in self.found_people.keys():
            self.found_people[Id].box = self.lk.find_box(Id)

    
    def merge_people(self, Id, Id2):
        pass

    
    #TODO: implement a way to destroy person objects when not found (ttl or no kp left?)





class LucasKanade(): #TODO: rewrite to use person objects instead of p0 dict
    def __init__(self, frame, people, track_full_frame=True) -> None:

        self.people = people

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
            qualityLevel = 0.005,
            minDistance = 10,
            blockSize = 5 )
        
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize = (15, 15),
            maxLevel = 200,
            criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03),
            minEigThreshold = 1e-4)
        
        
        # Take first frame and find corners in it
        old_frame = frame
        self.old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

        # self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
        '''
        if track_full_frame:
            self.p0 = {-1 : cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)} # np.ndarray((0,1,2))
            self.areas = {-1 : [0, 0, self.old_gray.shape[0], self.old_gray.shape[1]]}
        else:
            self.p0 = {}
            self.areas = {}
        '''

        # print(self.p0)


    def run(self, frame, draw_frame=None, dump_empty = True):
        self.frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        for person in self.people.values():

            if len(person.p0) < 1:
                if dump_empty:
                    #TODO: Idk if this should be included even, probably just add some flag
                    continue
                    
                else: self.find_good_features_on_person(self.frame_gray, person.Id)  # self.p0[Id] = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params) #if not enough points, find new features
            
            # calculate optical flow
            p1, st, err = cv.calcOpticalFlowPyrLK(self.old_gray, self.frame_gray, person.p0, None, **self.lk_params)
            
            # Select good points
            if p1 is not None:
                good_new = p1[st==1]
                good_old = person.p0[st==1]
            
            else: continue

            if draw_frame is not None:
                for pt in good_new:
                    x, y = pt
                    cv.circle(draw_frame, [int(x), int(y)], 3, (255, 0, 255), 2)
                center = self.calculate_center_of_mass(person.Id)
                cv.circle(draw_frame, [int(x) for x in center[0]], 5, (0,0,255), 3)

            # Now update the previous frame and previous points
            person.p0 = good_new.reshape(-1, 1, 2)
        self.old_gray = self.frame_gray.copy()



    
    def find_good_features_on_person(self, frame, Id=-1):
        try: person = self.people[Id]
        except: return -1

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        mask = np.zeros([frame.shape[0], frame.shape[1], 1], np.uint8)

        mask[person.box[1]:person.box[3],person.box[0]:person.box[2]] = 255

        found_p = cv.goodFeaturesToTrack(frame_gray, mask=mask, **self.feature_params)
        if found_p is None: found_p = np.ndarray((0,1,2))
        
        person.p0 = found_p



    def calculate_center_of_mass(self, Id):
        avg = np.mean(self.people[Id].p0, 0)
        return avg



    def find_box(self, Id):
        #TODO: throw away keypoints that are far from others
        kp = self.people[Id].p0
        if len(kp) == 0: return [0, 0, 0, 0]
        top_left = np.min(kp, 0)[0]
        lower_right = np.max(kp, 0)[0]
        return list(map(int,[top_left[0], top_left[1], lower_right[0], lower_right[1]]))
    

    def filter_stray_points(self, Id):
        pass








if __name__ == "__main__":
    vCap = Cam.vCap(0)
    
    frame = vCap.read()
    
    tracker = Tracker(frame)

    

    teams_config = team_json_loader.TeamConfig()
    # teams_config.
    

    # MAIN LOOP ------------
    iteration = 0
    while True:
        frame = vCap.read()
        draw_frame = frame.copy()

        if iteration % 600 == 0:
            tracker.find_people(frame)

        tracker.update_boxes()

        tracker.update_lk_features(frame)

        tracker.run_lk(frame, draw_frame)
        
        # tracker.run_yolo(tracker.frame)
        # lk.run(tracker.frame, draw_frame)

        

        cv.imshow("window", draw_frame)
        cv.waitKey(1)
        iteration += 1