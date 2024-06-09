import cv2 as cv
import numpy as np
from ultralytics import YOLO
from threading import Thread
import time
from packages import sort
from packages.rpiControll import Cam
from packages import team_json_loader
# from packages import team_detector
from types import NoneType
import math

try: 
    from packages.rpiControll import servoController_external as servoController
    servo_loaded = True
except:
    print("servo controll could not be loaded")
    servo_loaded = False




class Person():
    def __init__(self, Id, box, sort_reference) -> NoneType:
        self.Id = Id
        self.box = box
        self.team = "Unknown" #defalut team
        self.team_occurencies = {self.team:0}
        self.p0 = np.ndarray([0,1,2]) # init with empty array of desired shape; holds keypoints found on person
        self.center_of_mass = (0,0)
        # self.box_mass_center = ((box[0]-box[2])//2, (box[1]-box[3])//2)
        # self.ttl = 10 # time to live
        self.sort_reference = sort_reference # used for checking if detection is still "alive"
        self.aim_locked = False

    
    
    def compute_team(self):
        max_team = self.team_occurencies[self.team]
        for team in self.team_occurencies.keys():
            if self.team_occurencies[team] > max_team:
                max_team = team
        self.team = max_team



class Tracker():
    def __init__(self, init_frame, model='./Yolo_weights/yolov8n.pt') -> None:      
        self.model = YOLO(model) # load up neural network model
        self.sort = sort.Sort(1,1) # load Sort for indexing

        self.found_people = {}

        self.lk = LucasKanade(init_frame, self.found_people)

        self.crosshair_pos = ((init_frame.shape)[0]//2, (init_frame.shape)[0]//2)

        # self.team_detector = team_detector.TeamDetector(teams)




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
                    
                    person_arr = np.array([x1,y1,x2,y2,box.conf[0].cpu().numpy()]) #get data about detection into format required by sort
                    people = np.vstack((people,person_arr))


        sort_ret = self.sort.update(people) #sends data about detections to sort, sort tryes to associate people from previous frames with new detections gives them IDs
        
        for res in sort_ret:
            x1,y1,x2,y2,Id = res
            x1, y1, x2, y2, Id = int(x1), int(y1), int(x2), int(y2), int(Id-1)


            try: # try to update box or add person if not yet found
                self.found_people[Id].box = [x1, y1, x2, y2]
                # self.lk.filter_box_outliers(Id) #TODO: MAYBE RM
            except:
                sort_reference = list(filter(lambda x:x.id==Id, self.sort.trackers))[0]
                self.found_people.update({Id : Person(Id, [x1, y1, x2, y2], sort_reference)})
                self.lk.find_good_features_on_person(frame, Id)
                

            # self.lk.filter_outliers(Id) #TODO: RM


            if True:
                # graphics for visual valIdation of data
                cv.rectangle(frame,(x1,y1),(x2,y2),(255,0,255),2)
                cv.putText(frame,str(int(Id)),np.array([x1,y2-10]),cv.FONT_HERSHEY_SIMPLEX,1,(255,0,255),2,cv.LINE_AA)
        
        self.delete_expired_people()
        


    def delete_expired_people(self):
        keys = list(self.found_people.keys())
        for Id in keys:
            if self.found_people[Id].sort_reference.alive:
                self.found_people.pop(Id)




    def update_lk_features(self, frame, minimum_pt_treshold=6, force_update=False, Id=None):
        if Id is None:
            keys = self.found_people.keys()
        else:
            keys = [Id]

        for Id in keys:

            if len(self.found_people[Id].p0) < minimum_pt_treshold or force_update: # if Id does not exist or has less than n keypoints existing, find new features
                self.lk.find_good_features_on_person(frame, Id)

    
    def run_lk(self, frame, draw_frame=None):
        self.lk.run(frame, draw_frame=draw_frame)
    

    def update_boxes(self, Id=None):
        SIZE_WEIGHTS = (1, 600) # (new, old) weighted new vs old box (weighted average)
        POS_WEIGHTS = (5,1) # (box_center, center of mass)

        keys = [Id]
        if keys[0] is None:
            keys = self.found_people.keys()

        for Id in keys:
            box = self.lk.find_box(Id)
            box_size = (box[2]-box[0], box[3]-box[1])
            old_box = self.found_people[Id].box
            old_box_size = (old_box[2]-old_box[0], old_box[3]-old_box[1])

            mid = [((box[0]+(box[2]-box[0])//2, box[1]+(box[3]-box[1])//2)[i]*POS_WEIGHTS[0] + self.found_people[Id].center_of_mass[i]*POS_WEIGHTS[1])//sum(POS_WEIGHTS) for i in range(2)]

            avg_box_size = [ (box_size[i]*SIZE_WEIGHTS[0] + old_box_size[i]*SIZE_WEIGHTS[1])//sum(SIZE_WEIGHTS) for i in range(2)]
            avg_box = (mid[0]-avg_box_size[0]//2, mid[1]-avg_box_size[1]//2, mid[0]+avg_box_size[0]//2, mid[1]+avg_box_size[1]//2)
            

            self.found_people[Id].box = avg_box

    

    def cleanup_outliers(self):
        for Id in self.found_people.keys():
            self.lk.filter_outliers(Id)
    
    
    
    def merge_people(self, Id, Id2):
        #TODO: merge two person objects under single Id, merge points, teams, ...;    for use when two "centers of mass" are too close together
        person = self.found_people[Id]
        to_merge_person = self.found_people[Id2]

        person.p0 = np.vstack([person.p0, to_merge_person.p0]) # WIP

    
    #TODO: implement a way to destroy person objects when not found (ttl or no kp left?)

    def find_closest_enemy(self, enemy_teams_names) -> Person:
        closest = None
        
        for person in self.found_people.values():
            person.locked = False
            if person.team in enemy_teams_names:
                if person.center_of_mass == [[-1, -1]]: continue
                if closest is None:
                    closest = person
                elif np.linalg.norm(movement_vector(self.crosshair_pos, person.center_of_mass)) < np.linalg.norm(movement_vector(self.crosshair_pos, closest.center_of_mass)): # compare distances to crosshair of closest and current person
                    closest = person
        
        if closest is not None: closest.locked = True
        return closest





class LucasKanade(): #TODO: rewrite to use person objects instead of p0 dict
    def __init__(self, frame, people) -> None:

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



    def run(self, frame, draw_frame=None, dump_empty = True):
        self.frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        for person in self.people.values():

            if len(person.p0) < 1:
                if dump_empty:
                    #TODO: Idk if this should be included even, probably just add some flag
                    continue
                    
                else: self.find_good_features_on_person(self.frame_gray, person.Id)
            
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
                # center = self.calculate_center_of_mass(person.Id)
                cv.circle(draw_frame, person.center_of_mass, 5, (0,0,255), 3)

                cv.rectangle(draw_frame, [person.box[0], person.box[1]], [person.box[2], person.box[3]], (255, 0, 0), 1)


            # Now update the previous frame and previous points
            person.p0 = good_new.reshape(-1, 1, 2)
            person.center_of_mass = tuple(map(int, self.calculate_center_of_mass(person.Id)[0]))
        self.old_gray = self.frame_gray.copy()



    
    def find_good_features_on_person(self, frame, Id=-1):
        try: person = self.people[Id]
        except: return -1

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        mask = np.zeros([frame.shape[0], frame.shape[1], 1], np.uint8)

        mask[person.box[1]:person.box[3],person.box[0]:person.box[2]] = 255

        found_p = cv.goodFeaturesToTrack(frame_gray, mask=mask, **self.feature_params) #! has to have None return handeled
        if found_p is None: found_p = np.ndarray((0,1,2))
        
        person.p0 = found_p
        
        person.center_of_mass = tuple(map(int, self.calculate_center_of_mass(Id)[0]))



    def calculate_center_of_mass(self, Id):
        if len(self.people[Id].p0) != 0: avg = np.mean(self.people[Id].p0, 0)
        else: avg = [[-1,-1]]
        
        return avg



    def find_box(self, Id):
        kp = self.people[Id].p0
        if len(kp) == 0: return [0, 0, 0, 0]
        top_left = np.min(kp, 0)[0]
        lower_right = np.max(kp, 0)[0]
        return list(map(int,[top_left[0], top_left[1], lower_right[0], lower_right[1]]))
    

    def filter_outliers(self, Id):
        #TODO: throw away keypoints that are far from others
        ALLOWED_DEVIATION = 0.5
        p0 = self.people[Id].p0
        mean = np.mean(p0, 0)
        std_dev = np.std(p0, 0)

        z_scores = (p0-mean)/std_dev

        for i in range(len(z_scores)-1, -1, -1):
            if (z_scores[i][0][0] > ALLOWED_DEVIATION or z_scores[i][0][0] < -ALLOWED_DEVIATION) or (z_scores[i][0][1] > ALLOWED_DEVIATION or z_scores[i][0][1] < -ALLOWED_DEVIATION) :
                np.delete(p0, i)


    def filter_box_outliers(self, Id):
        person = self.people[Id]
        box = person.box

        for i in range(len(person.p0)-1, -1, -1):
            pt = person.p0[i][0]
            if (pt[0] < box[0] or pt[0] > box[2]) or (pt[1] < box[1] or pt[1] > box[3]): # outside bounding box
                np.delete(person.p0, i)



def movement_vector(cor1, cor2):
    cor1, cor2 = np.array(cor1), np.array(cor2)
    vec = cor1 - cor2
    return vec


    


#TODO: add TeamDetector class (integrate color treshold/blob detection algorithm)




if __name__ == "__main__":
    
    vCap = Cam.vCap(0)
    
    frame = vCap.read()
    
    tracker = Tracker(frame)


    teams_config = team_json_loader.TeamConfig()
    # teams_config.
    

    if servo_loaded:
        controller = servoController.Controller(frame.shape, 0, 1, 2)
        controller.yServo.setVal(0)


    # MAIN LOOP ------------
    iteration = 0
    while True:
        capture_t = time.time()
        frame = vCap.read()
        draw_frame = frame.copy()

        if iteration % 600 == 0:
            tracker.find_people(frame)
            tracker.update_lk_features(frame, force_update=True)

        if iteration %50 == 0:
            tracker.cleanup_outliers()

            tracker.update_boxes()

        tracker.update_lk_features(frame)

        tracker.run_lk(frame, draw_frame)
        
        # tracker.run_yolo(tracker.frame)
        # lk.run(tracker.frame, draw_frame)


        closest_enemy = tracker.find_closest_enemy(["Unknown"])

        if servo_loaded and closest_enemy is not None:

            controller.aim(tracker.crosshair_pos, closest_enemy.center_of_mass, time.time()-capture_t)



        cv.imshow("window", draw_frame)
        cv.waitKey(1)
        iteration += 1