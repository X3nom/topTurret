import cv2 as cv
import numpy as np
from ultralytics import YOLO
from threading import Thread
import time
from packages import sort
from packages.rpiControll import Cam
from packages import team_json_loader
from packages.rpiControll import servoController_external


class Id_team(): #associate id with team
    def __init__(self,Id,team=None,all_teams=[],countdown=30,ttit=30) -> None:
        self.Id = Id
        self.team = team
        self.teams = all_teams
        self.teams_values = [ 0 for i in all_teams]
        self.all_teams = all_teams
        self.countdown = countdown # number of frames in row where id is not found, object gets deleted after reaching 0
        self.max_countdown = countdown
        self.ttit = ttit # "time to identify team" if colorless team is playing, this is cooldown till Unknown player is marked as colorless player
        self.max_ttit = ttit
        self.switched_to_colorless = False
    def update_team(self,team,colorless_playing):
        if self.ttit <= 0 and not self.switched_to_colorless:
            for t in self.teams:
                if t.name == 'Unknown':
                    self.teams[self.teams.index(t)] = self.all_teams[0]
        if team.name == 'Unknown':
            if colorless_playing:
                if not self.switched_to_colorless:
                    self.teams_values[self.teams.index(team)] += 1
                else:
                    self.teams_values[1] += 1
                self.team = self.teams[self.teams_values.index(max(self.teams_values))]
        elif self.team.name == 'Unknown' and not colorless_playing:
            self.team = team
            self.teams_values[self.teams.index(team)] += 1
        else:
            self.teams_values[self.teams.index(team)] += 1
            self.team = self.teams[self.teams_values.index(max(self.teams_values))]
        self.countdown = self.max_countdown



class Ids():
    def __init__(self,teams,colorless_playing=False) -> None:
        self.ids = []
        self.updated = []
        self.colorless_playing = colorless_playing
        self.teams = teams

    def get_id_from_ids(self,wanted_id):
        for id in self.ids:
            if id.Id == wanted_id:
                return id
        return None
            
    def check_id(self,id_to_check,team):
        id = self.get_id_from_ids(id_to_check)
        if id != None:
            id.update_team(team,self.colorless_playing)
            self.updated.append(id.Id)
            return id.team
        self.ids.append(Id_team(id_to_check,team,self.teams))
        self.updated.append(id_to_check)
        return team

    def update(self):
        to_pop = []
        for id in self.ids:
            if id.countdown <= 0:
                to_pop.append(id.Id)
            if id.Id not in self.updated:
                id.countdown -= 1
            if id.team.name == 'Unknown' and self.colorless_playing and id.ttit > 0:
                id.ttit -= 1
            elif id.team.name != 'colorless':
                id.ttit = id.max_ttit
        for id_to_pop in to_pop:
            self.ids.pop(self.ids.index(self.get_id_from_ids(id_to_pop)))
    


class Team(): #class containing info about what clolor range of armband is associated to which team name
    def __init__(self,name,upper_color,lower_color,display_color=(255,0,255)) -> None:
        self.name = name #team name
        self.upper_color = upper_color # brightest/highest color shade that is recognized as teams armband (numpy array, color has to be in VHS format)
        self.lower_color = lower_color # darkest/lowest color shade that is recognized as teams armband (numpy array, color has to be in VHS format)
        self.display_color = display_color # color of player border (mainly for debuging purposes)


def team_from_dict(dict) -> Team:
    return Team(dict["name"], np.array(dict["upper"]), np.array(dict["lower"]))
    


def find_closest_enemy(enemies,crosshair_coor):
    if len(enemies) > 0:
        centers = []
        for enemy in enemies:
            x1,y1,x2,y2,Id = enemy
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            center = [round(x1+abs(x1-x2)/2),round(y1+abs(y1-y2)/2)]
            centers.append(center)
        closest_center = centers[0]
        closest_center_dist = np.sqrt(abs(closest_center[0]-crosshair_coor[0])**2+abs(closest_center[1]-crosshair_coor[1])**2)
        for center in centers:
            if np.sqrt(abs(center[0]-crosshair_coor[0])**2+abs(center[1]-crosshair_coor[1])**2) < closest_center_dist:
                closest_center = center

        return closest_center, enemies[centers.index(closest_center)]



def attack_enemy(servo_controller, last_frame_time, enemy, crosshair_coor):
    KILL_DISTANCE = 0.5 # how close to center to shoot (portion of detection)

    #TODO: fix turret pointing down

    x1,y1,x2,y2,Id = enemy
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    enemy_center = [round(x1+abs(x1-x2)/2),round(y1+abs(y1-y2)/4)] # center not in center bc servo is retarded

    dist = np.sqrt(abs(enemy_center[0]-crosshair_coor[0])**2+abs(enemy_center[1]-crosshair_coor[1])**2)

    if servo_controller is None: return
    if dist <= abs(x1-x2)*KILL_DISTANCE and dist <= abs(y1-y2)*KILL_DISTANCE: # target in acceptable range to shoot
        # servo_controller.shoot()
        print("\033[33mSHOOT!\033[0m")
        pass

    else: #target not aimed at, perform aiming
        servo_controller.aim(crosshair_coor, enemy_center, last_frame_time)









# SETUPS
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
run_visual = False

# capture setup ----------------

capture = 0#r"C:/Users/Jakub/Programming/Python/openCV/samples/randalls squad sample.mp4" # <--- set video capture (source)

cap = Cam.vCap(capture)

cap_size = cap.size

screencenter = [round(cap_size[0]/2),round(cap_size[1]/2)]

crosshair_coor = screencenter

# capture setup =========
# tracking setup --------------

model = YOLO('./Yolo_weights/yolov8n.pt') # load up neural network model

tracker = sort.Sort(30,1)

# tracking setup =======
# teams setup ----------------

team_config = team_json_loader.TeamConfig()

colorless_playing = False # True = FORCE DETECTION OF COLORLESS TEAM !
people = np.empty((0,5))
color = (0,0,255)

teams = []
teams.append( Team('Unknown', np.array([0,0,0]), np.array([255,255,255]), (0,255,0)) )

for tm in [*team_config.get_enemy(), *team_config.get_friendly()]:
    teams.append(team_from_dict(tm))

enemy_teams = team_config.teams_dict["enemy"]


ids = Ids(teams,colorless_playing)

# teams setup ========
# servo setup ---------

try:
    servo_controller = servoController_external.Controller(cap_size, 0, 1, 2)
    servo_controller.yServo.setVal(-0.5)
except:
    print("ERROR: servo controller could not be loaded, running without servos!!")
    servo_controller = None

# servo setup =======
# ENTERING MAIN LOOP ----------------------------------------------------------------------------------------------------------------------------------------------------------

if cap.mode == 'pc' and not cap.isOpen():
    print("Cannot open camera")
    exit()

last_frame_time = time.time()

while True: # Main loop !!!!!!
    last_frame_time = time.time()

    frame = cap.read() #get frame from camera
    
    detection = model(frame,stream=True) #detect objects in frame trough neural network

    if run_visual: frame_out = np.copy(frame)

    people = np.empty((0,5))

    #find people in detected objects
    for detected in detection: 
        boxes = detected.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            if int(box.cls[0]) == 0 and len(frame[y1:y2,x1:x2]) > 0:
                person_arr = np.array([x1,y1,x2,y2,box.conf[0].cpu().numpy()]) #get data about detection into format required by tracker
                people = np.vstack((people,person_arr))


    tracker_return = tracker.update(people) #sends data about detections to sort, sort tryes to associate people from previous frames with new detections gives them IDs
    enemies = np.empty((0,5))
    for res in tracker_return:
        x1,y1,x2,y2,Id = res
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        center = [round(x1+abs(x1-x2)/2),round(y1+abs(y1-y2)/2)]

        person = frame[y1:y2,x1:x2] #cut out section of frame containing detected person

        if len(person) > 0 and all(i > -1 for i in [x1,y1,x2,y2]): #check if cordinates of person are valid, othervise empty selections or negative cordinates can cause openCV error
            #find color matches with defined teams
            hsv_person = cv.cvtColor(person,cv.COLOR_BGR2HSV)
            mask_sums = []
            for team in teams:
                mask = cv.inRange(hsv_person,team.lower_color,team.upper_color)
                mask_sums.append(np.sum(mask))
            if max(mask_sums) > 15:     #//TODO: implement blob detection for higher detection accuracy
                best_team_match = teams[mask_sums.index(max(mask_sums))]
            else: #not matched with any team
                best_team_match = teams[0]

            person_team = ids.check_id(Id,best_team_match)
            color = person_team.display_color

            if person_team.name in enemy_teams:
                enemies = np.vstack((enemies,res))
            
            if run_visual:
                # graphics for visual validation of data
                cv.rectangle(frame_out,(x1,y1),(x2,y2),color,2)
                cv.drawMarker(frame_out,center,color,cv.MARKER_CROSS,thickness=2)
                cv.putText(frame_out,person_team.name,np.array([x1+10,y1-10]),cv.FONT_HERSHEY_SIMPLEX,1,color,2,cv.LINE_AA)
                cv.putText(frame_out,str(int(Id)),np.array([x1,y2-10]),cv.FONT_HERSHEY_SIMPLEX,1,color,2,cv.LINE_AA)

                cv.drawMarker(frame_out,screencenter,(255,0,255),cv.MARKER_CROSS,50,2)


    if len(enemies) > 0: # aim at closest enemy to crosshair
        closest_center, closest_enemy = find_closest_enemy(enemies, crosshair_coor)

        attack_enemy(servo_controller, last_frame_time, closest_enemy, crosshair_coor)

        if run_visual:
            cv.line(frame_out,closest_center,screencenter,(255,0,255),2,cv.LINE_AA)
            cv.drawMarker(frame_out,closest_center,ids.get_id_from_ids(closest_enemy[4]).team.display_color,cv.MARKER_SQUARE,thickness=2)


    ids.update()
    if run_visual:
        cv.imshow("test",frame_out)
        cv.waitKey(1)
