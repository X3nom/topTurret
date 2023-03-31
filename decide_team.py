import cv2 as cv
import numpy as np
from ultralytics import YOLO
from packages import sort

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
    

class Team():
    def __init__(self,name,upper_color,lower_color,display_color=(255,0,255)) -> None:
        self.name = name
        self.upper_color = upper_color
        self.lower_color = lower_color
        self.display_color = display_color
        

model = YOLO('.\Yolo weights\yolov8l.pt') # load up neural network model

tracker = sort.Sort(30,1)

colorless_playing = False # True = FORCE DETECTION OF COLORLESS TEAM !
people = np.empty((0,5))
color = (0,0,255)

cap = cv.VideoCapture(r'C:\Users\Jakub\Programming\Python\openCV\samples\randalls squad sample.mp4') # <--- set video capture

all_teams = [ # \/ add/change teams  \/ --------------------------------------------------------
    Team('Unknown', np.array([0,0,0]), np.array([255,255,255]), (0,255,0)), #used for people not matching description of any other team, !-DO NOT CHANGE OR REMOVE-!

    Team('colorless', np.array([0,0,0]), np.array([255,255,255]), (255,0,255)),#special team with invalid color range, only if team with no color is playing

    Team('blue', np.array([123,255,191]), np.array([106,174,52]), (255,0,0)),
    Team('red', np.array([179,255,255]), np.array([162,169,106]), (0,0,255)),
    Team('yellow', np.array([29,255,255]), np.array([18,165,89]), (0,255,255))
]
playing_teams = ['red','blue'] # EDIT ALL PLAYING TEAMS !
enemy_teams = ['blue'] # EDIT ENEMY TEAMS !

teams = []
teams.append(all_teams[0])
for et in playing_teams:
    for t in all_teams:
        if t.name == et:
            teams.append(t)
if 'colorless' in playing_teams:
    colorless_playing = True
ids = Ids(teams,colorless_playing)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True: # Main loop
    ret, frame = cap.read()
    if not ret:
        break
    
    detection = model(frame,stream=True) #detect objects in frame trough neural network

    frame_out = np.copy(frame)
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

    for res in tracker_return:
        x1,y1,x2,y2,Id = res
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        center = [round(x1+abs(x1-x2)/2),round(y1+abs(y1-y2)/2)]

        person = frame[y1:y2,x1:x2]
        if len(person) > 0 and all(i > -1 for i in [x1,y1,x2,y2]): #check if cordinates of person are valid, othervise empty selections or negative cordinates can cause openCV error
            #find color matches with defined teams
            hsv_person = cv.cvtColor(person,cv.COLOR_BGR2HSV)
            mask_sums = []
            for team in teams:
                mask = cv.inRange(hsv_person,team.lower_color,team.upper_color)
                mask_sums.append(np.sum(mask))
            if max(mask_sums) > 15:
                best_team_match = teams[mask_sums.index(max(mask_sums))]
            else:
                best_team_match = all_teams[0]

            person_team = ids.check_id(Id,best_team_match)
            color = person_team.display_color
            
            # graphics for visual validation of data
            cv.rectangle(frame_out,(x1,y1),(x2,y2),color,2)
            cv.drawMarker(frame_out,center,color,cv.MARKER_CROSS,thickness=2)
            cv.putText(frame_out,person_team.name,np.array([x1+10,y1-10]),cv.FONT_HERSHEY_SIMPLEX,1,color,2,cv.LINE_AA)
            cv.putText(frame_out,str(int(Id)),np.array([x1,y2-10]),cv.FONT_HERSHEY_SIMPLEX,1,color,2,cv.LINE_AA)

    ids.update()
    cv.imshow("test",frame_out)
    cv.waitKey(1)
