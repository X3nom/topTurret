import cv2 as cv
import numpy as np
from ultralytics import YOLO
from threading import Thread
import time
from packages import sort
from packages.rpiControll import Cam
from packages import team_json_loader

try: from packages.rpiControll import servoController_external as servoController
except: print("servo controll could not be loaded")





class VideoProcessor():
    def __init__(self, vcap, model='./Yolo_weights/yolov8n.pt') -> None:
        self.vCap = Cam.vCap(vcap)
        self.model = YOLO(model) # load up neural network model

        self.prev_frame = self.vCap.read()
        self.frame = self.vCap.read()


    def next_frame(self): # advance to next frame
        self.prev_frame = self.frame.copy()
        self.frame = self.vCap.read()


    def find_movement(self):
        pass


    def find_people():
        pass


    def track():



if __name__ == "__main__":

    
    vid_process = VideoProcessor(0)





    teams_config = team_json_loader.TeamConfig()
    # teams_config.
    

    # MAIN LOOP ------------
    while True:
        vid_process.next_frame()

        

        cv.imshow("window", frame)
        cv.waitKey(1)