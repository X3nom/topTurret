from threading import Thread
import cv2 as cv

class vCap():
    def __init__(self,source):
        self.size = [1280,720]
        self.mode = "pc"
        if source == 0:
            try: from picamera2 import Picamera2
            except: self.mode = "pc"
            else: self.mode = "rpi"

        if self.mode == "rpi": # if the code is running on raspberry pi
            self.vcap = Picamera2()
            self.vcap.preview_configuration.main.size = self.size
            self.vcap.preview_configuration.main.format = "RGB888"
            self.vcap.preview_configuration.align()
            self.vcap.configure("preview")
            self.vcap.start()
        else: # the code is running on machine other than raspberry
            self.vcap = WebcamStream(source)
            self.vcap.start()
            self.size = [ int(self.vcap.vcap.get(cv.CAP_PROP_FRAME_WIDTH )),int(self.vcap.vcap.get(cv.CAP_PROP_FRAME_HEIGHT )) ]

    def read(self):
        if self.mode == "rpi":
            return self.vcap.capture_array()
        else:
            return self.vcap.read()
    def isOpen(self):
        if self.mode == "rpi": pass
        else:
            return self.vcap.vcap.isOpened()


class WebcamStream : #credits to https://github.com/vasugupta9 (https://github.com/vasugupta9/DeepLearningProjects/blob/main/MultiThreadedVideoProcessing/video_processing_parallel.py)
    def __init__(self, stream_id=0): 
        self.stream_id = stream_id   # default is 0 for primary camera 
        
        # opening video capture stream 
        self.vcap      = cv.VideoCapture(self.stream_id)
        if self.vcap.isOpened() is False :
            print("[Exiting]: Error accessing webcam stream.")
            exit(0)
        fps_input_stream = int(self.vcap.get(5))
        print("FPS of webcam hardware/input stream: {}".format(fps_input_stream))
            
        # reading a single frame from vcap stream for initializing 
        self.grabbed , self.frame = self.vcap.read()
        if self.grabbed is False :
            print('[Exiting] No more frames to read')
            exit(0)

        # self.stopped is set to False when frames are being read from self.vcap stream 
        self.stopped = True 

        # reference to the thread for reading next available frame from input stream 
        self.t = Thread(target=self.update, args=())
        self.t.daemon = True # daemon threads keep running in the background while the program is executing 
        
    # method for starting the thread for grabbing next available frame in input stream 
    def start(self):
        self.stopped = False
        self.t.start() 

    # method for reading next frame 
    def update(self):
        while True :
            if self.stopped is True :
                break
            self.grabbed , self.frame = self.vcap.read()
            if self.grabbed is False :
                print('[Exiting] No more frames to read')
                self.stopped = True
                break 
        self.vcap.release()

    # method for returning latest read frame 
    def read(self):
        return self.frame

    # method called to stop reading frames 
    def stop(self):
        self.stopped = True
