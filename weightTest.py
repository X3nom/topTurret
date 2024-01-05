import cv2 as cv
from ultralytics import YOLO
from torch import nn
from torch.nn import functional as F

model = YOLO(r".\Yolo_weights\v8_s.pt")
model.predict(source="0", show=True, stream=True, classes=0)

cap = cv.VideoCapture(0)

while True:
    success, frame = cap.read()
    detected = model(frame,stream=False)
    #cv.imshow("frame",frame)
    #cv.waitKey(1)