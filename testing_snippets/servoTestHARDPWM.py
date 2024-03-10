import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages import pi5PWM
from packages.rpiControll import servoController

import random

'''
servo1 = pi5PWM.pi5RC(18)
servo2 = pi5PWM.pi5RC(19)
servo3 = pi5PWM.pi5RC(12)
'''
servo1 = servoController.Servo(18)

pwRange = [500, 2500]

while True:
    inp = float(input("val: "))
    servo1.setVal(inp)
