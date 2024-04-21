import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages import pi5PWM
from packages.rpiControll import servoController

import random


servo1 = pi5PWM.pi5RC(18)
'''
servo2 = pi5PWM.pi5RC(19)
servo3 = pi5PWM.pi5RC(12)
'''
#servo1 = servoController.Servo(18)

#pwRange = [500, 2500]

servo1.set_duty(2300)
time.sleep(0.5)
servo1.set_duty(0)