import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages import pi5PWM

import random

servo1 = pi5PWM.pi5RC(18)
servo2 = pi5PWM.pi5RC(19)
servo3 = pi5PWM.pi5RC(12)

pwRange = [500, 2500]

while True:
    servo1.setDutyCycle(random.randint(pwRange[0],pwRange[1]))
    print("servo1")
    time.sleep(0.1)
    servo2.setDutyCycle(random.randint(pwRange[0],pwRange[1]))
    print("servo2")
    time.sleep(0.1)
    servo3.setDutyCycle(random.randint(pwRange[0],pwRange[1]))
    print("servo3")
    time.sleep(0.1)
    
