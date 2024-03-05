import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages import pi5PWM

servo = pi5PWM.pi5RC(18)
servo.enable(True)

while True:
    dutyCycle = int(input("pwm> "))
    servo.setDutyCycle(dutyCycle)