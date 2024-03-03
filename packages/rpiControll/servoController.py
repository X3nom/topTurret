#import RPi.GPIO as GPIO
import time
import math
import threading
import queue

try:
  import gpiozero
  from gyro import mpu
except:
  print("ServoController WARNING: enviroment is not setup properly, servos will not work")
  raise MissingLibrary


class MissingLibrary(Exception):
  pass


class Controller():
  def __init__(self,imShape,xServo_pin,yServo_pin,trigServo_pin):
    self.FOV = [41,66]
    self.pxDeg = []

    self.xServo = Servo360(xServo_pin)
    self.yServo = Servo180(yServo_pin)
    self.trigServo = Servo180(trigServo_pin)

  def computeAngles(self,coor1,coor2):
    angle = [(coor1[i]-coor2[i])*self.pxDeg[i] for i in range(2)]
    return angle
  
  def aim(movement):
    pass

  def shoot(self):
    pass

  def stopAndExit(self):
    self.xServo.stop()
    self.yServo.stop()
    self.trigServo.stop()
    


class Servo():
  def __init__(self,servoPIN):
    self.valRange = [2,12]
    GPIO.setup(servoPIN, GPIO.OUT)
    self.pin = GPIO.PWM(servoPIN, 50)

    if self.type == '180': 
      self.curRot = 2
    else:
      self.curRot = 0

    self.pin.start(self.curRot)

  def stop(self):
    self.pin.stop()

  def changeCycle(self,value,add=False):
    if add:
      tempRot = self.curRot + value
      if tempRot < self.valRange[0]:
        tempRot == self.valRange[0]
      elif tempRot > self.valRange[1]:
        tempRot == self.valRange[1]
      self.curRot = tempRot
    else:
      self.curRot = value
    self.pin.ChangeDutyCycle(self.curRot)

  def rotateDeg(self,degrees,absolute=True):
    pass


class Servo180(Servo):
  def rotateDeg(self, degrees, absolute=True):
    signal = degrees/36*2+2
    self.changeCycle()


class Servo360(Servo):
  def rotateDeg(self, degrees, absolute=True):
    pass

  def controllFunc(inQueue, outQueue):
    pass

  def __init__(self, servoPIN):
    super().__init__(servoPIN)
    self.angleQ = queue.Queue()
    self.controllThread = threading.Thread()
    self.controllThread.start()