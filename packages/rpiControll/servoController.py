#import RPi.GPIO as GPIO
import time
import math
import threading
import queue
#import gyro



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
  def __init__(self):
    pass

class Servo180(Servo): #/TODO: REWORK SERVO OBJECTS TO WORK WITH HARDWARE PWM


  def rotateDeg(self, degrees, absolute=True):
    if absolute:
      self.angle = degrees
    else:
      if self.min_angle > self.angle+degrees:
        self.angle = self.min_angle

      elif self.angle+degrees < self.max_angle:
        self.angle = self.max_angle

      else:
        self.angle = self.angle + degrees




class Servo360(Servo):
  def rotateDeg(self, degrees, absolute=True):
    self.angleQ.put(degrees)

  def controllFunc(inQueue, outQueue):
    pass

  def __init__(self, pin=None, *, initial_value=0, min_pulse_width=1 / 1000, max_pulse_width=2 / 1000, frame_width=20 / 1000, pin_factory=None):

    super().__init__(pin, initial_value=initial_value, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width, frame_width=frame_width, pin_factory=pin_factory)

    self.angleQ = queue.Queue()
    self.controllThread = threading.Thread()
    self.controllThread.start()
