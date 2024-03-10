#import RPi.GPIO as GPIO
import time
import math
import threading
import queue
import sys
sys.path.append('../topTurret')
from packages.rpiControll import gyro
from packages import pi5PWM


class Controller():
  def __init__(self,imShape,xServo_pin,yServo_pin,trigServo_pin):
    self.FOV = [41,66]
    self.pxDeg = [self.FOV[i] for i in range(2)]

    self.xServo = Servo360(xServo_pin)
    self.yServo = Servo180(yServo_pin)
    self.trigServo = Servo180(trigServo_pin)

  def computeAngles(self,coor1,coor2):
    angle = [(coor1[i]-coor2[i])*self.pxDeg[i] for i in range(2)]
    return angle
  
  def aim(movementVec):
    pass

  def shoot(self):
    pass
    

class Servo():
  def __init__(self, pin, pwmRange=[500,2500]):
    '''
    - pin: pin on which the servo PWM wire is connected, HAS TO SUPPORT HW PWM, pwm supporting pins: 12,13,14,15,18,19
    - pwmRange: `[lowest pulse width (lowest angle), highest pulse width (highest angle)]`
    '''
    self.pin = pin
    self.pwmRange = pwmRange
    self.pwmController = pi5PWM.pi5RC(pin)
  
  def setVal(self,value):
    self.pwmController.setDutyCycle(self.val2pw(value))

  def val2pw(self,val):
    '''- val: value in range [-1,1], where -1 represents minimum pulse width and 1 maximum'''
    return self.pwmRange[0]+(self.pwmRange[1]-self.pwmRange[0])*(val+1)/2
  
  def stop(self):
    '''stop servo from moving and readjusting itself'''
    self.pwmController.setDutyCycle(0)



class Servo180(Servo):
  def __init__(self, pin, pwmRange):
    super().__init__(pin, pwmRange)

  def rotateDeg(self, degrees, absolute=True):
    if absolute:
      self.angle = degrees
    else:
      if self.min_angle > self.angle+degrees:
        self.angle = self.min_angle

      elif self.angle+degrees < self.max_angle: #//Tomas was here
        self.angle = self.max_angle

      else:
        self.angle = self.angle + degrees




class Servo360(Servo):
  def __init__(self, pin, pwmRange):
    super().__init__(pin, pwmRange)
    self.angleQ = queue.Queue()
    self.controllThread = threading.Thread()
    self.controllThread.start()

  def rotateDeg(self, degrees, absolute=True):
    self.angleQ.put(degrees)

  def __controllFunc(inQueue, outQueue):
    pass

