#import RPi.GPIO as GPIO
import time
import math
import threading
import queue
import sys
sys.path.append('../topTurret')
from packages.rpiControll import gyro
from packages import pico_serial_pwm


class Controller():
  def __init__(self,imShape,xServoPin,yServoPin,triggerPin,serial_port='/dev/ttyACM0'):
    self.FOV = [41,66]
    self.min_max = [0] #TODO max
    self.pxDeg = [self.FOV[i] for i in range(2)]

    self.serial_controller = pico_serial_pwm.Pico_serial_pwm(serial_port)

    self.xServo = Servo360(xServoPin, self.serial_controller)
    self.yServo = Servo180(yServoPin, self.serial_controller)
    self.trigger = PWM(triggerPin, self.serial_controller)


  def computeAngles(self,coor1,coor2):
    angle = [(coor1[i]-coor2[i])*self.pxDeg[i] for i in range(2)]
    return angle
  
  def aim(movementVec):
    pass

  def shoot(self):
    pass

  def controllThread(self):
    self.trigger.setVal(1)
    

class PWM():
  def __init__(self, pin, serial_controller, pwmRange=[0,65535]):
    self.pin = pin
    self.pwmRange = pwmRange

    self.pwmController = serial_controller0
  
  def setVal(self,value):
    self.pwmController.set_duty_cycle(self.pin, self.val2pw(value))

  def setValRaw(self, value):
    self.pwmController.set_duty_cycle(self.pin, value) 

  def val2pw(self,val):
    '''- val: value in range [-1,1], where -1 represents minimum pulse width and 1 maximum'''
    return self.pwmRange[0]+(self.pwmRange[1]-self.pwmRange[0])*(val+1)/2
  
  def stop(self):
    '''stop servo from moving and readjusting itself'''
    self.pwmController.set_duty_cycle(self.pin, 0)



class Servo180(PWM):
  def __init__(self, pin, serial_controller, pwm_gap_range=[4700, 5070], pwmRange=[0,65535]):
    super().__init__(pin, serial_controller, pwmRange)

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




class Servo360(PWM):
  def __init__(self, pin, serial_controller, pwm_gap_range=[4700, 5070], pwmRange=[0,65535]):
    super().__init__(pin, serial_controller, pwmRange)
    self.angleQ = queue.Queue()
    self.controllThread = threading.Thread()
    self.controllThread.start()
    #TODO undomment \/
    #self.gyro = gyro.mpu()

  def rotateDeg(self, degrees, absolute=True):
    self.angleQ.put(degrees)

  def __controllFunc(inQueue, outQueue):
    pass





#TODO REMOVE

cont = Controller([1,1], 3, 1, 0)
