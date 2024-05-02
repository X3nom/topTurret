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
    self.FOV = [41,66] #deg
    self.min_max = [0] #TODO max
    self.pxDeg = [self.FOV[i]/imShape[i] for i in range(2)] #degres/pixel

    self.serial_controller = pico_serial_pwm.Pico_serial_pwm(serial_port)

    self.xServo = Servo360(xServoPin, self.serial_controller)
    self.yServo = Servo180(yServoPin, self.serial_controller)
    self.trigger = PWM(triggerPin, self.serial_controller)

  def computeAngles(self,coor1,coor2):
    angle = [(coor1[i]-coor2[i])*self.pxDeg[i] for i in range(2)]
    return angle
  
  def aim(self, movement_px_vec): # [x,y]
    movement_deg_vec = [movement_px_vec[i]*self.pxDeg[i] for i in range(2)]

  def shoot(self):
    pass
    



class PWM():
  def __init__(self, pin, serial_controller, pwmRange=[0,65535]):
    self.pin = pin
    self.pwmRange = pwmRange
    self.val = 0
    self.pwmController = serial_controller
  
  def setVal(self,value, absolute=True):
    if absolute:
      self.val = value

    else:
      self.val += value
      if self.val > 1: self.val = 1
      elif self.val < -1: self.val = -1

    self.pwmController.set_duty_cycle(self.pin, self.val2pw(self.val))


  def setValRaw(self, value):
    self.pwmController.set_duty_cycle(self.pin, value) 

  def val2pw(self,val):
    '''- val: value in range [-1,1], where -1 represents minimum pulse width and 1 maximum'''
    return self.pwmRange[0]+(self.pwmRange[1]-self.pwmRange[0])*(val+1)/2
  
  def stop(self):
    '''stop servo from moving and readjusting itself'''
    self.pwmController.set_duty_cycle(self.pin, 0)
    self.val = -2





class Servo180(PWM):
  def __init__(self, pin, serial_controller, pwmRange=[3700, 6070]):
    super().__init__(pin, serial_controller, pwmRange)
    self.angle_range = [0,180]
    self.angle = (self.angle_range[1]-self.angle_range[0])/2



  def rotateDeg(self, degrees, absolute=True):
    if absolute:
      self.angle = degrees
    else:
      if self.angle_range[0] > self.angle+degrees:
        self.angle = self.angle_range[0]

      elif self.angle_range[1] < self.angle+degrees:
        self.angle = self.angle_range[1]

      else:
        self.angle = self.angle + degrees


  def deg2val(self, degress):
    val = degress/(self.angle_range[1] - self.angle_range[0])



class Servo360(PWM):
  def __init__(self, pin, serial_controller, pwm_range_L=[3700, 4700], pwm_range_R=[5070, 6070]):
    super().__init__(pin, serial_controller, [pwm_range_L[0], pwm_range_R[1]])
    self.angleQ = queue.Queue()
    self.controllThread = threading.Thread(target=self.gyroThread)
    self.controllThread.start()

    self.l_range = pwm_range_L
    self.r_range = pwm_range_R


  def rotateDeg(self, degrees, absolute=True):
    self.angleQ.put(degrees)
  

  def val2pw(self, val):
    if val > 0: #positive rotation (counter-clockwise)
      return self.l_range[1]-(self.l_range[1]-self.l_range[0])*abs(val)
    else: #negative rotation (clockwise)
      return self.r_range[0]+(self.r_range[1]-self.r_range[0])*abs(val)
  

  def setVal(self, val):
    if self.val != val:
      self.val = val
      self.pwmController.set_duty_cycle(self.pin, self.val2pw(val))

      


  def gyroThread(self):
    ROTATING_AXIS = 2
    ROTATION_ERROR = 1

    mpu = gyro.mpu()
    last_calibration_time = time.time()

    current_rotation = 0

    rotating = False

    last_time = time.time()

    while True:
      if rotating:
        try:
          angle = self.angleQ.get_nowait()
          current_rotation = 0
        except: pass

      else:
        if time.time() - last_calibration_time > 60:
          mpu.zeroGyro() # recalibrate after 1 minute
          last_calibration_time = time.time()

        print("waiting")
        angle = self.angleQ.get()

        current_rotation = 0
        rotating = True

      rotation_rate = mpu.getGyroZData()[ROTATING_AXIS]
      delta_time = time.time() - last_time
      delta_rotation = rotation_rate * delta_time
      last_time = time.time()
      
      current_rotation += delta_rotation



      print(angle, current_rotation)
      
      if angle - ROTATION_ERROR < current_rotation < angle + ROTATION_ERROR: # desired angle reached
        self.stop()
        rotating = False


      elif current_rotation < angle:
        self.setVal(-1)

      else:
        self.setVal(1)



#TODO REMOVE

cont = Controller([1,1], 0, 1, 2)

while True:
  cont.xServo.rotateDeg(int(input(">")))