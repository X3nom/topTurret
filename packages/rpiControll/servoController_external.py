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
    self.FOV = [66, 41] #deg
    self.min_max = [0] #TODO max
    self.pxDeg = [self.FOV[i]/imShape[i] for i in range(2)] #degres/pixel

    self.serial_controller = pico_serial_pwm.Pico_serial_pwm(serial_port)
    # self.serial_controller.init_threaded()

    self.xServo = Servo360(xServoPin, self.serial_controller)
    self.yServo = Servo180(yServoPin, self.serial_controller, [-30,30])
    self.trigger = PWM(triggerPin, self.serial_controller)

  
  def aim(self, crosshair_coor, target_coor, measure_delta_time): # [x,y]
    movement_px_vec = [crosshair_coor[i] - target_coor[i] for i in range(2)]
    movement_deg_vec = [movement_px_vec[i]*self.pxDeg[i] for i in range(2)]

    # perform rotation
    self.xServo.rotateDeg_compensated(movement_deg_vec[0], measure_delta_time)
    self.yServo.rotateDeg(movement_deg_vec[1]*-1)


  def shoot(self):
    self.trigger.pulseVal(1, 0.3)
    



class PWM():
  def __init__(self, pin, serial_controller, pwmRange=[0,65535]):
    self.pin = pin
    self.pwmRange = pwmRange
    self.val = 0
    self.pwmController = serial_controller

  def __del__(self):
    self.stop()

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
  
  def pulseVal(self, val, pulse_time):
    thread = threading.Thread(target=self._pulse_thread, args=(val, pulse_time))
    thread.start()

  def _pulse_thread(self, val, pulse_time):
    self.setVal(val)
    time.sleep(pulse_time)
    self.stop()





class Servo180(PWM):
  def __init__(self, pin, serial_controller, angle_range=[0,180], pwmRange=[3700, 6070]):
    super().__init__(pin, serial_controller, pwmRange)
    self.angle_range = angle_range #[0,180]
    self.angle = (self.angle_range[1]-self.angle_range[0])/2
    
    self.rotating = False



  def rotateDeg(self, degrees, absolute=False):
    if absolute:
      self.angle = degrees
    else:
      self.angle = self.angle + degrees

    if self.angle_range[0] > self.angle+degrees:
      self.angle = self.angle_range[0]
    elif self.angle_range[1] < self.angle+degrees:
      self.angle = self.angle_range[1]

    self.setVal(self.deg2val(self.angle))

    # TODO: rotateDeg_compensated


  def deg2val(self, degress):
    val = ((degress-self.angle_range[0])/(self.angle_range[1] - self.angle_range[0]))*2-1
    return val




class Servo360(PWM):
  def __init__(self, pin, serial_controller, pwm_range_L=[3700, 4700], pwm_range_R=[5070, 6070]):
    super().__init__(pin, serial_controller, [pwm_range_L[0], pwm_range_R[1]])
    self.angleQ = queue.Queue()

    self.l_range = pwm_range_L
    self.r_range = pwm_range_R

    self.rotating = False
    self.rotation_stop_time = 0
    self.current_rotation = 0

    self.controllThread = threading.Thread(target=self.gyroThread)
    self.controllThread.start()


  def rotateDeg(self, degrees):
    self.angleQ.put(degrees)


  def rotateDeg_compensated(self, degrees, capture_time):
    if self.rotating or (not self.rotating and capture_time < self.rotation_stop_time):
      degrees -= self.current_rotation

      self.rotateDeg(degrees)

    else:
      self.rotateDeg(degrees)

  

  def val2pw(self, val):
    if val > 0: #positive rotation (counter-clockwise)
      return self.l_range[1]-(self.l_range[1]-self.l_range[0])*abs(val)
    elif val < 0: #negative rotation (clockwise)
      return self.r_range[0]+(self.r_range[1]-self.r_range[0])*abs(val)
  

  def setVal(self, val):
    if self.val != val:
      self.val = val
      self.pwmController.set_duty_cycle(self.pin, self.val2pw(val))

      


  def gyroThread(self):
    ROTATING_AXIS = 1
    ROTATION_ERROR = 1

    mpu = gyro.mpu()
    last_calibration_time = time.time()

    self.current_rotation = 0

    last_time = time.time()

    while True:
      if self.rotating:
        try:
          raise #TODO: REMOVE
          angle = self.angleQ.get_nowait()
          self.current_rotation = 0
        except: pass

      else:
        if time.time() - last_calibration_time > 180:
          time.sleep(0.5)
          mpu.zeroGyro() # recalibrate after 1 minute
          last_calibration_time = time.time()

        angle = self.angleQ.get()

        self.current_rotation = 0
        self.rotating = True

      rotation_rate = mpu.getGyroZData()[ROTATING_AXIS]
      delta_time = time.time() - last_time
      delta_rotation = rotation_rate * delta_time
      last_time = time.time()
      
      self.current_rotation += delta_rotation



      
      if angle - ROTATION_ERROR < self.current_rotation < angle + ROTATION_ERROR: # desired angle reached
        self.stop()
        self.rotating = False
        self.rotation_stop_time = time.time()


      elif self.current_rotation < angle:
        self.setVal(-0.25)

      else:
        self.setVal(0.25)

