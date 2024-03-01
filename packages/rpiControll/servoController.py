#import RPi.GPIO as GPIO
import time
import math
import socket
import threading
import queue

class Controller():
  def __init__(self,imShape,xServo_pin,yServo_pin,trigServo_pin):
    try: import RPi.GPIO as GPIO
    except: 
      self.mode = 'pc'
      return
    else: self.mode = 'rpi'

    self.FOV = [41,66]
    self.pxDeg = []


    self.xServo = servo360(xServo_pin,'360')
    self.yServo = servo180(yServo_pin,'180')
    self.trigServo = servo180(trigServo_pin,'180')

  def computeAngles(self,coor1,coor2):
    angle = [(coor1[i]-coor2[i])*self.pxDeg[i] for i in range(2)]
    return angle
  
  def aim(movement):
    pass

  def shoot(self):
    pass

  def stopAndExit(self):
    if self.mode=='rpi':
      self.xServo.stop()
      self.yServo.stop()
      self.trigServo.stop()
      GPIO.cleanup()


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

  def controllRot(self):
    pass


class servo180(Servo):
  def rotateDeg(self, degrees, absolute=True):
    signal = degrees/36*2+2
    self.changeCycle()


class servo360(Servo):
  def rotateDeg(self, degrees, absolute=True):
    pass

  def controllThread(que):
    pass

  def __init__(self, servoPIN):
    super().__init__(servoPIN)


'''
if __name__ == '__main__':
  SERVO1 = 18
  SERVO2 = int()
  SERVO3 = int()

  servoPIN = 18
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(SERVO1, GPIO.OUT)

  p = GPIO.PWM(SERVO1, 50)
  p.start(2)

  serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  serversocket.bind(('0.0.0.0', 12345))
  while True:
    print('listening')
    serversocket.listen()
    sock,clientAddr = serversocket.accept()
    print('accepted')
    try:
      while True:
        received = sock.recv(1024).decode()
        print(received)
        instructions = received.split('-')
        p.ChangeDutyCycle(float(instructions[0]))

    except KeyboardInterrupt:
      print('keyboard interrupt')
      p.stop()
      GPIO.cleanup()

    except:
      print('fail, retrying..')
'''