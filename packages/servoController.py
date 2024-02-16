import RPi.GPIO as GPIO
import time
import math
import socket
import threading

class Controller():
  def __init__(self,xServo,yServo,trigServo):
    self.xServo = Servo(xServo,'360')
    self.yServo = Servo(yServo,'180')
    self.trigServo = Servo(trigServo,'180')

  def computeAngle(self,coor1,coor2):
    angle = int
    return angle
  
  def shoot(self):
    self.trigServo.


class Servo():
  def __init__(self,servoPIN,servoType='180'):
    self.valRange = [2,12]
    self.type = servoType
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

  def rotateRad(self,radians,absolute=True):
    if self.type == '180':
      signal = radians*10+2

  def rotateDeg(self,degrees,absolute=True):
    if self.type == '180':
      signal = degrees/36*2+2
      self.changeCycle()

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
