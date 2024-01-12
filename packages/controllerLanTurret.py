import RPi.GPIO as GPIO
import time
import math
import socket
import threading

class servo():
  def __init__(self,servoPIN,servoType='180'):
    self.type = servoType
    GPIO.setup(servoPIN, GPIO.OUT)
    self.pin = GPIO.PWM(servoPIN, 50)

    if self.type == '180': 
      self.pin.start(2)
    else:
      self.pin.start(0)

  def stop(self):
    self.pin.stop()

  def changeCycle(self,value):
    self.pin.ChangeDutyCycle(value)

  def rotateRad(self,radians):
    if self.type == '180':
      signal = 

  def rotateDeg(self,degrees):
    if self.type == '180':
      signal = degrees-2

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
