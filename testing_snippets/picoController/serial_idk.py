import serial, time, sys


s = serial.Serial('/dev/ttyACM0')

s.write(b"abc\r")
while True:
    
    r = s.read(1)
    print(r)