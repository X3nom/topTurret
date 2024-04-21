import sys
from machine import Pin, PWM
import select

print("main running")

while True:
    ch = sys.stdin.readline()
    sys.stdout.write(ch+"x")
    