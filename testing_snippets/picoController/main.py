import sys
#import mpu6050 as mpu
from machine import Pin, PWM

'''
n(byte) <pin_number>(byte) //setup pwm on pin
w(byte) <pin_number>(byte) <value>(byte string until \\n char)

'''

pwm_pins = {}

while True:

    instruction = sys.stdin.read(1)

    pin = sys.stdin.read(1)
    pin = ord(pin)
    #pin = int(pin)
    #pin = int.from_bytes(pin,'little')

    if instruction == 'w':
        value = sys.stdin.readline().strip()
        value = int(value/1e-6) # ms to ns

        pwm = pwm_pins[pin]
        pwm.duty_ns(value)

    elif instruction == 'n' and pin not in pwm_pins.keys():
        pwm = PWM(Pin(pin))
        pwm.freq(50) #2000
        pwm_pins.update((pin, pwm))
