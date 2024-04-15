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
        value = int(value)

        pwm = pwm_pins[pin]
        pwm.duty_u16(value)

    elif instruction == 'n':
        pwm = PWM(Pin(pin))
        pwm.freq(2000)
        pwm_pins.update((pin, pwm))
